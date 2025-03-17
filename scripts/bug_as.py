#! /usr/bin/env python3

"""
.. module:: bug0
   :platform: Unix
   :synopsis: Python module implementing a bug0 approach to navigation

.. moduleauthor:: Arian Tavousi

This ROS node implements a bug0-based navigation strategy. It alternates between
driving straight to a goal and following walls when obstacles are detected.

**Subscribers:**
    - `/scan` (:class:`sensor_msgs.msg.LaserScan`): Receives laser scan data to identify obstacles.
    - `/odom` (:class:`nav_msgs.msg.Odometry`): Receives odometry data to track the robot's position and orientation.

**Publishers:**
    - `/cmd_vel` (:class:`geometry_msgs.msg.Twist`): Publishes velocity commands to move the robot.

**Services:**
    - `/go_to_point_switch` (:class:`std_srvs.srv.SetBool`): Toggles the go-to-point functionality.
    - `/wall_follower_switch` (:class:`std_srvs.srv.SetBool`): Toggles the wall-following functionality.

**Action Server:**
    - `/reaching_goal` (:class:`assignment_2_2024.msg.PlanningAction`): Receives target positions for navigation.

Additional Details:
    This node embodies a “Bug0” approach to navigation: 
    1) The robot attempts to move in a direct line toward its goal.
    2) If there is an obstacle directly in front, it switches to wall-following behavior.
    3) Once it can head straight for the goal again, it switches back to go-to-point mode. 

    This simple yet effective reactive algorithm helps navigate cluttered environments 
    without complex mapping or planning, making it suitable for real-time robotics scenarios.
"""

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2024.msg
from tf import transformations
from std_srvs.srv import *
import time

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
pose_ = Pose()
desired_position_ = Point()
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'done']
state_ = 0
# 0 - go to point
# 1 - wall following
# 2 - done
# 3 - canceled

def clbk_odom(msg):
    """
    Callback for the /odom subscriber.
    
    Updates the global position_, pose_, and yaw_ variables based on odometry data.

    :param msg: The Odometry message containing the robot's current position and orientation.
    :type msg: nav_msgs.msg.Odometry

    The orientation (yaw) is extracted from the quaternion in the Odometry message and used
    to guide directional checks during navigation.
    """
    global position_, yaw_, pose_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    """
    Callback for the /scan subscriber.

    Stores the minimum distances in different regions around the robot.

    :param msg: The LaserScan message containing distance readings.
    :type msg: sensor_msgs.msg.LaserScan

    The node splits the laser range array into slices (right, fright, front, fleft, left),
    each limited to a maximum distance of 10. This helps the robot decide whether to proceed
    forward or switch to wall-following mode.
    """
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

def change_state(state):
    """
    Changes the current navigation state and toggles the respective services.
    
    :param state: The state to switch to (0 -> go to point, 1 -> wall following, 2 -> done).
    :type state: int

    Depending on the state:
    - **Go to point**: Calls `/go_to_point_switch` with True; `/wall_follower_switch` with False.
    - **Wall following**: Calls `/go_to_point_switch` with False; `/wall_follower_switch` with True.
    - **Done**: Disables both to stop movement.
    """
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)

def normalize_angle(angle):
    """
    Normalizes the given angle to be within [-pi, pi].

    :param angle: The angle to normalize (in radians).
    :type angle: float
    :return: The normalized angle.
    :rtype: float

    This utility helps compare headings by preventing large jumps 
    when angles cross the +pi/-pi boundary.
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    
def done():
    """
    Publishes a zero velocity command to stop the robot.

    Typically invoked when the goal is reached or canceled,
    ensuring the robot remains stationary.
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    
def planning(goal):
    """
    Action server callback for handling navigation goals.
    
    Drives the robot towards the desired position, switching to wall-following when needed.

    :param goal: The navigation goal containing the target pose.
    :type goal: assignment_2_2024.msg.PlanningGoal

    Steps:
    1. Set initial state to "go to point".
    2. Continuously check the distance to the goal.
    3. If the robot is close, stop the action as "succeeded".
    4. If there's an obstacle, switch to wall following.
    5. If the obstacle is cleared, switch back to go to point.
    6. Handle any preemption requests (cancellation).

    During execution, feedback is published so other nodes or tools can track the robot’s status.
    """
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pose_
    change_state(0)
    rate = rospy.Rate(20)
    success = True
    
    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    
    feedback = assignment_2_2024.msg.PlanningFeedback()
    result = assignment_2_2024.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) +
                            pow(desired_position_.x - position_.x, 2))
        if act_s.is_preempt_requested():
            rospy.loginfo("Goal was preeempted")
            feedback.stat = "Target cancelled!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            act_s.set_preempted()
            success = False
            change_state(2)
            done()
            break
        elif err_pos < 0.5:
            change_state(2)
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break       
        elif regions_ == None:
            continue
        
        elif state_ == 0:
            feedback.stat = "State 0: go to point"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            if regions_['front'] < 0.2:
                change_state(1)
        elif state_ == 1:
            feedback.stat = "State 1: avoid obstacle"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                change_state(0)
        elif state_ == 2:
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)

def main():
    """
    Initializes the node, sets up subscriptions, publishers, and services,
    and starts the action server for receiving navigation goals.

    The node runs at ~20 Hz, continuously checking whether it should go
    forward or switch to wall following. On receiving action goals via
    `/reaching_goal`, it heads toward the target while reacting to obstacles.
    """
    time.sleep(2)
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_, act_s, pub

    rospy.init_node('bug0')
    
    desired_position_.x = 0.0
    desired_position_.y = 1.0
    rospy.set_param('des_pos_x', desired_position_.x)
    rospy.set_param('des_pos_y', desired_position_.y)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    act_s = actionlib.SimpleActionServer('/reaching_goal',
                                         assignment_2_2024.msg.PlanningAction,
                                         planning,
                                         auto_start=False)
    act_s.start()
   
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == "__main__":
    main()
