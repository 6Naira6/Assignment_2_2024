#!/usr/bin/env python3

"""
.. module:: go_to_point
   :platform: Unix
   :synopsis: Python module for navigating a robot to a specified goal position

.. moduleauthor:: Arian Tavousi

This node enables a robot to navigate towards a predefined goal position using odometry data.
It uses a state machine approach to:
    1) Rotate the robot to face the goal.
    2) Move straight towards the goal.
    3) Stop upon reaching the goal.

**Subscribers:**
    - `/odom` (:class:`nav_msgs.msg.Odometry`): Receives odometry data to determine the robot's position and orientation.

**Publishers:**
    - `/cmd_vel` (:class:`geometry_msgs.msg.Twist`): Publishes velocity commands to control the robot.

**Services:**
    - `/go_to_point_switch` (:class:`std_srvs.srv.SetBool`): Activates or deactivates the navigation behavior.

Additional Details:
    This node operates as a minimal navigation controller. By defining a desired (x, y) position:
    - The robot first rotates in place until its yaw aligns with the goal.
    - Then it proceeds forward, continuously correcting its course.
    - Finally, once within a specified distance threshold, it stops.

    Users can enable or disable this behavior by calling the `/go_to_point_switch` service.
    When inactive, the node does nothing and the robot remains idle.
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import SetBool, SetBoolResponse
import math

# Global Variables
active_ = False  # Flag to activate or deactivate the navigation behavior
position_ = Point()  # Current position of the robot
yaw_ = 0  # Current orientation (yaw) of the robot
state_ = 0  # State machine variable (0: Rotate to goal, 1: Move forward, 2: Stop at goal)

desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

# Precision and Control Parameters
yaw_precision_ = math.pi / 9   # Allowed yaw error (~20 degrees)
yaw_precision_2_ = math.pi / 90  # Fine-tuned yaw precision (~2 degrees)
dist_precision_ = 0.3           # Distance threshold to goal

kp_a = 3.0   # Proportional gain for angular control
kp_d = 0.2   # Proportional gain for linear control
ub_a = 0.6   # Upper bound for angular velocity
lb_a = -0.5  # Lower bound for angular velocity
ub_d = 0.6   # Upper bound for linear velocity

pub = None   # Publisher for velocity commands

def go_to_point_switch(req):
    """
    Service callback to enable or disable the go-to-point behavior.
    
    :param req: Service request containing a boolean value indicating whether to activate the node.
    :type req: std_srvs.srv.SetBoolRequest
    :return: Response confirming the operation.
    :rtype: std_srvs.srv.SetBoolResponse

    When True, the node begins rotating and moving toward the specified goal.
    When False, the node stops controlling the robot's movement.
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_odom(msg):
    """
    Callback function for odometry data.
    
    Updates the robot's position and orientation.
    
    :param msg: Odometry message providing the robot’s current pose.
    :type msg: nav_msgs.msg.Odometry

    The yaw angle is extracted from the quaternion to facilitate directional calculations.
    """
    global position_, yaw_
    position_ = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    Updates the robot's state if it has changed.
    
    :param state: The new state (0 -> rotate to goal, 1 -> go straight, 2 -> goal reached).
    :type state: int

    Logs the state transition for debugging.
    """
    global state_
    state_ = state
    rospy.loginfo('State changed to [%s]', state_)


def normalize_angle(angle):
    """
    Normalizes an angle to the range [-pi, pi].
    
    :param angle: Input angle in radians.
    :type angle: float
    :return: Normalized angle.
    :rtype: float

    This helps prevent large jumps in angular calculations
    when crossing the +/- pi boundary.
    """
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / math.fabs(angle)
    return angle


def fix_yaw(des_pos):
    """
    Adjusts the robot's yaw to face the goal position.
    
    :param des_pos: Desired goal position.
    :type des_pos: geometry_msgs.msg.Point

    If the error in yaw is greater than a fine threshold (yaw_precision_2_),
    the robot rotates with a limited angular speed. Once the yaw is aligned,
    the node transitions to the forward-movement state.
    """
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    
    # Rotate if the yaw error is above the fine threshold
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a * err_yaw
        twist_msg.angular.z = max(min(twist_msg.angular.z, ub_a), lb_a)
    pub.publish(twist_msg)
    
    # Transition to the next state once yaw is sufficiently aligned
    if math.fabs(err_yaw) <= yaw_precision_2_:
        rospy.loginfo('Yaw error within threshold: [%s]', err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    Moves the robot straight towards the goal position.
    
    :param des_pos: Desired goal position.
    :type des_pos: geometry_msgs.msg.Point

    If the robot is still far from the goal, it applies
    proportional linear and angular velocity. Once the robot
    is within a distance threshold, it switches to the 'done' state.
    If the yaw error becomes large, it switches back to yaw-correction.
    """
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt((des_pos.y - position_.y)**2 + (des_pos.x - position_.x)**2)
    
    # If the robot is not close enough to the goal, keep moving forward
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = min(kp_d * err_pos, ub_d)
        twist_msg.angular.z = kp_a * err_yaw
        pub.publish(twist_msg)
    else:
        rospy.loginfo('Position error within threshold: [%s]', err_pos)
        change_state(2)
    
    # If the yaw error grows too large, go back to rotate state
    if math.fabs(err_yaw) > yaw_precision_:
        rospy.loginfo('Yaw error exceeded threshold: [%s]', err_yaw)
        change_state(0)


def done():
    """
    Stops the robot once the goal is reached.

    Publishes a zeroed Twist message, effectively halting linear and angular motion.
    """
    twist_msg = Twist()
    pub.publish(twist_msg)


def main():
    """
    Main function to initialize the node and manage robot movement.

    1. Advertises the `/go_to_point_switch` service to toggle navigation.
    2. Subscribes to `/odom` to update position and yaw.
    3. Runs a state machine at 20 Hz:
       - State 0: Rotate until yaw aligned with goal.
       - State 1: Move forward until within distance threshold of goal (or yaw drifts).
       - State 2: Stop the robot.
    """
    global pub, active_
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # Only operate if the service is set to active
        if not active_:
            rate.sleep()
            continue
        # State machine logic
        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
        else:
            rospy.logerr('Unknown state!')
        rate.sleep()

if __name__ == '__main__':
    main()
