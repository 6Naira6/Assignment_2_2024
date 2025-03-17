#!/usr/bin/env python3

"""
.. module:: wall_follow_service
   :platform: Unix
   :synopsis: Python module for wall-following behavior in a robot

.. moduleauthor:: Arian Tavousi

This node enables a robot to follow walls using laser scan data.

**Subscribers:**
    - `/scan` (:class:`sensor_msgs.msg.LaserScan`): Receives laser scan data.

**Publishers:**
    - `/cmd_vel` (:class:`geometry_msgs.msg.Twist`): Publishes velocity commands to control the robot.

**Services:**
    - `/wall_follower_switch` (:class:`std_srvs.srv.SetBool`): Activates or deactivates the wall-following behavior.

Additional Details:
    The wall follower maintains three states:
      0. **find the wall** – Moves forward while turning right gently, seeking a wall to follow.
      1. **turn left** – Used when the robot faces an obstacle or reaches a corner.
      2. **follow the wall** – Moves forward, hugging the wall at a safe distance.
    
    By monitoring the minimum distances in specific sectors (front, fright, fleft, etc.), 
    the robot can navigate along walls or avoid obstacles. Activating and deactivating 
    wall-following is managed through the `/wall_follower_switch` service.
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
import math

active_ = False

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    """
    Service callback to enable or disable the wall-following behavior.
    
    :param req: Service request containing a boolean value.
    :type req: std_srvs.srv.SetBoolRequest
    :return: Response confirming the operation.
    :rtype: std_srvs.srv.SetBoolResponse

    If True, the node starts reading laser scans and controlling the robot’s velocity
    to maintain a wall-following strategy. If False, the robot stops.
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    """
    Callback function for laser scan data.
    
    Updates `regions_` with the minimum distances in different directions and triggers `take_action()`.
    
    :param msg: Laser scan message.
    :type msg: sensor_msgs.msg.LaserScan

    Sectors are defined as follows (angles approximate, for a 720-range scan):
      - right:   [0..143]
      - fright: [144..287]
      - front:  [288..431]
      - fleft:  [432..575]
      - left:   [576..713]
    """
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    take_action()

def change_state(state):
    """
    Updates the robot's state if it has changed.
    
    :param state: The new state (0 -> find the wall, 1 -> turn left, 2 -> follow the wall).
    :type state: int

    Logs a message to indicate the new state for debug purposes.
    """
    global state_, state_dict_
    if state != state_:
        rospy.loginfo('Wall follower - [%s] - %s', state, state_dict_[state])
        state_ = state

def take_action():
    """
    Determines the appropriate action based on the detected obstacles and changes the robot's state accordingly.

    This function checks the distances in front, fright, and fleft to decide:
      - If there's a clear path ahead and on sides, it goes to state 0 (find the wall).
      - If the front is blocked, it turns left (state 1).
      - If the fright is too close, it follows the wall (state 2).
      - If the fleft is too close, also revert to find wall (state 0).
    """
    global regions_
    regions = regions_
    d0 = 1   # Threshold for immediate obstacle in front
    d  = 1.5 # Threshold for deciding whether to follow or turn

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        change_state(0)
    elif regions['front'] < d0:
        change_state(1)
    elif regions['fright'] < d:
        change_state(2)
    elif regions['fleft'] < d:
        change_state(0)
    else:
        rospy.loginfo("Unknown case: %s", regions)

def find_wall():
    """
    Generates a velocity command to move forward while turning right slightly.
    
    :return: Twist message to move the robot.
    :rtype: geometry_msgs.msg.Twist

    Used when no walls are detected close by.
    """
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    """
    Generates a velocity command to turn left in place.
    
    :return: Twist message to turn the robot.
    :rtype: geometry_msgs.msg.Twist

    Typically triggered when the robot detects an obstacle ahead.
    """
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_the_wall():
    """
    Generates a velocity command to follow the wall by moving forward.
    
    :return: Twist message to move forward.
    :rtype: geometry_msgs.msg.Twist

    Designed to keep the robot parallel to the wall on its right, 
    adjusted by the distance thresholds in `take_action()`.
    """
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    """
    Main function to initialize the node and manage robot movement.

    Steps:
      1. Initializes the ROS node named `reading_laser`.
      2. Subscribes to `/scan` for laser data.
      3. Advertises the `/wall_follower_switch` service for toggling wall-following.
      4. Runs a loop at ~20 Hz:
         - If disabled, does nothing.
         - If enabled, checks the current state and calls the appropriate function to control motion.
    """
    global pub_, active_
    rospy.init_node('reading_laser')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')
            pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
