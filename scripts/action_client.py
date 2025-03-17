#!/usr/bin/env python3

"""
.. module:: action_client_node
   :platform: Unix
   :synopsis: Python module for an action client controlling a robot

.. moduleauthor:: Arian Tavousi

This ROS node acts as an action client, sending navigation goals to an action server
and monitoring the robot's position and velocity based on odometry data.

**Subscribers:**
    - `/odom` (:class:`nav_msgs.msg.Odometry`): Receives odometry data to track the robot's position and velocity.

**Publishers:**
    - `/last_target` (:class:`std_msgs.msg.String`): Publishes the last navigation goal coordinates.

**Action Clients:**
    - `/reaching_goal` (:class:`assignment_2_2024.msg.PlanningAction`): Sends target positions to the action server.

Additional Details:
    The node allows command-based interaction in the console. Users can type a command
    like ``set 2 3`` to send a goal at coordinates (2, 3) or ``cancel`` to abort the current goal.
    The node then reports the latest robot pose via odometry updates and logs feedback from the
    action server, helping users track task progress in real time.

    To run this node, ensure that:
        - The ROS Master is running (e.g., via `roscore`).
        - The action server (`/reaching_goal`) is available.
        - The `/odom` topic publishes odometry data.

    Example usage:
        .. code-block:: bash

           rosrun assignment_2_2024 action_client_node.py

        Then in the same terminal:
        .. code-block:: none

           Enter command (set x y / cancel): set 5 5
           [Feedback in the console...]
           Enter command (set x y / cancel): cancel

    This node is valuable in interactive or testing scenarios where a user or script
    needs to dynamically command the robot’s position.
"""

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

class ActionClientNode:
    """
    A ROS action client node that sends navigation goals, monitors the robot's status,
    and logs real-time position and velocity updates.

    **Attributes**:
        - `client` (:class:`actionlib.SimpleActionClient`): The action client used to send goals.
        - `last_target_pub` (:class:`rospy.Publisher`): Publishes the last target coordinate string.
        - `current_position` (:class:`geometry_msgs.msg.Point`): Tracks the robot's current position.
        - `current_velocity` (:class:`geometry_msgs.msg.Vector3`): Tracks the robot's current linear velocity.
    """

    def __init__(self):
        """
        Initializes the action client node.
        - Connects to the `/reaching_goal` action server.
        - Subscribes to `/odom` to track the robot's movement.
        - Publishes the last target position on `/last_target`.

        This setup ensures the node receives continuous updates of the robot’s
        position while providing an interface for sending/canceling navigation goals.
        """
        rospy.init_node('action_client_node')

        # Initialize action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server connected.")

        # Subscriber for odometry updates
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher for the last target position
        self.last_target_pub = rospy.Publisher('/last_target', String, queue_size=10)

        # Internal variables
        self.current_position = None
        self.current_velocity = None

    def set_goal(self, x, y):
        """
        Sends a new navigation goal to the action server.
        
        :param x: X-coordinate of the goal.
        :type x: float
        :param y: Y-coordinate of the goal.
        :type y: float

        Once sent, the goal is monitored by the action server, and this node logs
        real-time feedback of the robot’s progress. The method also publishes
        the target on the `/last_target` topic, which can be used by other nodes
        or services to keep track of the most recent requested goal.
        """
        self.last_target_pub.publish(f"{x}, {y}")
        rospy.loginfo("Updated last target to: x={}, y={}".format(x, y))

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo("New goal sent: x={}, y={}".format(x, y))

    def cancel_goal(self):
        """
        Cancels the current goal sent to the action server.

        Useful if the robot is directed to an incorrect or
        unreachable destination, or if the user decides
        to reassign the task.
        """
        self.client.cancel_goal()
        rospy.loginfo("Goal canceled.")

    def feedback_callback(self, feedback):
        """
        Callback function for action feedback, logging the robot's current position.
        
        :param feedback: Feedback containing the actual position of the robot.
        :type feedback: assignment_2_2024.msg.PlanningFeedback

        Called by the action client whenever partial progress
        or position updates occur. Displays the current x, y of the robot.
        """
        rospy.loginfo("Current position from feedback: x={}, y={}".format(
            feedback.actual_pose.position.x, feedback.actual_pose.position.y))

    def odom_callback(self, data):
        """
        Callback function for odometry data, updating the robot's position and velocity.
        
        :param data: Odometry message containing position and velocity information.
        :type data: nav_msgs.msg.Odometry

        Publishes log messages indicating the robot’s position (x, y) and velocity.
        These logs can be monitored to see the robot’s motion in real time.
        """
        self.current_position = data.pose.pose.position
        self.current_velocity = data.twist.twist.linear

        rospy.loginfo("Robot Position: x={}, y={}".format(
            self.current_position.x, self.current_position.y))
        rospy.loginfo("Robot Velocity: linear_x={}, angular_z={}".format(
            self.current_velocity.x, data.twist.twist.angular.z))

    def monitor_status(self):
        """
        Continuously monitors the status of the goal and logs updates on its completion or failure.

        By default, this runs at 1 Hz. If the action server reports the goal as
        SUCCEEDED, it logs a success message; if the goal is ABORTED, a warning is shown.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Target reached.")
            elif state == GoalStatus.ABORTED:
                rospy.logwarn("Goal aborted by action server.")
            rate.sleep()

if __name__ == '__main__':
    """
    Main function to instantiate and operate the ActionClientNode.

    The script prompts the user for commands to set or cancel goals until
    the node is shut down. Use "set x y" to specify a new goal, or "cancel"
    to abort the current one.

    Example:
        Enter command (set x y / cancel): set 2 3
        [Logs from feedback...]
        Enter command (set x y / cancel): cancel

    The node terminates on a CTRL+C or after a rospy.ROSInterruptException.
    """
    try:
        client = ActionClientNode()
        while not rospy.is_shutdown():
            command = input("Enter command (set x y / cancel): ")
            if command.startswith("set"):
                _, x, y = command.split()
                client.set_goal(float(x), float(y))
            elif command == "cancel":
                client.cancel_goal()
            else:
                rospy.logwarn("Invalid command.")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client terminated.")
