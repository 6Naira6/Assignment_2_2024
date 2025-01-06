#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest


class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node')

        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server connected.")

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.last_target_pub = rospy.Publisher('/last_target', String, queue_size=10)

        self.current_position = None
        self.current_velocity = None

    def set_goal(self, x, y):

        self.last_target_pub.publish(f"{x}, {y}")
        rospy.loginfo("Updated last target to: x={}, y={}".format(x, y))

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo("New goal sent: x={}, y={}".format(x, y))

    def cancel_goal(self):
        self.client.cancel_goal()
        rospy.loginfo("Goal canceled.")

    def feedback_callback(self, feedback):
        rospy.loginfo("Current position from feedback: x={}, y={}".format(
            feedback.actual_pose.position.x, feedback.actual_pose.position.y))

    def odom_callback(self, data):
        self.current_position = data.pose.pose.position
        self.current_velocity = data.twist.twist.linear

        rospy.loginfo("Robot Position: x={}, y={}".format(
            self.current_position.x, self.current_position.y))
        rospy.loginfo("Robot Velocity: linear_x={}, angular_z={}".format(
            self.current_velocity.x, data.twist.twist.angular.z))

    def monitor_status(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Target reached.")
            elif state == GoalStatus.ABORTED:
                rospy.logwarn("Goal aborted by action server.")
            rate.sleep()


if __name__ == '__main__':
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

