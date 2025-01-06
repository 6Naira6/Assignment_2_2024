#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String


class TargetServiceNode:
    def __init__(self):
        rospy.init_node('target_service_node')

        rospy.Subscriber('/last_target', String, self.target_callback)

        rospy.Service('get_last_target', SetBool, self.handle_get_target)

        self.last_target = "0.0, 0.0"

    def target_callback(self, msg):
        self.last_target = msg.data
        rospy.loginfo("Last target updated to: {}".format(self.last_target))

    def handle_get_target(self, req):
        rospy.loginfo("Returning last target: {}".format(self.last_target))
        return SetBoolResponse(success=True, message=self.last_target)


if __name__ == '__main__':
    try:
        service_node = TargetServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Service node terminated.")

