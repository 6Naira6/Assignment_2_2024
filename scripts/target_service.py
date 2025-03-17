#!/usr/bin/env python3

"""
.. module:: target_service
   :platform: Unix
   :synopsis: Python module for target service

.. moduleauthor:: Arian Tavousi

This node tracks the last received target position and provides a service to retrieve it.

**Subscribers:**
    - `/last_target` (:class:`std_msgs.msg.String`): Updates the last known target position.

**Services:**
    - `/get_last_target` (:class:`std_srvs.srv.SetBool`): Returns the last recorded target position.

Additional Details:
    This node is especially helpful in multi-node systems that need to share a “last known target”
    without duplicating state logic. By subscribing to the `/last_target` topic, it continuously
    updates its record of the current goal. Other nodes can then call the `/get_last_target` service
    to retrieve the position, which is stored as a simple string (for example, `"x, y"`). This approach
    simplifies cross-node communication and keeps the system’s architecture more modular.
"""

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String

class TargetServiceNode:
    """
    A ROS node that tracks the last received target position and provides a service to retrieve it.

    **Attributes:**
        - `last_target` (str): Stores the last received target position.

    Usage Example:
        1. Start this node:
           .. code-block:: bash

              rosrun assignment_2_2024 target_service.py

        2. Publish a new target:
           .. code-block:: bash

              rostopic pub /last_target std_msgs/String "data: '4.0, 3.0'"

        3. Request the last target:
           .. code-block:: bash

              rosservice call /get_last_target true

        You should see output like:
        .. code-block:: none

           success: True
           message: "4.0, 3.0"

    By maintaining a single reference to the latest goal, this node helps ensure that all other
    components in the system have easy, consistent access to the most current target.
    """

    def __init__(self):
        """
        Initializes the node, subscriber, and service.

        - Initializes the ROS node named `target_service_node`.
        - Subscribes to `/last_target` to receive the last target position.
        - Creates a ROS service `/get_last_target` to return the last recorded target position.

        Initially, the `last_target` is set to `"0.0, 0.0"` to indicate no valid target has
        been received if no messages arrive yet.
        """
        rospy.init_node('target_service_node')
        rospy.Subscriber('/last_target', String, self.target_callback)
        rospy.Service('get_last_target', SetBool, self.handle_get_target)
        self.last_target = "0.0, 0.0"
    
    def target_callback(self, msg):
        """
        Callback function for the `/last_target` topic.
        
        Updates the stored last target position.
        
        :param msg: The message containing the new target position.
        :type msg: std_msgs.msg.String

        The node logs every update so you can track changes in real time.
        """
        self.last_target = msg.data
        rospy.loginfo("Last target updated to: {}".format(self.last_target))
    
    def handle_get_target(self, req):
        """
        Service handler for `/get_last_target`.
        
        Returns the last known target position.
        
        :param req: The service request (unused).
        :type req: std_srvs.srv.SetBool
        :return: The last recorded target position as a service response.
        :rtype: std_srvs.srv.SetBoolResponse

        The returned `message` field contains the target in `"x, y"` format,
        or `"0.0, 0.0"` if no valid target has been received yet.
        """
        rospy.loginfo("Returning last target: {}".format(self.last_target))
        return SetBoolResponse(success=True, message=self.last_target)

if __name__ == '__main__':
    """
    Main function to initialize and run the target service node.
    
    - Creates an instance of `TargetServiceNode`.
    - Keeps the node running using `rospy.spin()`.
    - Handles `ROSInterruptException` for safe shutdown.
    """
    try:
        service_node = TargetServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Service node terminated.")
