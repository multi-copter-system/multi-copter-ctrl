#!/usr/bin/env python

import rospy

from multi_copter_msgs.srv import (
    Command,
    CommandRequest,
    Waypoint,
    WaypointRequest,
)


class ControlClient:
    def __init__(self, node_name):
        """
        Control Client
        """
        # Initialize node
        rospy.init_node(node_name)

        # Namespace
        self.multi_copter_ctrl = "/multi_copter_ctrl"
        # Service name
        self.control_center = self.multi_copter_ctrl + "/control_center"
        self.waypoint_manager = self.multi_copter_ctrl + "/waypoint_manager"

        # Create ros service client
        rospy.loginfo(
            "Creating service client: {}, wait a minute...".format(self.control_center)
        )
        rospy.wait_for_service(self.control_center)
        self.control_center_client = rospy.ServiceProxy(self.control_center, Command)
        rospy.loginfo(
            "Creating service client: {}, wait a minute...".format(
                self.waypoint_manager
            )
        )
        rospy.wait_for_service(self.waypoint_manager)
        self.waypoint_manager_client = rospy.ServiceProxy(
            self.waypoint_manager, Waypoint
        )

        rospy.loginfo("Get ready")

    def callControlCenter(self, cmd):
        """
        Call control center (ros service)
        """
        # Create message
        request_msg = CommandRequest()
        request_msg.cmd = cmd

        # Send request message and return response message
        return self.control_center_client(request_msg)

    def callWaypointManager(self, cmd, wp=(0, 0, 0)):
        """
        Call waypoint manager (ros service)
        """
        # Create message
        request_msg = WaypointRequest()
        request_msg.cmd = cmd
        request_msg.wp.x = float(wp[0])
        request_msg.wp.y = float(wp[1])
        request_msg.wp.z = float(wp[2])

        # Send request message and return response message
        return self.waypoint_manager_client(request_msg)


def main():
    node = ControlClient("control_client")

    # Send command to Control Center
    cmd = "start"
    rospy.loginfo("Send command to server: {}".format(cmd))
    ret = node.callControlCenter(cmd)
    if ret.result:
        rospy.loginfo("The command was accepted\n")
    else:
        rospy.loginfo("The command was rejected\n")

    cmd = "hoge"
    rospy.loginfo("Send command to server: {}".format(cmd))
    ret = node.callControlCenter(cmd)
    if ret.result:
        rospy.loginfo("The command was accepted\n")
    else:
        rospy.loginfo("The command was rejected\n")

    # Send command to Waypoint Manager
    cmd = "write"
    rospy.loginfo("Send command to server: {}".format(cmd))
    ret = node.callWaypointManager(cmd, (1, 2, 3))
    if ret.result:
        rospy.loginfo("The command was accepted\n")
    else:
        rospy.loginfo("The command was rejected\n")

    cmd = "read"
    rospy.loginfo("Send command to server: {}".format(cmd))
    ret = node.callWaypointManager(cmd)
    if ret.result:
        rospy.loginfo("The command was accepted")
        rospy.loginfo("Waypoint: x={}, y={}, z={}\n".format(ret.wp.x, ret.wp.y, ret.wp.z))
    else:
        rospy.loginfo("The command was rejected\n")

    cmd = "read"
    rospy.loginfo("Send command to server: {}".format(cmd))
    ret = node.callWaypointManager(cmd)
    if ret.result:
        rospy.loginfo("The command was accepted")
        rospy.loginfo("Waypoint: x={}, y={}, z={}\n".format(ret.wp.x, ret.wp.y, ret.wp.z))
    else:
        rospy.loginfo("The command was rejected\n")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\r", end="")
        rospy.loginfo("Shutdown")
