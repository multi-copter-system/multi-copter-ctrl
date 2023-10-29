#!/usr/bin/env python

import rospy

from multi_copter_msgs.srv import (
    Command,
    CommandRequest,
    Waypoint,
    WaypointRequest,
)


class ControlClientSample:
    def __init__(self, node_name):
        # initialize node
        rospy.init_node(node_name, anonymous=True)

        # namespace
        self.multi_copter_ctrl = "/multi_copter_ctrl"
        # service name
        self.control_center = self.multi_copter_ctrl + "/control_center"
        self.waypoint_manager = self.multi_copter_ctrl + "/waypoint_manager"

        # create ros service client
        rospy.loginfo(
            "Creating service client: {}, wait a minute...".format(self.control_center)
        )
        rospy.wait_for_service(self.control_center)
        self.client_for_control_center = rospy.ServiceProxy(
            self.control_center, Command
        )

        rospy.loginfo(
            "Creating service client: {}, wait a minute...".format(
                self.waypoint_manager
            )
        )
        rospy.wait_for_service(self.waypoint_manager)
        self.client_for_waypoint_manager = rospy.ServiceProxy(
            self.waypoint_manager, Waypoint
        )

        rospy.loginfo("Get ready")

    def callControlCenter(self, cmd):
        # create request message
        msg = CommandRequest()
        msg.cmd = cmd
        msg.waypoint.x = 0.0
        msg.waypoint.y = 0.0
        msg.waypoint.z = 0.0

        # send request message / receive response message
        response_msg = self.client_for_control_center(msg)

        return response_msg  # return response message

    def callWaypointManager(self):
        # create request message
        request_msg = WaypointRequest()
        request_msg.request = ""

        # send request message / receive response message
        response_msg = self.client_for_waypoint_manager(request_msg)

        return response_msg  # return response message


def main():
    node = ControlClientSample("control_client_sample")

    # call control_center service
    rospy.loginfo("call control_center service")
    cmd = "start"
    ret = node.callControlCenter(cmd)
    rospy.loginfo("send: {}".format(cmd))
    rospy.loginfo("receive: {}".format(ret))

    # call waypoint_manager service
    rospy.loginfo("call waypoint_manager service")
    ret = node.callWaypointManager()
    rospy.loginfo("receive: {}".format(ret))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\r", end="")
        rospy.loginfo("Shutdown")
