#!/usr/bin/env python

import rospy

from multi_copter_msgs.srv import (
    Command,
    CommandResponse,
    Waypoint,
    WaypointResponse
)
from std_msgs.msg import String


class ControlServer:
    def __init__(self, node_name):
        # initialize node
        rospy.init_node(node_name)

        # namespace
        self.multi_copter_ctrl = "/multi_copter_ctrl"
        # topic name
        self.gnc_node_cmd = "/gnc_node/cmd"
        # service name
        self.control_center = self.multi_copter_ctrl + "/control_center"
        self.waypoint_manager = self.multi_copter_ctrl + "/waypoint_manager"

        # create ros service server
        rospy.loginfo("Create service server: {}".format(self.control_center))
        rospy.Service(self.control_center, Command, self.controlCenterCallback)
        rospy.loginfo("Create service server: {}".format(self.waypoint_manager))
        rospy.Service(self.waypoint_manager, Waypoint, self.waypointManagerCallback)

        # create ros topic publisher
        self.gnc_node_cmd_publisher = rospy.Publisher(
            self.gnc_node_cmd, String, queue_size=10
        )

        rospy.loginfo("Get ready\n")

    def controlCenterCallback(self, request_msg):
        rospy.loginfo(
            "Control center has received new command: {}".format(request_msg.cmd)
        )

        response_msg = CommandResponse()
        response_msg.response = "received: {}".format(request_msg.cmd)

        self.publishCommand(request_msg.cmd)

        return response_msg  # return response to client

    def waypointManagerCallback(self, request_msg):
        rospy.loginfo("Waypoint manager has received new request")

        response_msg = WaypointResponse()
        response_msg.waypoint.x = 1.0
        response_msg.waypoint.y = 2.0
        response_msg.waypoint.z = 3.0

        return response_msg  # return response to client

    def publishCommand(self, cmd):
        # create message
        msg = String()
        msg.data = cmd

        # publish message (send messeage)
        self.gnc_node_cmd_publisher.publish(msg)
        rospy.loginfo("Publish message to {}: {}".format(self.gnc_node_cmd, msg))


def main():
    node = ControlServer("control_server")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\r", end="")
        rospy.loginfo("Shutdown")
