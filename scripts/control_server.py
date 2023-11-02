#!/usr/bin/env python

import rospy

from multi_copter_cmd import MultiCopterCmd as Cmd
from multi_copter_msgs.srv import (
    Command,
    CommandResponse,
    Waypoint,
    WaypointResponse,
)
from std_msgs.msg import String


class ControlServer:
    def __init__(self, node_name):
        """
        Constructor
        """
        # Initialize node
        rospy.init_node(node_name)

        # Namespace
        self.multi_copter_ctrl = "/multi_copter_ctrl"
        # Topic name
        self.gnc_node_cmd = "/gnc_node/cmd"
        # Service name
        self.control_center = self.multi_copter_ctrl + "/control_center"
        self.waypoint_manager = self.multi_copter_ctrl + "/waypoint_manager"

        # Create ros service server
        rospy.loginfo("Create service server: {}".format(self.control_center))
        rospy.Service(self.control_center, Command, self.controlCenterCallback)
        rospy.loginfo("Create service server: {}".format(self.waypoint_manager))
        rospy.Service(self.waypoint_manager, Waypoint, self.waypointManagerCallback)

        # Create ros topic publisher
        self.gnc_node_cmd_publisher = rospy.Publisher(
            self.gnc_node_cmd, String, queue_size=10
        )

        # Waypoint list (use like a stack)
        self.wp_list = []

        rospy.loginfo("Get ready\n")

    def controlCenterCallback(self, request_msg):
        """
        Callback for control_center service
        """
        rospy.loginfo(
            "Control Center has received new command: {}".format(request_msg.cmd)
        )

        # Create response message
        response_msg = CommandResponse()
        response_msg.result = False

        # When received invalid command, early return
        if not Cmd.is_member(request_msg.cmd):
            rospy.loginfo("Control Center has received invalid command")
            rospy.loginfo("Return response to client: \n{}\n".format(response_msg))
            return response_msg

        # Publish command to multi copter
        self.publishCommand(request_msg.cmd)
        response_msg.result = True

        # Return response to client
        rospy.loginfo("Return response to client: \n{}\n".format(response_msg))
        return response_msg

    def waypointManagerCallback(self, request_msg):
        """
        Callback for waypoint_manager service
        """
        rospy.loginfo(
            "Waypoint Manager has received new command: {}".format(request_msg.cmd)
        )

        # Create response message
        response_msg = WaypointResponse()
        response_msg.result = False
        response_msg.wp.x = 0.0
        response_msg.wp.y = 0.0
        response_msg.wp.z = 0.0

        # When received invalid command, early return
        if not Cmd.is_member(request_msg.cmd):
            rospy.loginfo("Waypoint Manager has received invalid command")
            rospy.loginfo("Return response to client: \n{}\n".format(response_msg))
            return response_msg

        # When received READ command, read a waypoint from list
        if request_msg.cmd == Cmd.READ.value:
            try:
                next_wp = self.wp_list.pop()
                response_msg.result = True
                response_msg.wp.x = next_wp.x
                response_msg.wp.y = next_wp.y
                response_msg.wp.z = next_wp.z
                rospy.loginfo("Read a waypoint from list")
            except IndexError:
                rospy.loginfo("Waypoint list is empty")
        # When received WRITE command, append a waypoint to list
        elif request_msg.cmd == Cmd.WRITE.value:
            self.wp_list.append(request_msg.wp)
            response_msg.result = True
            rospy.loginfo("Append a waypoint to list")
        rospy.loginfo(
            "The number of waypoint on the list: {}".format(len(self.wp_list))
        )

        # Return response to client
        rospy.loginfo("Return response to client: \n{}\n".format(response_msg))
        return response_msg

    def publishCommand(self, cmd):
        """
        Publish command to multi copter
        """
        # Create message
        msg = String()
        msg.data = cmd

        # Publish message (send messeage)
        self.gnc_node_cmd_publisher.publish(msg)
        rospy.loginfo("Publish message to {}: {}".format(self.gnc_node_cmd, cmd))


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
