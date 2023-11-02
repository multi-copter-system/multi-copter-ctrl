#!/usr/bin/env python

import rospy

from multi_copter_cmd import MultiCopterCmd as Cmd
from multi_copter_msgs.srv import (
    Command,
    CommandResponse,
    Waypoint,
    WaypointResponse
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
        self.multi_copter_ctrl = '/multi_copter_ctrl'
        # Topic name
        self.gnc_node_cmd = '/gnc_node/cmd'
        # Service name
        self.control_center = self.multi_copter_ctrl + '/control_center'
        self.waypoint_manager = self.multi_copter_ctrl + '/waypoint_manager'

        # Create ros service server
        rospy.loginfo('Create service server: {}'.format(self.control_center))
        rospy.Service(self.control_center, Command, self.controlCenterCallback)
        rospy.loginfo('Create service server: {}'.format(self.waypoint_manager))
        rospy.Service(self.waypoint_manager, Waypoint, self.waypointManagerCallback)

        # Create ros topic publisher
        self.gnc_node_cmd_publisher = rospy.Publisher(
            self.gnc_node_cmd, String, queue_size=10
        )

        # Waypoint list (use like a stack)
        self.wp_list = []

        rospy.loginfo('Get ready\n')

    def controlCenterCallback(self, request_msg):
        """
        Callback for control_center service
        """
        rospy.loginfo(
            'Control Center has received new command: {}'.format(request_msg.cmd)
        )

        # Create response message
        response_msg = CommandResponse()

        if Cmd.is_member(request_msg.cmd):
            pass
        # response_msg = CommandResponse()
        # response_msg.result = False

        # if Cmd.is_member(request_msg.cmd):
        #     send_cmd = request_msg.cmd
        #     response_msg.response = 'Accept'
        # elif request_msg.cmd == Cmd.write():
        #     wp = WaypointResponse()
        #     wp.waypoint.x = request_msg.waypoint.x
        #     wp.waypoint.y = request_msg.waypoint.y
        #     wp.waypoint.z = request_msg.waypoint.z
        #     self.waypoints.append(wp)
        #     response_msg.response = f'write waypoint ({len(self.waypoints)})'
        #     return response_msg
        # else:
        #     rospy.loginfo('unknow command')
        #     return response_msg

        # response_msg.response = f'send command: {send_cmd}'
        # self.publishCommand(send_cmd)

        # return response_msg  # return response to client

    def waypointManagerCallback(self, request_msg):
        """
        Callback for waypoint_manager service
        """
        rospy.loginfo(
            'Waypoint Manager has received new command: {}'.format(request_msg.cmd)
        )

        # Create response message
        response_msg = WaypointResponse()
        response_msg.result = False
        response_msg.waypoint.x = 0.0
        response_msg.waypoint.y = 0.0
        response_msg.waypoint.z = 0.0

        # When received invalid command, early return
        if not Cmd.is_member(request_msg.cmd):
            return response_msg

        # When received READ command, read a waypoint from list
        if request_msg.cmd == Cmd.READ.value:
            pass
        # When received WRITE command, append a waypoint to list
        elif request_msg.cmd == Cmd.WRITE.value:
            pass

        # Return response to client
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
        rospy.loginfo('Publish message to {}: {}'.format(self.gnc_node_cmd, cmd))


def main():
    node = ControlServer('control_server')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print('\r', end='')
        rospy.loginfo('Shutdown')
