#!/usr/bin/env python

import rospy
import sys

from multi_copter_cmd import MultiCopterCmd as Cmd
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

        # Service name
        self.control_center = '/control_center'
        self.waypoint_manager = '/waypoint_manager'

        # Create ros service client
        rospy.loginfo(
            'Creating service client: {}, wait a minute...'.format(self.control_center)
        )
        rospy.wait_for_service(self.control_center)
        self.control_center_client = rospy.ServiceProxy(self.control_center, Command)
        rospy.loginfo(
            'Creating service client: {}, wait a minute...'.format(
                self.waypoint_manager
            )
        )
        rospy.wait_for_service(self.waypoint_manager)
        self.waypoint_manager_client = rospy.ServiceProxy(
            self.waypoint_manager, Waypoint
        )

        rospy.loginfo('Get ready')

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
    node = ControlClient('control_client')

    while True:
        print('Input command:', file=sys.stderr)
        print(
            'arming | halt | begin_move | begin_return | write | read | end (end means exit this program)',
            file=sys.stderr,
        )
        input_from_stdin = input()

        if input_from_stdin == 'end':
            break
        if not Cmd.is_member(input_from_stdin):
            print('Invalid command\n', file=sys.stderr)
            continue

        if input_from_stdin == 'write':
            print('Input waypoint:', file=sys.stderr)
            print('x=', end='', file=sys.stderr)
            wp_x = input()
            print('y=', end='', file=sys.stderr)
            wp_y = input()
            print('z=', end='', file=sys.stderr)
            wp_z = input()
            ret = node.callWaypointManager('write', (wp_x, wp_y, wp_z))
        elif input_from_stdin == 'read':
            ret = node.callWaypointManager('read')
        else:
            ret = node.callControlCenter(input_from_stdin)

        print('{}\n'.format(ret), file=sys.stderr)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print('\r', end='')
        rospy.loginfo('Shutdown')