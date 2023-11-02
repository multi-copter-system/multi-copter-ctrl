#!/usr/bin/env python

import rospy

from multi_copter_cmd import MultiCopterCmd as Cmd
from multi_copter_msgs.srv import (
    Command,
    CommandRequest,
    Waypoint,
    WaypointRequest,
)


class ControlClient:
    def __init__(self, node_name):
        # initialize node
        rospy.init_node(node_name)

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

        # waypoint list
        self.wp_list = []

        rospy.loginfo("Get ready")

    def callControlCenter(self, cmd, wp):
        """
        Call control center (ros service)
        """
        pass

    def callWaypointManager(self):
        """
        Call waypoint manager (ros service)
        """
        pass


def main():
    node = ControlClient('control_client')

    flag = True
    cmds_1 = 'start ready halt arming disarming begin_move finish_move begin_return finish_return'
    cmds_2 = 'read write'

    while(flag):
        rospy.loginfo('input command:')
        input = input()
        if input == 'list':
            rospy.loginfo(cmds_1 + cmds_2)
            continue
        for cmd in cmds_1.split():
            if cmd == input:
                res = node.callControlCenter(cmd)
                rospy.loginfo(res)
                continue
        if 'write' == input:
            pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\r", end="")
        rospy.loginfo("Shutdown")
