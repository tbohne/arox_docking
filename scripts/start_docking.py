#!/usr/bin/env python

import actionlib
import rospy

from arox_docking.msg import DockAction


def start_smach():

    rospy.init_node('start_smach')

    smach_client = actionlib.SimpleActionClient('dock_to_charging_station', DockAction)
    goal_msg = DockAction().action_goal
    goal_msg.goal = "custom_goal"
    smach_client.wait_for_server()
    smach_client.send_goal(goal_msg)
    rospy.loginfo("goal sent, wait for accomplishment")
    success = smach_client.wait_for_result()
    rospy.loginfo("SMACH execution terminated successfully: %s", success)


if __name__ == '__main__':
    start_smach()
