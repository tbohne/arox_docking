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

    # success just means that the smach execution has been successful, not the docking itself
    success = smach_client.wait_for_result()

    if success:
        rospy.loginfo("SMACH execution terminated successfully")
        rospy.loginfo("result: %s", smach_client.get_result().result_state)
    else:
        rospy.loginfo("SMACH execution failed: %s", smach_client.get_goal_status_text())

    rospy.spin()


if __name__ == '__main__':
    try:
        start_smach()
    except rospy.ROSInterruptException:
        pass
