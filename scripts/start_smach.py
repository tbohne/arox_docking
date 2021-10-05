#!/usr/bin/env python

import actionlib
import rospy

from arox_docking.msg import DockAction


def start_smach():
    rospy.init_node('start_smach')

    for i in range(10):

        docking_client = actionlib.SimpleActionClient('dock_to_charging_station', DockAction)
        goal_msg = DockAction().action_goal
        goal_msg.goal = "custom_goal"
        docking_client.wait_for_server()
        rospy.loginfo("START DOCKING PROCEDURE..")
        docking_client.send_goal(goal_msg)
        rospy.loginfo("goal sent, wait for accomplishment")

        # success just means that the smach execution has been successful, not the docking itself
        success = docking_client.wait_for_result()

        if success:
            rospy.loginfo("SMACH execution terminated successfully")
            rospy.loginfo("DOCKING PROCEDURE FINISHED: %s", docking_client.get_result().result_state)

            if docking_client.get_result().result_state == "success":
                rospy.loginfo("sleeping a moment..")
                rospy.sleep(10)

                undocking_client = actionlib.SimpleActionClient('undock_from_charging_station', DockAction)
                goal_msg = DockAction().action_goal
                goal_msg.goal = "custom_goal"
                undocking_client.wait_for_server()
                rospy.loginfo("START UNDOCKING PROCEDURE..")
                undocking_client.send_goal(goal_msg)
                rospy.loginfo("goal sent, wait for accomplishment")

                success = undocking_client.wait_for_result()

                if success:
                    rospy.loginfo("SMACH execution terminated successfully")
                    rospy.loginfo("UNDOCKING PROCEDURE FINISHED: %s", undocking_client.get_result().result_state)
                else:
                    rospy.loginfo("SMACH execution failed: %s", undocking_client.get_goal_status_text())
                    break
        else:
            rospy.loginfo("SMACH execution failed: %s", docking_client.get_goal_status_text())
            break

        rospy.loginfo("###########################################################")
        rospy.loginfo("###########################################################")
        rospy.loginfo("###########################################################")
        rospy.loginfo("DOCKING - UNDOCKING PROCEDURE COMPLETED - successful iteration: %s", i)
        rospy.loginfo("###########################################################")
        rospy.loginfo("###########################################################")
        rospy.loginfo("###########################################################")
        rospy.sleep(5)

    rospy.spin()


if __name__ == '__main__':
    try:
        start_smach()
    except rospy.ROSInterruptException:
        pass
