#!/usr/bin/env python

import actionlib
import rospy

from arox_docking.msg import DockAction, UndockAction, DockGoal, UndockGoal
from arox_docking import config
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def test_loop():
    rospy.init_node('test_loop')

    robot_pose = None
    try:
        # create a new subscription to the topic, receive one message, then unsubscribe
        odom = rospy.wait_for_message("/odometry/filtered_odom", Odometry, timeout=10)
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        robot_pose = pose
    except rospy.ROSException as e:
        rospy.loginfo("problem retrieving robot pose: %s", e)

    for i in range(config.TEST_RUNS):
        rospy.loginfo("START DOCKING - UNDOCKING PROCEDURE - iteration: %s", i)

        docking_client = actionlib.SimpleActionClient('dock_to_charging_station', DockAction)
        goal = DockGoal()
        goal.goal = "custom_goal"
        docking_client.wait_for_server()
        rospy.loginfo("START DOCKING PROCEDURE..")
        docking_client.send_goal(goal)
        rospy.loginfo("goal sent, wait for accomplishment")
        # success just means that the smach execution has been successful, not the docking itself
        success = docking_client.wait_for_result()

        if success:
            rospy.loginfo("SMACH execution terminated successfully")
            rospy.loginfo("DOCKING PROCEDURE FINISHED: %s", docking_client.get_result().result_state)

            if docking_client.get_result().result_state == "success":
                rospy.loginfo("sleeping a moment..")
                rospy.sleep(10)
                undocking_client = actionlib.SimpleActionClient('undock_from_charging_station', UndockAction)
                goal = UndockGoal()
                goal.ramp_alignment_pose = robot_pose
                undocking_client.wait_for_server()
                rospy.loginfo("START UNDOCKING PROCEDURE..")
                undocking_client.send_goal(goal)
                rospy.loginfo("goal sent, wait for accomplishment")
                success = undocking_client.wait_for_result()
                if success:
                    rospy.loginfo("SMACH execution terminated successfully")
                    rospy.loginfo("UNDOCKING PROCEDURE FINISHED: %s", undocking_client.get_result().result_state)
                    if undocking_client.get_result().result_state == "failure":
                        rospy.loginfo("UNDOCKING FAILURE..")
                        break
                else:
                    rospy.loginfo("SMACH execution failed: %s", undocking_client.get_goal_status_text())
                    break
            else:
                rospy.loginfo("DOCKING FAILURE..")
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
        test_loop()
    except rospy.ROSInterruptException:
        pass
