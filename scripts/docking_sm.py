#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from arox_docking.msg import DockAction


# initial state
class ContainerProximity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state CONTAINER_PROXIMITY')
        # TODO: add gps check - correct area?
        return 'succeeded'


class DetectContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['detect_container_input'],
                             output_keys=['detect_container_output'])

        # TODO: subscribed to the 'detect_container_entry' node - check whether this is the best way to do that
        self.subscriber = rospy.Subscriber('/container_entry', PoseStamped, self.callback)
        self.received_goal = None

    def callback(self, data):
        self.received_goal = data

    def execute(self, userdata):
        rospy.loginfo('executing state DETECT_CONTAINER')

        if self.received_goal:
            rospy.loginfo("detected entry: %s", self.received_goal)
            userdata.detect_container_output = self.received_goal
            return 'succeeded'
        return 'aborted'


class AlignRobotToRamp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['align_robot_to_ramp_input'],
                             output_keys=['align_robot_to_ramp_output'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state ALIGN_ROBOT_TO_RAMP')

        if userdata.align_robot_to_ramp_input:
            rospy.loginfo("got userdata: %s", userdata.align_robot_to_ramp_input)

            pose_stamped = userdata.align_robot_to_ramp_input
            move_base_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
            move_base_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = pose_stamped.pose.position
            goal.target_pose.pose.orientation = pose_stamped.pose.orientation

            move_base_client.send_goal(goal)
            rospy.loginfo("now waiting...")
            move_base_client.wait_for_result()
            rospy.loginfo("after wait: %s", move_base_client.get_result())
            return 'succeeded'

        return 'aborted'


class DriveIntoContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state DRIVE_INTO_CONTAINER')
        return 'succeeded'


class LocalizeChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state LOCALIZE_CHARGING_STATION')
        return 'succeeded'


class AlignRobotToChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state ALIGN_ROBOT_TO_CHARGING_STATION')
        return 'succeeded'


class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state DOCK')
        return 'succeeded'


class DockingStateMachine(smach.StateMachine):
    def __init__(self):
        super(DockingStateMachine, self).__init__(
            outcomes=['failed', 'docked'],
            input_keys=['sm_input'],
            output_keys=[]
        )

        with self:
            # add states
            self.add('CONTAINER_PROXIMITY', ContainerProximity(), transitions={
                'succeeded': 'DETECT_CONTAINER',
                'aborted': 'failed'})

            self.add('DETECT_CONTAINER', DetectContainer(),
                     transitions={'succeeded': 'ALIGN_ROBOT_TO_RAMP',
                                  'aborted': 'DETECT_CONTAINER'},
                     remapping={'detect_container_output': 'sm_input'})

            self.add('ALIGN_ROBOT_TO_RAMP', AlignRobotToRamp(),
                     transitions={'succeeded': 'DRIVE_INTO_CONTAINER',
                                  'aborted': 'failed'},
                     remapping={'align_robot_to_ramp_input': 'sm_input'})

            self.add('DRIVE_INTO_CONTAINER', DriveIntoContainer(), transitions={
                'succeeded': 'LOCALIZE_CHARGING_STATION',
                'aborted': 'failed'})

            self.add('LOCALIZE_CHARGING_STATION', LocalizeChargingStation(), transitions={
                'succeeded': 'ALIGN_ROBOT_TO_CHARGING_STATION',
                'aborted': 'failed'})

            self.add('ALIGN_ROBOT_TO_CHARGING_STATION', AlignRobotToChargingStation(), transitions={
                'succeeded': 'DOCK',
                'aborted': 'failed'})

            self.add('DOCK', Dock(), transitions={
                'succeeded': 'docked',
                'aborted': 'failed'})


def main():
    rospy.init_node('docking_smach')

    sm = DockingStateMachine()

    # construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        'dock_to_charging_station', DockAction,
        wrapped_container=sm,
        succeeded_outcomes=['docked'],
        aborted_outcomes=['aborted'],
        goal_key='sm_input',
        # result_key='result'
    )

    rospy.loginfo("running action server wrapper..")

    # run the server in a background thread
    asw.run_server()

    rospy.spin()


if __name__ == '__main__':
    main()
