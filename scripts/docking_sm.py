#!/usr/bin/env python

import rospy
import smach
import smach_ros


# initial state
class ContainerProximity(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state CONTAINER_PROXIMITY')
        return 'succeeded'


class DetectContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state DETECT_CONTAINER')
        return 'succeeded'


class AlignRobotToRamp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state ALIGN_ROBOT_TO_RAMP')
        return 'succeeded'


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


def main():
    rospy.init_node('docking_smach')

    sm = smach.StateMachine(outcomes=['failed', 'docked'])

    with sm:
        # add states
        smach.StateMachine.add('CONTAINER_PROXIMITY', ContainerProximity(), transitions={
            'succeeded': 'DETECT_CONTAINER',
            'aborted': 'failed'})

        smach.StateMachine.add('DETECT_CONTAINER', DetectContainer(), transitions={
            'succeeded': 'ALIGN_ROBOT_TO_RAMP',
            'aborted': 'failed'})

        smach.StateMachine.add('ALIGN_ROBOT_TO_RAMP', AlignRobotToRamp(), transitions={
            'succeeded': 'DRIVE_INTO_CONTAINER',
            'aborted': 'failed'})

        smach.StateMachine.add('DRIVE_INTO_CONTAINER', DriveIntoContainer(), transitions={
            'succeeded': 'LOCALIZE_CHARGING_STATION',
            'aborted': 'failed'})

        smach.StateMachine.add('LOCALIZE_CHARGING_STATION', LocalizeChargingStation(), transitions={
            'succeeded': 'ALIGN_ROBOT_TO_CHARGING_STATION',
            'aborted': 'failed'})

        smach.StateMachine.add('ALIGN_ROBOT_TO_CHARGING_STATION', AlignRobotToChargingStation(), transitions={
            'succeeded': 'DOCK',
            'aborted': 'failed'})

        smach.StateMachine.add('DOCK', Dock(), transitions={
            'succeeded': 'docked',
            'aborted': 'failed'})

    # execute SMACH
    outcome = sm.execute()
    rospy.loginfo("docking outcome: %s", outcome)


if __name__ == '__main__':
    main()
