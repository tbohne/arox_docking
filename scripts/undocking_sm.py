#!/usr/bin/env python

import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from arox_docking.msg import DockAction


# initial state
class InsideContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state INSIDE_CONTAINER')
        return 'succeeded'


class DetectEntry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['detect_entry_input'],
                             output_keys=['detect_entry_output'])

        self.subscriber = rospy.Subscriber('/container_entry', PoseStamped, self.callback)
        self.received_goal = None

    def callback(self, data):
        self.received_goal = data

    def execute(self, userdata):
        rospy.loginfo('executing state DETECT_ENTRY')

        if self.received_goal:
            rospy.loginfo("detected entry: %s", self.received_goal)
            userdata.detect_container_output = self.received_goal
            return 'succeeded'
        return 'aborted'


class DriveOutOfContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state DRIVE_OUT_OF_CONTAINER')
        return 'succeeded'


class UndockingStateMachine(smach.StateMachine):
    def __init__(self):
        super(UndockingStateMachine, self).__init__(
            outcomes=['failed', 'undocked'],
            input_keys=['sm_input'],
            output_keys=['sm_output']
        )

        with self:
            # add states
            self.add('INSIDE_CONTAINER', InsideContainer(),
                     transitions={'succeeded': 'DETECT_ENTRY', 'aborted': 'failed'})

            self.add('DETECT_ENTRY', DetectEntry(),
                     transitions={'succeeded': 'DRIVE_OUT_OF_CONTAINER', 'aborted': 'DETECT_ENTRY'},)

            self.add('DRIVE_OUT_OF_CONTAINER', DriveOutOfContainer(),
                     transitions={'succeeded': 'docked', 'aborted': 'failed'})


def main():
    rospy.init_node('undocking_smach')

    sm = UndockingStateMachine()

    # construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        'undock_from_charging_station', DockAction,
        wrapped_container=sm,
        succeeded_outcomes=['undocked'],
        aborted_outcomes=['failed'],
        goal_key='sm_input',
        result_key='sm_output'
    )

    rospy.loginfo("running action server wrapper..")

    # run the server in a background thread
    asw.run_server()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
