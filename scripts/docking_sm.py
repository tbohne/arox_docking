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


def main():
    rospy.init_node('docking_smach')

    sm = smach.StateMachine(outcomes=['failed', 'docked'])

    with sm:
        # add states
        smach.StateMachine.add('CONTAINER_PROXIMITY', ContainerProximity(), transitions={
            'succeeded': 'docked',
            'aborted': 'failed'
        })

    # execute SMACH
    outcome = sm.execute()
    rospy.loginfo("docking outcome: %s", outcome)


if __name__ == '__main__':
    main()
