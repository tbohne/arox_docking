#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from arox_docking.msg import DockAction
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

TF_BUFFER = None


def transform_pose(pose_stamped, target_frame):
    global TF_BUFFER

    try:
        transform = TF_BUFFER.lookup_transform(target_frame,
                                               pose_stamped.header.frame_id,  # source frame
                                               rospy.Time(0),  # get tf at first available time
                                               rospy.Duration(1.0))  # wait for one second

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Exception while trying to transform pose stamped from %s to %s", pose_stamped.header.frame_id,
              target_frame)
        raise

    return pose_transformed


# initial state
class InsideContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state INSIDE_CONTAINER')
        # TODO: implement check whether robot is actually inside the container
        return 'succeeded'


class DetectEntry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['detect_entry_input'],
                             output_keys=['detect_entry_output'])

        self.received_goal = None
        self.subscriber = None

    def callback(self, data):
        self.received_goal = data

    def execute(self, userdata):
        rospy.loginfo('executing state DETECT_ENTRY')

        cnt = 0
        self.subscriber = rospy.Subscriber('/outdoor', PoseStamped, self.callback)

        while cnt < 15:
            if self.received_goal:
                rospy.loginfo("detected center: %s", self.received_goal)
                userdata.detect_entry_output = self.received_goal
                return 'succeeded'
            rospy.sleep(2)
            cnt += 1
        return 'aborted'


class DriveOutOfContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_out_of_container_input'],
                             output_keys=['drive_out_of_container_output'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state DRIVE_OUT_OF_CONTAINER')

        if userdata.drive_out_of_container_input:
            rospy.loginfo("got userdata: %s", userdata.drive_out_of_container_input)

            pose_stamped = userdata.drive_out_of_container_input
            pose_stamped = transform_pose(pose_stamped, 'map')

            move_base_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
            move_base_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position = pose_stamped.pose.position
            goal.target_pose.pose.orientation = pose_stamped.pose.orientation

            move_base_client.send_goal(goal)
            rospy.loginfo("now waiting...")
            move_base_client.wait_for_result()
            rospy.loginfo("after wait: %s", move_base_client.get_result())
            return 'succeeded'

        return 'aborted'


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
                     transitions={'succeeded': 'DRIVE_OUT_OF_CONTAINER', 'aborted': 'DETECT_ENTRY'},
                     remapping={'detect_entry_output': 'sm_input'})

            self.add('DRIVE_OUT_OF_CONTAINER', DriveOutOfContainer(),
                     transitions={'succeeded': 'undocked', 'aborted': 'failed'},
                     remapping={'drive_out_of_container_input': 'sm_input'})


def main():
    global TF_BUFFER
    rospy.init_node('undocking_smach')

    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)

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
