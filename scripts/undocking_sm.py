#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros
import math
import tf2_ros
from arox_docking.config import CONTAINER_WIDTH, CONTAINER_LENGTH, EPSILON
from arox_docking.msg import DockAction, DetectAction, DetectGoal
from arox_docking.util import transform_pose, dist, FAILURE, get_failure_msg, get_success_msg
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from tf.transformations import quaternion_from_euler

TF_BUFFER = None


class InsideContainer(smach.State):
    """
    Initial state - the robot is assumed to be inside the mobile container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        """
        Executes the 'INSIDE CONTAINER' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state INSIDE_CONTAINER')
        # TODO: implement check whether robot is actually inside the container
        return 'succeeded'


class DetectEntry(smach.State):
    """
    State to detect the entry of the container from within.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['detect_entry_input'],
                             output_keys=['detect_entry_output'])

        self.outdoor_pub = rospy.Publisher("/publish_outdoor", Point, queue_size=1)

    def execute(self, userdata):
        """
        Executes the 'DETECT ENTRY' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DETECT_ENTRY')

        attempts = 5
        cnt = 0
        while cnt < attempts:
            cnt += 1
            client = actionlib.SimpleActionClient('detect_container', DetectAction)
            client.wait_for_server()
            goal = DetectGoal()
            client.send_goal(goal)
            client.wait_for_result()
            res = client.get_result().corners

            # found something - either the whole container or only the entry
            if res:
                # detect only entry: 2 corners + 3 avg line points
                if len(res) == 5:
                    rospy.loginfo("CONTAINER ENTRY DETECTED..")
                    container_corners = res[:2]
                # detected whole container: 4 corners + 4 avg points
                elif len(res) == 8:
                    rospy.loginfo("WHOLE CONTAINER DETECTED..")
                    container_corners = res[:4]

                center = Point()
                center.x = np.average([p.x for p in container_corners])
                center.y = np.average([p.y for p in container_corners])

                outdoor = self.determine_point_in_front_of_container(container_corners)
                self.outdoor_pub.publish(outdoor)
                angle = math.atan2(center.y - outdoor.y, center.x - outdoor.x)
                outdoor_res = self.compute_outdoor(outdoor, angle)
                if outdoor_res:
                    userdata.detect_entry_output = outdoor_res
                    return 'succeeded'
        userdata.detect_entry_output = get_failure_msg()
        return 'aborted'

    @staticmethod
    def compute_outdoor(outdoor_point, angle):
        """
        Computes the outdoor pose (position + orientation).

        :param outdoor_point: position
        :param angle: orientation
        :return: pose
        """
        outdoor = PoseStamped()
        outdoor.header.frame_id = "base_link"
        outdoor.header.stamp = rospy.Time.now()
        outdoor.pose.position.x = outdoor_point.x
        outdoor.pose.position.y = outdoor_point.y
        outdoor.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, angle)
        outdoor.pose.orientation = Quaternion(*q)
        return outdoor

    @staticmethod
    def determine_point_in_front_of_container(corners):
        """
        Computes a point in front of the container.

        :param corners: detected corner points of the container
        :return: outdoor point
        """

        base_point = Point()
        direction_vector = Point()

        # whole container detected
        if len(corners) == 4:
            for i in range(len(corners)):
                for j in range(len(corners)):
                    if i != j:
                        if CONTAINER_WIDTH + CONTAINER_WIDTH * EPSILON >= dist(corners[i], corners[
                            j]) >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON:
                            base_point.x = (corners[i].x + corners[j].x) / 2
                            base_point.y = (corners[i].y + corners[j].y) / 2
                            direction_vector = (corners[j].x - corners[i].x, corners[j].y - corners[i].y)
                            break
        # only entry detected
        elif len(corners) == 2:
            base_point.x = (corners[0].x + corners[1].x) / 2
            base_point.y = (corners[0].y + corners[1].y) / 2
            direction_vector = (corners[1].x - corners[0].x, corners[1].y - corners[0].y)

        length = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        res_vec = (direction_vector[0] / length, direction_vector[1] / length)
        distance = CONTAINER_LENGTH * 1.5
        outdoor = Point()
        outdoor.x = base_point.x - res_vec[1] * distance
        outdoor.y = base_point.y + res_vec[0] * distance

        return outdoor


class DriveOutOfContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_out_of_container_input'],
                             output_keys=['drive_out_of_container_output'])

        self.outdoor_pub = rospy.Publisher("/publish_outdoor", Point, queue_size=1)

    def drive_to(self, pos):
        """
        Drives the robot to the specified position.

        :param pos: goal position
        :return: true if successful, false otherwise
        """
        pose_stamped = transform_pose(TF_BUFFER, pos, 'map')
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

        if move_base_client.get_result().outcome == FAILURE:
            rospy.loginfo("navigation failed: %s", move_base_client.get_goal_status_text())
            return False
        return True

    def execute(self, userdata):
        rospy.loginfo('executing state DRIVE_OUT_OF_CONTAINER')

        if userdata.drive_out_of_container_input:
            rospy.loginfo("got userdata: %s", userdata.drive_out_of_container_input)

            if self.drive_to(userdata.drive_out_of_container_input):
                userdata.drive_out_of_container_output = get_success_msg()
                # clear marker
                self.outdoor_pub.publish(Point())
                return 'succeeded'

            userdata.drive_out_of_container_output = get_failure_msg()
            return 'aborted'

        userdata.drive_out_of_container_output = get_failure_msg()
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
                     remapping={'drive_out_of_container_input': 'sm_input',
                                'drive_out_of_container_output': 'sm_output'})


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
