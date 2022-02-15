#!/usr/bin/env python
import math

import actionlib
import numpy as np
import rospy
import smach
import smach_ros
import tf2_ros
from arox_docking import config
from arox_docking.msg import UndockAction, DetectAction, DetectGoal
from arox_docking.util import transform_pose, dist, get_failure_msg, get_success_msg
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

TF_BUFFER = None


class InsideContainer(smach.State):
    """
    Initial state - the robot is assumed to be inside the mobile container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata: smach.user_data.Remapper) -> str:
        """
        Executes the 'INSIDE_CONTAINER' state.

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

        self.robot_pose = None
        rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)
        self.outdoor_pub = rospy.Publisher("/publish_outdoor", Point, queue_size=1)

    def odom_callback(self, odom: Odometry):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pose with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        self.robot_pose = pose_stamped.pose

    def execute(self, userdata: smach.user_data.Remapper) -> str:
        """
        Executes the 'DETECT_ENTRY' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DETECT_ENTRY')

        if userdata.detect_entry_input:
            rospy.loginfo("entry (position in front of container) provided as input - no need for detection..")
            pose_stamped = transform_pose(TF_BUFFER, userdata.detect_entry_input.ramp_alignment_pose, 'map')
            userdata.detect_entry_output = pose_stamped
            return 'succeeded'

        for _ in range(config.DETECTION_ATTEMPTS):
            client = actionlib.SimpleActionClient('detect_container', DetectAction)
            client.wait_for_server()
            goal = DetectGoal()
            goal.scan_mode = "full"
            client.send_goal(goal)
            client.wait_for_result()
            res = client.get_result().corners

            if res:
                # detected only entry: 2 corners + 3 avg line points
                if len(res) == 5:
                    rospy.loginfo("CONTAINER ENTRY DETECTED..")
                    container_corners = res[:2]
                # detected whole container: 4 corners + 4 avg points
                elif len(res) == 8:
                    rospy.loginfo("WHOLE CONTAINER DETECTED..")
                    container_corners = res[:4]
                else:
                    userdata.detect_entry_output = get_failure_msg()
                    return 'aborted'

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
    def front_or_back_detected(first_corner: Point, second_corner: Point) -> bool:
        """
        Determines whether the container front or back was detected (not a side).

        :param first_corner: first detected corner
        :param second_corner: second detected corner
        :return: whether the container front or back was detected
        """
        return config.CONTAINER_WIDTH + config.CONTAINER_WIDTH * config.EPSILON >= dist(first_corner, second_corner) \
               >= config.CONTAINER_WIDTH - config.CONTAINER_WIDTH * config.EPSILON

    def compute_direction_vector(self, first_corner: Point, second_corner: Point) -> (float, float):
        """
        Computes an appropriate direction vector for the two corners.

        :param first_corner: first detected corner
        :param second_corner: second detected corner
        :return: coordinates of direction vector
        """
        if dist(self.robot_pose.position, first_corner) < dist(self.robot_pose.position, second_corner):
            return second_corner.x - first_corner.x, second_corner.y - first_corner.y
        else:
            return first_corner.x - second_corner.x, first_corner.y - second_corner.y

    def determine_external_point(self, base_point: Point, res_vec: (float, float), corners: list, corner_indices: list) -> Point:
        """
        Computes a point outside of the container.

        :param base_point: point in the center of the baseline
        :param res_vec: normalized direction vector
        :param corners: entry corners
        :param corner_indices: indices of entry corners
        :return: point outside the container
        """
        distance = config.CONTAINER_LENGTH * config.EXTERNAL_POINT_DIST
        if len(corners) == 2:
            # compensate center point distance
            distance += config.CONTAINER_LENGTH / 2
        # consider both candidates and take the one that is farther away from the backside corners
        outdoor_candidate_one = Point()
        outdoor_candidate_one.x = base_point.x + res_vec[1] * distance
        outdoor_candidate_one.y = base_point.y - res_vec[0] * distance
        outdoor_candidate_two = Point()
        outdoor_candidate_two.x = base_point.x + (res_vec[1] * -1) * distance
        outdoor_candidate_two.y = base_point.y - (res_vec[0] * -1) * distance

        if len(corners) > 2:
            check_corner = None
            for i in range(len(corners)):
                if i not in corner_indices:
                    check_corner = corners[i]
                    break
            if dist(outdoor_candidate_one, check_corner) > dist(outdoor_candidate_two, check_corner):
                return outdoor_candidate_one
            return outdoor_candidate_two
        else:
            if dist(self.robot_pose.position, outdoor_candidate_one) \
                    < dist(self.robot_pose.position, outdoor_candidate_two):
                return outdoor_candidate_one
            return outdoor_candidate_two

    def determine_point_in_front_of_container(self, corners: list) -> Point:
        """
        Computes a point in front of the container.

        :param corners: detected corner points of the container
        :return: point in front of container
        """
        base_point = None
        direction_vector = Point()
        detected_entries = 0
        used_indices = []
        front_corner_indices = []

        # whole container detected
        if len(corners) == 4:
            for i in range(len(corners) - 1):
                if i in used_indices:
                    continue
                for j in range(i + 1, len(corners)):
                    if j in used_indices:
                        continue
                    # front or back of the container -> there are two options
                    if self.front_or_back_detected(corners[i], corners[j]):
                        detected_entries += 1
                        if base_point is None:
                            base_point = Point()
                            base_point.x = (corners[i].x + corners[j].x) / 2
                            base_point.y = (corners[i].y + corners[j].y) / 2
                            direction_vector = self.compute_direction_vector(corners[i], corners[j])
                            used_indices.extend([i, j])
                            front_corner_indices = [i, j]
                        else:
                            tmp = Point()
                            tmp.x = (corners[i].x + corners[j].x) / 2
                            tmp.y = (corners[i].y + corners[j].y) / 2
                            # the base_point (front / back) that's closer to the robot pos is the one to choose
                            if dist(tmp, self.robot_pose.position) < dist(base_point, self.robot_pose.position):
                                base_point = tmp
                                direction_vector = self.compute_direction_vector(corners[i], corners[j])
                                used_indices.extend([i, j])
                                front_corner_indices = [i, j]
                            break
        # only entry detected
        elif len(corners) == 2:
            rospy.loginfo("ONLY ENTRY DETECTED")
            base_point = Point()
            base_point.x = (corners[0].x + corners[1].x) / 2
            base_point.y = (corners[0].y + corners[1].y) / 2
            direction_vector = self.compute_direction_vector(corners[0], corners[1])
            front_corner_indices = [0, 1]

        length = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        res_vec = (direction_vector[0] / length, direction_vector[1] / length)
        return self.determine_external_point(base_point, res_vec, corners, front_corner_indices)

    @staticmethod
    def compute_outdoor(outdoor_point: Point, angle: float) -> PoseStamped:
        """
        Computes the outdoor pose (position + orientation).

        :param outdoor_point: position outside the container
        :param angle: target orientation
        :return: outdoor pose (position + orientation)
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


class DriveOutOfContainer(smach.State):
    """
    State to navigate the robot out of the container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_out_of_container_input'],
                             output_keys=['drive_out_of_container_output'])

        self.outdoor_pub = rospy.Publisher("/publish_outdoor", Point, queue_size=1)
        self.clear_markers_pub = rospy.Publisher("/clear_markers", String, queue_size=1)

    @staticmethod
    def drive_to(pose: PoseStamped) -> bool:
        """
        Drives the robot to the specified pose (position + orientation).

        :param pose: goal pose
        :return: true if successful, false otherwise
        """
        pose_stamped = transform_pose(TF_BUFFER, pose, 'map')
        move_base_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = pose_stamped.pose.position
        goal.target_pose.pose.orientation = pose_stamped.pose.orientation

        move_base_client.send_goal(goal)
        rospy.loginfo("mbf nav goal sent..")
        move_base_client.wait_for_result()

        if move_base_client.get_result().outcome in [config.MBF_FAILURE, config.MBF_PAT_EXCEEDED]:
            rospy.loginfo("navigation failed: %s", move_base_client.get_goal_status_text())
            return False
        return True

    def execute(self, userdata: smach.user_data.Remapper) -> str:
        """
        Executes the 'DRIVE_OUT_OF_CONTAINER' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DRIVE_OUT_OF_CONTAINER')

        if userdata.drive_out_of_container_input:
            if self.drive_to(userdata.drive_out_of_container_input):
                userdata.drive_out_of_container_output = get_success_msg()
                self.clear_markers_pub.publish("clearing")
                return 'succeeded'
            userdata.drive_out_of_container_output = get_failure_msg()
            return 'aborted'
        userdata.drive_out_of_container_output = get_failure_msg()
        return 'aborted'


class UndockingStateMachine(smach.StateMachine):
    """
    State machine for the procedure of undocking the robot from the inductive charging station
    inside the mobile container.
    """

    def __init__(self):
        super(UndockingStateMachine, self).__init__(
            outcomes=['failed', 'undocked'],
            input_keys=['sm_input'],
            output_keys=['sm_output']
        )

        with self:
            self.add('INSIDE_CONTAINER', InsideContainer(),
                     transitions={'succeeded': 'DETECT_ENTRY',
                                  'aborted': 'failed'})

            self.add('DETECT_ENTRY', DetectEntry(),
                     transitions={'succeeded': 'DRIVE_OUT_OF_CONTAINER',
                                  'aborted': 'DETECT_ENTRY'},
                     remapping={'detect_entry_input': 'sm_input',
                                'detect_entry_output': 'sm_input'})

            self.add('DRIVE_OUT_OF_CONTAINER', DriveOutOfContainer(),
                     transitions={'succeeded': 'undocked',
                                  'aborted': 'failed'},
                     remapping={'drive_out_of_container_input': 'sm_input',
                                'drive_out_of_container_output': 'sm_output'})


def main():
    global TF_BUFFER
    rospy.init_node('undocking_smach')

    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)
    sm = UndockingStateMachine()

    sis = smach_ros.IntrospectionServer('undocking', sm, '/UNDOCKING')
    sis.start()

    # construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        'undock_from_charging_station', UndockAction,
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
