#!/usr/bin/env python
import math

import actionlib
import numpy as np
import rospy
import smach
import smach_ros
import tf2_ros
from arox_docking.config import CONTAINER_WIDTH, CONTAINER_LENGTH, EPSILON, MBF_FAILURE, DETECTION_ATTEMPTS
from arox_docking.msg import DockAction, DetectAction, DetectGoal
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

        self.robot_pose = None
        # subscribe to robot pose (ground truth)
        self.pose_sub = rospy.Subscriber("/odometry/filtered_map", Odometry, self.odom_callback, queue_size=1)
        self.outdoor_pub = rospy.Publisher("/publish_outdoor", Point, queue_size=1)

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        self.robot_pose = pose_stamped.pose

    def execute(self, userdata):
        """
        Executes the 'DETECT ENTRY' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DETECT_ENTRY')

        for _ in range(DETECTION_ATTEMPTS):
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
    def front_or_back_detected(first_corner, second_corner):
        return CONTAINER_WIDTH + CONTAINER_WIDTH * EPSILON >= dist(first_corner,
                                                                   second_corner) >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON

    def compute_direction_vector(self, first_corner, second_corner):
        if dist(self.robot_pose.position, first_corner) < dist(self.robot_pose.position, second_corner):
            return second_corner.x - first_corner.x, second_corner.y - first_corner.y
        else:
            return first_corner.x - second_corner.x, first_corner.y - second_corner.y

    def determine_external_point(self, base_point, res_vec, corners, front_corner_indices):
        distance = CONTAINER_LENGTH * 1.5
        if len(corners) == 2:
            # compensate center point distance
            distance += CONTAINER_LENGTH / 2
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
                if i not in front_corner_indices:
                    check_corner = corners[i]
                    break
            if dist(outdoor_candidate_one, check_corner) > dist(outdoor_candidate_two, check_corner):
                return outdoor_candidate_one
            return outdoor_candidate_two

        else:
            if dist(self.robot_pose.position, outdoor_candidate_one) < dist(self.robot_pose.position, outdoor_candidate_two):
                return outdoor_candidate_one
            return outdoor_candidate_two

    def determine_point_in_front_of_container(self, corners):
        """
        Computes a point in front of the container.

        :param corners: detected corner points of the container
        :return: outdoor point
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
            # TODO
            rospy.loginfo("ONLY ENTRY DETECTED -- TO BE IMPLEMENTED")
            base_point = Point()
            base_point.x = (corners[0].x + corners[1].x) / 2
            base_point.y = (corners[0].y + corners[1].y) / 2
            direction_vector = self.compute_direction_vector(corners[0], corners[1])
            front_corner_indices = [0, 1]

        # # both (front and back) should be considered
        # assert detected_entries == 2
        length = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        res_vec = (direction_vector[0] / length, direction_vector[1] / length)
        return self.determine_external_point(base_point, res_vec, corners, front_corner_indices)

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


class DriveOutOfContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_out_of_container_input'],
                             output_keys=['drive_out_of_container_output'])

        self.outdoor_pub = rospy.Publisher("/publish_outdoor", Point, queue_size=1)
        self.clear_markers_pub = rospy.Publisher("/clear_markers", String, queue_size=1)

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

        # rospy.loginfo("mbf res: %s", move_base_client.get_result())

        if move_base_client.get_result().outcome == MBF_FAILURE:
            rospy.loginfo("navigation failed: %s", move_base_client.get_goal_status_text())
            return False
        return True

    def execute(self, userdata):
        rospy.loginfo('executing state DRIVE_OUT_OF_CONTAINER')

        if userdata.drive_out_of_container_input:
            #rospy.loginfo("got userdata: %s", userdata.drive_out_of_container_input)

            if self.drive_to(userdata.drive_out_of_container_input):
                userdata.drive_out_of_container_output = get_success_msg()
                self.clear_markers_pub.publish("clearing")
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
