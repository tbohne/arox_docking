#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

import tf2_ros
import tf2_geometry_msgs

from visualization_msgs.msg import Marker
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from arox_docking.msg import DockAction, DetectAction, DetectGoal
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import math

def dist(p1, p2):
    """
    Computes the Euclidean distance between the specified points.

    :param p1: point one
    :param p2: point two
    :return: Euclidean distance
    """
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

CONTAINER_WIDTH = 2.83
CONTAINER_LENGTH = 3.7
EPSILON = 0.2

def determine_point_in_front_of_container(corners):
    base_point = Point()
    for i in range(len(corners)):
        for j in range(len(corners)):
            if i != j:
                if CONTAINER_WIDTH + CONTAINER_WIDTH * EPSILON >= dist(corners[i], corners[j]) >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON:
                    base_point.x = (corners[i].x + corners[j].x) / 2
                    base_point.y = (corners[i].y + corners[j].y) / 2
                    direction_vector = (corners[j].x - corners[i].x, corners[j].y - corners[i].y)
                    break

    length = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
    res_vec = (direction_vector[0] / length, direction_vector[1] / length)

    distance = CONTAINER_LENGTH * 1.5
    outdoor = Point()
    outdoor.x = base_point.x - res_vec[1] * distance
    outdoor.y = base_point.y + res_vec[0] * distance

    return outdoor

TF_BUFFER = None
FAILURE = 50
CENTER_DETECTION_ATTEMPTS = 30
CENTER_MARKER_PUB = None
OUTDOOR_MARKER_PUB = None
ENTRY_MARKER_PUB = None


def publish_center_marker(center):
    global CENTER_MARKER_PUB
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    if center:
        marker.points = [center]
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.a = marker.color.g = marker.color.r = 1.0
    CENTER_MARKER_PUB.publish(marker)


def publish_outdoor_marker(outdoor):
    global OUTDOOR_MARKER_PUB
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    if outdoor:
        marker.points = [outdoor]
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.6
        marker.color.g = 0.0
        marker.color.b = 0.9
    OUTDOOR_MARKER_PUB.publish(marker)


def publish_container_entry_arrow(container_entry, angle):
    """
    Publishes the position of the container entry as arrow marker pointing towards the entry.

    :param container_entry: point in front of the container entry
    :param angle: orientation in front of the container
    """
    global ENTRY_MARKER_PUB
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    if container_entry:
        marker.pose.position.x = container_entry.x
        marker.pose.position.y = container_entry.y

        q = quaternion_from_euler(0, 0, angle)
        marker.pose.orientation = Quaternion(*q)

        marker.scale.x = 0.75
        marker.scale.y = 0.2
        marker.scale.z = 0.0

        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.color.a = 1.0

    ENTRY_MARKER_PUB.publish(marker)


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


def get_failure_msg():
    msg = DockAction
    msg.result_state = "failure"
    return msg


def get_success_msg():
    msg = DockAction
    msg.result_state = "success"
    return msg


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

    def determine_container_entry(self, points):
        """
        Determines the container entry based on the detected corners and the average line points.

        :param points: detected container corners + avg points for each line
        :return: base point, container entry
        """
        corners = points[:int(len(points) / 2)]
        avg_points = points[int(len(points) / 2):]

        base_point = Point()
        base_point.x = (corners[0].x + corners[1].x) / 2
        base_point.y = (corners[0].y + corners[1].y) / 2

        direction_vector = (corners[1].x - corners[0].x, corners[1].y - corners[0].y)
        length = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        res_vec = (direction_vector[0] / length, direction_vector[1] / length)

        distance = 2.5
        entry_candidate_one = Point()
        entry_candidate_one.x = base_point.x - res_vec[1] * distance
        entry_candidate_one.y = base_point.y + res_vec[0] * distance
        entry_candidate_two = Point()
        entry_candidate_two.x = base_point.x + res_vec[1] * distance
        entry_candidate_two.y = base_point.y - res_vec[0] * distance

        # the one further away from the averages is the position to move to
        avg_entry_candidate_one = np.average([dist(entry_candidate_one, p) for p in avg_points])
        avg_entry_candidate_two = np.average([dist(entry_candidate_two, p) for p in avg_points])
        if avg_entry_candidate_one > avg_entry_candidate_two:
            return base_point, entry_candidate_one
        return base_point, entry_candidate_two

    def get_container_entry_with_orientation(self, container_entry, angle):
        """
        Publishes the position in front of the container entry where the robot should move to (as pose stamped).

        :param container_entry: position in front of the container entry
        :param angle: orientation in front of the container
        """
        global ENTRY_PUB
        goal = PoseStamped()
        goal.header.frame_id = "base_link"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = container_entry.x
        goal.pose.position.y = container_entry.y
        goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, angle)
        goal.pose.orientation = Quaternion(*q)
        return goal

    def execute(self, userdata):
        rospy.loginfo('executing state DETECT_CONTAINER')

        client = actionlib.SimpleActionClient('detect_container', DetectAction)
        client.wait_for_server()
        goal = DetectGoal()
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().corners
        rospy.loginfo("RESULT: %s", res)
        if res:
            rospy.loginfo("detected entry: %s", res)
            base_point, container_entry = self.determine_container_entry(res)
            angle = math.atan2(base_point.y - container_entry.y, base_point.x - container_entry.x)
            publish_container_entry_arrow(container_entry, angle)
            userdata.detect_container_output = self.get_container_entry_with_orientation(container_entry, angle)
            return 'succeeded'
        return 'aborted'


class AlignRobotToRamp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['align_robot_to_ramp_input'],
                             output_keys=['align_robot_to_ramp_output'])

    def execute(self, userdata):
        rospy.loginfo('executing state ALIGN_ROBOT_TO_RAMP')

        if userdata.align_robot_to_ramp_input:
            rospy.loginfo("got userdata: %s", userdata.align_robot_to_ramp_input)
            pose_stamped = userdata.align_robot_to_ramp_input
            pose_stamped = transform_pose(pose_stamped, "map")
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
            rospy.loginfo("sleeping for 5s...")
            rospy.sleep(5)
            return 'succeeded'
        return 'aborted'


class DriveIntoContainer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_into_container_input'],
                             output_keys=['drive_into_container_output'])

    def drive_in(self, center_pos):
        pose_stamped = transform_pose(center_pos, 'map')
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

    def compute_center(self, center_point, angle):
        center = PoseStamped()
        center.header.frame_id = "base_link"
        center.header.stamp = rospy.Time.now()
        center.pose.position.x = center_point.x
        center.pose.position.y = center_point.y
        center.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, angle)
        center.pose.orientation = Quaternion(*q)
        return center

    def execute(self, userdata):
        rospy.loginfo('executing state DRIVE_INTO_CONTAINER')

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
            rospy.loginfo("RESULT: %s", res)

            # 4 corners + 4 avg points
            if res and len(res) == 8:
                rospy.loginfo("CONTAINER DETECTED!")

                container_corners = res[:4]

                center = Point()
                center.x = np.average([p.x for p in container_corners])
                center.y = np.average([p.y for p in container_corners])

                # point in front of container
                outdoor = determine_point_in_front_of_container(container_corners)

                publish_center_marker(center)
                publish_outdoor_marker(outdoor)

                angle = math.atan2(outdoor.y - center.y, outdoor.x - center.x)
                center_res = self.compute_center(center, angle)

                if center_res:
                    rospy.loginfo("detected center: %s", center_res)
                    if self.drive_in(center_res):
                        userdata.drive_into_container_output = get_success_msg()
                        return 'succeeded'
                    userdata.drive_into_container_output = get_failure_msg()
                    return 'aborted'
        rospy.loginfo("NUMBER OF CORNERS: %s", len(res))
        userdata.drive_into_container_output = get_failure_msg()
        return 'aborted'


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
            output_keys=['sm_output']
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

            self.add('DRIVE_INTO_CONTAINER', DriveIntoContainer(),
                     transitions={'succeeded': 'LOCALIZE_CHARGING_STATION',
                                  'aborted': 'failed'},
                     remapping={'drive_into_container_output': 'sm_output'})

            self.add('LOCALIZE_CHARGING_STATION', LocalizeChargingStation(), transitions={
                'succeeded': 'ALIGN_ROBOT_TO_CHARGING_STATION',
                'aborted': 'failed'})

            self.add('ALIGN_ROBOT_TO_CHARGING_STATION', AlignRobotToChargingStation(), transitions={
                'succeeded': 'DOCK',
                'aborted': 'failed'})

            self.add('DOCK', Dock(),
                     transitions={'succeeded': 'docked', 'aborted': 'failed'})


def main():
    rospy.init_node('docking_smach')

    global TF_BUFFER, CENTER_MARKER_PUB, OUTDOOR_MARKER_PUB, ENTRY_MARKER_PUB

    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)

    CENTER_MARKER_PUB = rospy.Publisher("/center_point", Marker, queue_size=1)
    OUTDOOR_MARKER_PUB = rospy.Publisher("/outdoor_marker", Marker, queue_size=1)
    ENTRY_MARKER_PUB = rospy.Publisher("/entry_point", Marker, queue_size=1)

    sm = DockingStateMachine()

    # construct action server wrapper
    asw = smach_ros.ActionServerWrapper(
        'dock_to_charging_station', DockAction,
        wrapped_container=sm,
        succeeded_outcomes=['docked'],
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
