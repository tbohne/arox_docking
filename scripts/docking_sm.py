#!/usr/bin/env python
import math

import actionlib
import numpy as np
import rospy
import smach
import smach_ros
import tf2_ros
from arox_docking.config import CONTAINER_WIDTH, CONTAINER_LENGTH, EPSILON, MBF_FAILURE, MBF_PAT_EXCEEDED
from arox_docking.msg import DockAction, DetectAction, DetectGoal, LocalizeGoal, LocalizeAction
from arox_docking.util import dist, transform_pose, get_failure_msg, get_success_msg
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

TF_BUFFER = None


class ContainerProximity(smach.State):
    """
    Initial state - the robot is assumed to be located near the container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    @staticmethod
    def execute(userdata):
        """
        Executes the 'CONTAINER PROXIMITY' state.

        :param userdata: input for the state
        :return: atm just success
        """
        rospy.loginfo('executing state CONTAINER_PROXIMITY')
        # TODO: add gps check - correct area?
        return 'succeeded'


class DetectContainer(smach.State):
    """
    Container detection state - tries to detect the container in the laser scan.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['detect_container_input'],
                             output_keys=['detect_container_output'])

        self.entry_pub = rospy.Publisher("/publish_entry", PoseStamped, queue_size=1)

    def execute(self, userdata):
        """
        Executes the 'DETECT CONTAINER' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DETECT_CONTAINER')
        client = actionlib.SimpleActionClient('detect_container', DetectAction)
        client.wait_for_server()
        goal = DetectGoal()
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().corners
        if res:
            base_point, container_entry = self.determine_container_entry(res)
            angle = math.atan2(base_point.y - container_entry.y, base_point.x - container_entry.x)
            pose = PoseStamped()
            pose.pose.position = container_entry
            q = quaternion_from_euler(0, 0, angle)
            pose.pose.orientation = Quaternion(*q)
            self.entry_pub.publish(pose)
            userdata.detect_container_output = self.get_container_entry_with_orientation(container_entry, angle)
            return 'succeeded'
        return 'aborted'

    @staticmethod
    def determine_container_entry(points):
        """
        Determines the container entry based on the detected corners and the average line points.

        :param points: detected container corners + avg points for each line
        :return: base point, container entry
        """
        # TODO: check whether that's correct
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

    @staticmethod
    def get_container_entry_with_orientation(container_entry, angle):
        """
        Generates the pose in front of the container entry where the robot should move to (as pose stamped).

        :param container_entry: position in front of the container entry
        :param angle: goal orientation for the robot
        """
        entry_pose = PoseStamped()
        entry_pose.header.frame_id = "base_link"
        entry_pose.header.stamp = rospy.Time.now()
        entry_pose.pose.position.x = container_entry.x
        entry_pose.pose.position.y = container_entry.y
        entry_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, angle)
        entry_pose.pose.orientation = Quaternion(*q)
        return entry_pose


class AlignRobotToRamp(smach.State):
    """
    Robot alignment state - aligns the robot to the ramp of the container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['align_robot_to_ramp_input'],
                             output_keys=['align_robot_to_ramp_output'])

        self.entry_pub = rospy.Publisher("/publish_entry", PoseStamped, queue_size=1)

    def execute(self, userdata):
        """
        Executes the 'ALIGN ROBOT TO RAMP' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state ALIGN_ROBOT_TO_RAMP')

        if userdata.align_robot_to_ramp_input:
            rospy.loginfo("got userdata: %s", userdata.align_robot_to_ramp_input)
            pose_stamped = userdata.align_robot_to_ramp_input
            pose_stamped = transform_pose(TF_BUFFER, pose_stamped, "map")
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
            # clear arrow maker
            self.entry_pub.publish(PoseStamped())
            return 'succeeded'
        return 'aborted'


class DriveIntoContainer(smach.State):
    """
    State that drives the robot into the container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_into_container_input'],
                             output_keys=['drive_into_container_output'])

        self.center_pub = rospy.Publisher("/publish_center", Point, queue_size=1)

    def execute(self, userdata):
        """
        Executes the 'DRIVE INTO CONTAINER' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DRIVE_INTO_CONTAINER')

        # TODO: remove hard coded value
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

            # 4 corners + 4 avg points
            if res and len(res) == 8:
                rospy.loginfo("CONTAINER DETECTED!")
                container_corners = res[:4]
                center = Point()
                center.x = np.average([p.x for p in container_corners])
                center.y = np.average([p.y for p in container_corners])
                outdoor = self.determine_point_in_front_of_container(container_corners)
                self.center_pub.publish(center)
                angle = math.atan2(outdoor.y - center.y, outdoor.x - center.x)
                center_res = self.compute_center(center, angle)

                if center_res:
                    if self.drive_in(center_res):
                        userdata.drive_into_container_output = get_success_msg()
                        # clear marker
                        self.center_pub.publish(Point())
                        return 'succeeded'
                    userdata.drive_into_container_output = get_failure_msg()
                    return 'aborted'
        userdata.drive_into_container_output = get_failure_msg()
        return 'aborted'

    @staticmethod
    def drive_in(center_pos):
        """
        Drives the robot to the specified center position inside the container.

        :param center_pos: goal position at the center of the container
        :return: true if successful, false otherwise
        """
        pose_stamped = transform_pose(TF_BUFFER, center_pos, 'map')
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

        out = move_base_client.get_result().outcome
        if out == MBF_FAILURE or out == MBF_PAT_EXCEEDED:
            rospy.loginfo("navigation failed: %s", move_base_client.get_goal_status_text())
            return False
        return True

    @staticmethod
    def compute_center(center_point, angle):
        """
        Computes the center pose (position + orientation).

        :param center_point: position
        :param angle: orientation
        :return: pose
        """
        center = PoseStamped()
        center.header.frame_id = "base_link"
        center.header.stamp = rospy.Time.now()
        center.pose.position.x = center_point.x
        center.pose.position.y = center_point.y
        center.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, angle)
        center.pose.orientation = Quaternion(*q)
        return center

    @staticmethod
    def determine_point_in_front_of_container(corners):
        """
        Computes a point in front of the container.

        :param corners: detected corner points of the container
        :return: outdoor point
        """
        base_point = Point()
        for i in range(len(corners)):
            for j in range(len(corners)):
                if i != j:
                    if CONTAINER_WIDTH + CONTAINER_WIDTH * EPSILON >= dist(corners[i], corners[
                        j]) >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON:
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


class LocalizeChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['localize_charging_station_input'],
                             output_keys=['localize_charging_station_output'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state LOCALIZE_CHARGING_STATION')

        client = actionlib.SimpleActionClient('localize_charging_station', LocalizeAction)
        client.wait_for_server()
        goal = LocalizeGoal()
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().station_pos

        if res:
            rospy.loginfo("DETECTED CHARGING STATION: %s", res)
            pose = transform_pose(TF_BUFFER, res, 'map')
            userdata.localize_charging_station_output = pose
            return 'succeeded'

        userdata.localize_charging_station_output = get_failure_msg()
        return 'aborted'


class AlignRobotToChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['align_robot_to_charging_station_input'],
                             output_keys=['align_robot_to_charging_station_output'])

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def move_backwards_to_wall(self):
        rospy.loginfo("moving backwards to wall..")
        # move a bit backwards -> publish to /cmd_vel
        twist = Twist()
        twist.linear.x = -3.0
        rate = rospy.Rate(4)
        for _ in range(3):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def execute(self, userdata):
        rospy.loginfo('executing state ALIGN_ROBOT_TO_CHARGING_STATION')

        move_base_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        charging_station_pose = userdata.align_robot_to_charging_station_input
        goal.target_pose.pose.position = charging_station_pose.pose.position
        goal.target_pose.pose.orientation = charging_station_pose.pose.orientation

        move_base_client.send_goal(goal)
        rospy.loginfo("now waiting...")
        move_base_client.wait_for_result()

        if move_base_client.get_result().outcome == MBF_FAILURE:
            rospy.loginfo("navigation failed: %s", move_base_client.get_goal_status_text())
            return 'aborted'

        self.move_backwards_to_wall()
        return 'succeeded'


class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.clear_markers_pub = rospy.Publisher("/clear_markers", String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('executing state DOCK')
        self.clear_markers_pub.publish("clearing")
        return 'succeeded'


class DockingStateMachine(smach.StateMachine):
    """
    State machine for the procedure of docking the robot to the inductive charging station
    inside the mobile container.
    """

    def __init__(self):
        super(DockingStateMachine, self).__init__(
            outcomes=['failed', 'docked'],
            input_keys=['sm_input'],
            output_keys=['sm_output']
        )

        with self:
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

            self.add('LOCALIZE_CHARGING_STATION', LocalizeChargingStation(),
                     transitions={'succeeded': 'ALIGN_ROBOT_TO_CHARGING_STATION', 'aborted': 'failed'},
                     remapping={'localize_charging_station_output': 'sm_input'})

            self.add('ALIGN_ROBOT_TO_CHARGING_STATION', AlignRobotToChargingStation(),
                     transitions={'succeeded': 'DOCK', 'aborted': 'failed'},
                     remapping={'align_robot_to_charging_station_input': 'sm_input'})

            self.add('DOCK', Dock(),
                     transitions={'succeeded': 'docked', 'aborted': 'failed'})


def main():
    global TF_BUFFER
    rospy.init_node('docking_smach')
    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)
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
