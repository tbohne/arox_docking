#!/usr/bin/env python
import math

import actionlib
import numpy as np
import rospy
import smach
import smach_ros
import tf2_ros
from arox_docking.config import CONTAINER_WIDTH, CONTAINER_LENGTH, EPSILON, MBF_FAILURE, MBF_PAT_EXCEEDED, \
    DETECTION_ATTEMPTS, CHARGING_STATION_POS_X, CHARGING_STATION_POS_Y
from arox_docking.msg import DockAction, DetectAction, DetectGoal, LocalizeGoal, LocalizeAction
from arox_docking.util import dist, transform_pose, get_failure_msg, get_success_msg
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
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
        smach.State.__init__(self, outcomes=['succeeded_two_corners', 'succeeded_four_corners', 'aborted', 'failed'],
                             input_keys=['detect_container_input'],
                             output_keys=['detect_container_output', 'sm_output'])

        self.entry_pub = rospy.Publisher("/publish_entry", PoseStamped, queue_size=1)
        self.robot_pose = None
        rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)
        self.abortion_cnt = 0

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
        Executes the 'DETECT CONTAINER' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DETECT_CONTAINER')
        client = actionlib.SimpleActionClient('detect_container', DetectAction)
        client.wait_for_server()
        goal = DetectGoal()
        # robot is only expected to stand roughly in container proximity -> full scan required
        goal.scan_mode = "full"
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().corners
        if res:
            corners = res[:int(len(res) / 2)]
            # already detected whole container
            if len(corners) == 4:
                userdata.detect_container_output = res
                rospy.loginfo("already detected all four corners of the container..")
                rospy.loginfo("--> skip state 'ALIGN_ROBOT_TO_RAMP' and move directly into it..")
                self.abortion_cnt = 0
                return 'succeeded_four_corners'

            # only detected entry
            base_point, container_entry = self.determine_container_entry(res)
            if base_point is not None and container_entry is not None:
                angle = math.atan2(base_point.y - container_entry.y, base_point.x - container_entry.x)
                pose = PoseStamped()
                pose.pose.position = container_entry
                q = quaternion_from_euler(0, 0, angle)
                pose.pose.orientation = Quaternion(*q)
                self.entry_pub.publish(pose)
                userdata.detect_container_output = self.get_container_entry_with_orientation(container_entry, angle)
                self.abortion_cnt = 0
                return 'succeeded_two_corners'

        # TODO: implement slight pos change
        rospy.loginfo("DETECTION ATTEMPT: %s", self.abortion_cnt)
        rospy.sleep(2)
        self.abortion_cnt += 1
        if self.abortion_cnt == DETECTION_ATTEMPTS:
            userdata.sm_output = get_failure_msg()
            return 'failed'
        return 'aborted'

    def determine_container_entry(self, points):
        """
        Determines the container entry based on the detected corners and the average line points.

        :param points: detected container corners + avg points for each line
        :return: base point, container entry
        """
        # TODO: check whether that's correct
        corners = points[:int(len(points) / 2)]
        avg_points = points[int(len(points) / 2):]
        distance = 3.5
        first = sec = None

        if len(corners) == 2:
            corner_dist = dist(self.robot_pose.position, corners[0])
            avg_dists = [dist(self.robot_pose.position, p) for p in avg_points]
            for d in avg_dists:
                if corner_dist - d > CONTAINER_WIDTH / 2:
                    rospy.loginfo("##############################################")
                    rospy.loginfo("##############################################")
                    rospy.loginfo("detected wrong, i.e. closed, side of the container as entry..")
                    rospy.loginfo("##############################################")
                    rospy.loginfo("##############################################")
                    return None, None

            first, sec = corners
        elif len(corners) == 4:
            first = corners[0]
            curr_min = dist(self.robot_pose.position, first)
            # compute closest corner to robot pos
            for i in range(1, len(corners)):
                d = dist(self.robot_pose.position, corners[i])
                if d < curr_min:
                    first = corners[i]
                    curr_min = d
            # has to be a short side
            for i in range(0, len(corners)):
                if corners[i] != first and (CONTAINER_WIDTH + CONTAINER_WIDTH * EPSILON >= dist(first, corners[
                    i]) >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON):
                    sec = corners[i]
                    break

        base_point = Point()
        base_point.x = (first.x + sec.x) / 2
        base_point.y = (first.y + sec.y) / 2

        direction_vector = (sec.x - first.x, sec.y - first.y)
        length = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        res_vec = (direction_vector[0] / length, direction_vector[1] / length)
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
                             output_keys=['align_robot_to_ramp_output', 'sm_output'])

        self.entry_pub = rospy.Publisher("/publish_entry", PoseStamped, queue_size=1)

    def execute(self, userdata):
        """
        Executes the 'ALIGN ROBOT TO RAMP' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state ALIGN_ROBOT_TO_RAMP')

        if userdata.align_robot_to_ramp_input:
            #rospy.loginfo("got userdata: %s", userdata.align_robot_to_ramp_input)
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

            # TODO: add error treatment for mbf failures

            #rospy.loginfo("result: %s", move_base_client.get_result())
            # rospy.loginfo("sleeping for 5s...")
            # rospy.sleep(5)
            # clear arrow maker
            self.entry_pub.publish(PoseStamped())
            userdata.align_robot_to_ramp_output = None
            return 'succeeded'
        userdata.sm_output = get_failure_msg()
        return 'aborted'


class DriveIntoContainer(smach.State):
    """
    State that drives the robot into the container.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['drive_into_container_input'],
                             output_keys=['drive_into_container_output', 'sm_output'])

        self.robot_pose = None
        rospy.Subscriber("/odometry/gps", Odometry, self.odom_callback, queue_size=1)
        self.center_pub = rospy.Publisher("/publish_center", Point, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.init_pose = None

    def move_back_and_forth(self):
        rospy.loginfo("moving robot back and forth..")
        twist = Twist()
        twist.linear.x = -3.0
        rate = rospy.Rate(4)
        for _ in range(2):
            for _ in range(3):
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            twist.linear.x = 3.0

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        pose_stamped.header.frame_id = 'base_link'
        self.robot_pose = pose_stamped

    def compute_entry_from_four_corners(self, container_corners):
        """
        Computes the entry corners based on the four detected corners and the robot's position.

        :param container_corners: four detected corners of the container
        :return: entry corners
        """
        # find the closest corner
        first = container_corners[0]
        curr_min = dist(self.robot_pose.pose.position, first)
        for i in range(1, len(container_corners)):
            d = dist(container_corners[i], self.robot_pose.pose.position)
            if d < curr_min:
                first = container_corners[i]
                curr_min = d
        # find corner corresponding to 'first'
        sec = None
        for i in range(len(container_corners)):
            d = dist(first, container_corners[i])
            if container_corners[i] != first and (
                    CONTAINER_WIDTH + CONTAINER_WIDTH * EPSILON >= d >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON):
                sec = container_corners[i]
                break
        return first, sec

    @staticmethod
    def get_pose_from_point(point):
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.header.frame_id = "base_link"
        return pose

    def execute(self, userdata):
        """
        Executes the 'DRIVE INTO CONTAINER' state.

        :param userdata: input for the state
        :return: outcome of the execution (success / failure)
        """
        rospy.loginfo('executing state DRIVE_INTO_CONTAINER')
        self.init_pose = transform_pose(TF_BUFFER, self.robot_pose, "map")

        for _ in range(DETECTION_ATTEMPTS):
            # complete container already detected in previous state - no need to detect it again
            if userdata.drive_into_container_input:
                res = userdata.drive_into_container_input
            else:
                client = actionlib.SimpleActionClient('detect_container', DetectAction)
                client.wait_for_server()
                goal = DetectGoal()
                # robot is expected to stand right in front of the container in this state -> partial scan
                goal.scan_mode = "partial"
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

                self.center_pub.publish(center)
                angle = math.atan2(self.robot_pose.pose.position.y - center.y, self.robot_pose.pose.position.x - center.x)
                center_res = self.compute_center(center, angle)

                if center_res:
                    first, sec = self.compute_entry_from_four_corners(container_corners)
                    first_pose = self.get_pose_from_point(first)
                    sec_pose = self.get_pose_from_point(sec)
                    center_pose = self.get_pose_from_point(center)
                    # transform poses to 'map' before driving in with mbf (would impair precision)
                    first_pose = transform_pose(TF_BUFFER, first_pose, "map")
                    sec_pose = transform_pose(TF_BUFFER, sec_pose, "map")
                    center_pose = transform_pose(TF_BUFFER, center_pose, "map")

                    rospy.loginfo("DRIVING INTO CONTAINER..")

                    if self.drive_to(center_res):
                        # transform back to the new 'base_link'
                        first_pose = transform_pose(TF_BUFFER, first_pose, "base_link")
                        sec_pose = transform_pose(TF_BUFFER, sec_pose, "base_link")
                        center_pose = transform_pose(TF_BUFFER, center_pose, "base_link")
                        first = Point(first_pose.pose.position.x, first_pose.pose.position.y, 0)
                        sec = Point(sec_pose.pose.position.x, sec_pose.pose.position.y, 0)
                        center = Point(center_pose.pose.position.x, center_pose.pose.position.y, 0)
                        # clear marker
                        self.center_pub.publish(Point())
                        # pass the detected entry corners to the next state
                        userdata.drive_into_container_output = [first, sec, center]
                        return 'succeeded'

                    userdata.sm_output = get_failure_msg()
                    rospy.loginfo("not able to drive to center - realign in front of ramp before trying again..")
                    self.drive_to(self.init_pose)
                    return 'aborted'

            self.move_back_and_forth()

        rospy.loginfo("failed to detect container corners -> moving back and forth before trying again..")
        userdata.sm_output = get_failure_msg()
        return 'aborted'

    @staticmethod
    def drive_to(pose):
        """
        Drives the robot to the specified pose.

        :param pose: goal pose (at the center of the container)
        :return: true if successful, false otherwise
        """
        move_base_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()
        if pose.header.frame_id != "map":
            goal.target_pose = transform_pose(TF_BUFFER, goal.target_pose, 'map')

        move_base_client.send_goal(goal)
        rospy.loginfo("now waiting...")
        move_base_client.wait_for_result()

        out = move_base_client.get_result().outcome
        if out == MBF_FAILURE or out == MBF_PAT_EXCEEDED:
            rospy.loginfo("DRIVE_INTO_CONTAINER - navigation failed: %s", move_base_client.get_goal_status_text())
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


class LocalizeChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['localize_charging_station_input'],
                             output_keys=['localize_charging_station_output', 'sm_output'])

    @staticmethod
    def execute(userdata):
        rospy.loginfo('executing state LOCALIZE_CHARGING_STATION')

        client = actionlib.SimpleActionClient('localize_charging_station', LocalizeAction)
        client.wait_for_server()
        goal = LocalizeGoal()
        goal.corners = userdata.localize_charging_station_input
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().station_pos

        # empty res
        if res.pose.position.x == 0 and res.pose.position.y == 0 and res.pose.position.z == 0:
            userdata.localize_charging_station_output = get_failure_msg()
            userdata.sm_output = get_failure_msg()
            return 'aborted'

        rospy.loginfo("DETECTED CHARGING STATION..")
        pose = transform_pose(TF_BUFFER, res, 'map')
        userdata.localize_charging_station_output = pose
        return 'succeeded'


class AlignRobotToChargingStation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['align_robot_to_charging_station_input'],
                             output_keys=['align_robot_to_charging_station_output', 'sm_output'])

        rospy.Subscriber("/odometry/gps", Odometry, self.odom_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.robot_pose = None
        self.init_pose = None

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        self.robot_pose = pose_stamped

    def move_backwards_to_wall(self):
        rospy.loginfo("moving backwards to wall..")
        # move a bit backwards -> publish to /cmd_vel
        twist = Twist()
        twist.linear.x = -3.0
        rate = rospy.Rate(4)
        for _ in range(3):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def drive_to(self, pose):
        move_base_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        charging_station_pose = pose
        goal.target_pose.pose.position = charging_station_pose.pose.position
        goal.target_pose.pose.orientation = charging_station_pose.pose.orientation

        move_base_client.send_goal(goal)
        rospy.loginfo("now waiting...")
        move_base_client.wait_for_result()
        return move_base_client.get_result().outcome

    def execute(self, userdata):
        rospy.loginfo('executing state ALIGN_ROBOT_TO_CHARGING_STATION')
        self.init_pose = transform_pose(TF_BUFFER, self.robot_pose, "map")

        out = self.drive_to(userdata.align_robot_to_charging_station_input)
        if out == MBF_FAILURE or out == MBF_PAT_EXCEEDED:
            rospy.loginfo("ALIGN_TO_CHARGING_STATION failed - driving back to center and retry")
            userdata.sm_output = "ALIGN_TO_CHARGING_STATION failed"
            self.drive_to(self.init_pose)
            return 'aborted'

        self.move_backwards_to_wall()
        return 'succeeded'


class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             input_keys=['dock_input'],
                             output_keys=['dock_output'])

        self.clear_markers_pub = rospy.Publisher("/clear_markers", String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('executing state DOCK')
        self.clear_markers_pub.publish("clearing")
        userdata.dock_output = get_success_msg()
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
                     transitions={'succeeded_two_corners': 'ALIGN_ROBOT_TO_RAMP',
                                  'succeeded_four_corners': 'DRIVE_INTO_CONTAINER',
                                  'aborted': 'DETECT_CONTAINER',
                                  'failed': 'failed'},
                     remapping={'detect_container_output': 'sm_input'})

            self.add('ALIGN_ROBOT_TO_RAMP', AlignRobotToRamp(),
                     transitions={'succeeded': 'DRIVE_INTO_CONTAINER',
                                  'aborted': 'failed'},
                     remapping={'align_robot_to_ramp_input': 'sm_input',
                                'align_robot_to_ramp_output': 'sm_input'})

            self.add('DRIVE_INTO_CONTAINER', DriveIntoContainer(),
                     transitions={'succeeded': 'LOCALIZE_CHARGING_STATION',
                                  'aborted': 'failed'},
                     remapping={'drive_into_container_input': 'sm_input',
                                'drive_into_container_output': 'sm_input'})

            self.add('LOCALIZE_CHARGING_STATION', LocalizeChargingStation(),
                     transitions={'succeeded': 'ALIGN_ROBOT_TO_CHARGING_STATION', 'aborted': 'failed'},
                     remapping={'localize_charging_station_input': 'sm_input',
                                'localize_charging_station_output': 'sm_input'})

            self.add('ALIGN_ROBOT_TO_CHARGING_STATION', AlignRobotToChargingStation(),
                     transitions={'succeeded': 'DOCK', 'aborted': 'ALIGN_ROBOT_TO_CHARGING_STATION'},
                     remapping={'align_robot_to_charging_station_input': 'sm_input'})

            self.add('DOCK', Dock(),
                     transitions={'succeeded': 'docked', 'aborted': 'failed'},
                     remapping={'dock_output': 'sm_output'})


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
