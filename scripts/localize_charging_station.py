#!/usr/bin/env python
import math

import actionlib
import rospy
import tf2_ros
from arox_docking.msg import DetectAction, DetectGoal, LocalizeAction
from arox_docking.msg import LocalizeResult
from arox_docking.util import transform_pose, dist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from arox_docking import config
import numpy as np

TF_BUFFER = None


class LocalizationServer:
    """
    Simple action server that localizes the inductive charging station inside of the mobile container.
    """

    def __init__(self):
        self.server = actionlib.SimpleActionServer('localize_charging_station', LocalizeAction, self.execute, False)
        self.station_pub = rospy.Publisher("/publish_charger", Point, queue_size=1)
        self.pose_sub = rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)
        self.robot_pos = None
        self.pose_sub = None
        self.server.start()

    @staticmethod
    def retrieve_corner_points():
        """
        Retrieves the entry corners from the container detection action.

        :return: entry corners of the container
        """
        client = actionlib.SimpleActionClient('detect_container', DetectAction)
        client.wait_for_server()
        goal = DetectGoal()
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().corners

        if len(res) >= 2:
            # TODO: possible to detect all 4 corners here, then it should be handled differently
            #       -> low priority - not yet clear how to receive charging station pos
            corners = res[:2]
            first_corner = Point()
            first_corner.x = corners[0].x
            first_corner.y = corners[0].y
            second_corner = Point()
            second_corner.x = corners[1].x
            second_corner.y = corners[1].y
            return first_corner, second_corner

    def compute_relative_pose(self, first_corner, second_corner, center):
        """
        Computes a pose relative to the container and the robot's position.

        :param first_corner: first corner of the container entry
        :param second_corner: second corner of the container entry
        :return: relative pose
        """
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()

        # pose.pose.position.x = (self.robot_pos.x + second_corner.x + first_corner.x) / 3
        # pose.pose.position.y = (self.robot_pos.y + second_corner.y + first_corner.y) / 3

        ############### charger pose #################
        charger = PoseStamped()
        charger.header.frame_id = "rampB"
        charger.header.stamp = rospy.Time.now()
        charger.pose.position.x = config.CHARGING_STATION_POS_X
        charger.pose.position.y = config.CHARGING_STATION_POS_Y
        ##############################################

        ############### corner poses #################
        first = PoseStamped()
        first.header.frame_id = "base_link"
        first.header.stamp = rospy.Time.now()
        first.pose.position.x = first_corner.x
        first.pose.position.y = first_corner.y

        sec = PoseStamped()
        sec.header.frame_id = "base_link"
        sec.header.stamp = rospy.Time.now()
        sec.pose.position.x = second_corner.x
        sec.pose.position.y = second_corner.y
        ##############################################

        pose_first = transform_pose(TF_BUFFER, first, 'map')
        pose_sec = transform_pose(TF_BUFFER, sec, 'map')
        charger = transform_pose(TF_BUFFER, charger, 'map')

        d1 = dist(charger.pose.position, pose_first.pose.position)
        d2 = dist(charger.pose.position, pose_sec.pose.position)
        if d1 < d2:
            pose.pose.position = first.pose.position
            direction_vector = (second_corner.x - first_corner.x, second_corner.y - first_corner.y)
            A = sec.pose.position
        else:
            pose.pose.position = sec.pose.position
            direction_vector = (first_corner.x - second_corner.x, first_corner.y - second_corner.y)
            A = first.pose.position

        angle = math.atan2(direction_vector[1], direction_vector[0])

        B = pose.pose.position
        P = Point()
        P.x = (A.x + B.x) / 2
        P.y = (A.y + B.y) / 2

        BA = (A.x - B.x, A.y - B.y)
        PC = (center.x - P.x, center.y - P.y)

        BA_length = np.sqrt(BA[0] ** 2 + BA[1] ** 2)
        BA_res = (BA[0] / BA_length, BA[1] / BA_length)

        PC_length = np.sqrt(PC[0] ** 2 + PC[1] ** 2)
        PC_res = (PC[0] / PC_length, PC[1] / PC_length)

        G1 = Point()
        G1.x = B.x + BA_res[0] * config.CHARGING_STATION_POS_X * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)
        G1.y = B.y + PC_res[1] * config.CHARGING_STATION_POS_Y * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)

        G2 = Point()
        G2.x = B.x - BA_res[0] * config.CHARGING_STATION_POS_X * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)
        G2.y = B.y - PC_res[1] * config.CHARGING_STATION_POS_Y * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)

        G3 = Point()
        G3.x = B.x + BA_res[0] * config.CHARGING_STATION_POS_X * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)
        G3.y = B.y - PC_res[1] * config.CHARGING_STATION_POS_Y * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)

        G4 = Point()
        G4.x = B.x - BA_res[0] * config.CHARGING_STATION_POS_X * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)
        G4.y = B.y + PC_res[1] * config.CHARGING_STATION_POS_Y * (config.CONTAINER_WIDTH + config.CONTAINER_LENGTH)

        d1 = dist(G1, center)
        d2 = dist(G2, center)
        d3 = dist(G3, center)
        d4 = dist(G4, center)
        if d1 <= d2 and d1 <= d3 and d1 <= d4:
            pose.pose.position = G1
        elif d2 <= d1 and d2 <= d3 and d2 <= d4:
            pose.pose.position = G2
        elif d3 <= d1 and d3 <= d2 and d3 <= d4:
            pose.pose.position = G3
        else:
            pose.pose.position = G4

        q = quaternion_from_euler(0, 0, angle)
        pose.pose.orientation = Quaternion(*q)
        return pose

    def execute(self, action_input):
        """
        Executes the action server.

        :param action_input: input to be used (entry corners)
        """
        if action_input:
            first_corner, second_corner, center = action_input.corners
            result = LocalizeResult()
            pose = self.compute_relative_pose(first_corner, second_corner, center)
            result.station_pos = pose

            # visualize result
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            self.station_pub.publish(point)
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted()

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        self.robot_pos = pose_stamped.pose.position


def node():
    """
    Node to detect the charging station inside of the mobile container.
    """
    global TF_BUFFER
    rospy.init_node("localize_charging_station")
    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)
    server = LocalizationServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
