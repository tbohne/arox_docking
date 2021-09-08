#!/usr/bin/env python
import math

import actionlib
import rospy
import tf2_ros
from arox_docking.msg import DetectAction, DetectGoal, LocalizeAction
from arox_docking.msg import LocalizeResult
from arox_docking.util import transform_pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

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

        corners = res[:2]
        first_corner = Point()
        first_corner.x = corners[0].x
        first_corner.y = corners[0].y
        second_corner = Point()
        second_corner.x = corners[1].x
        second_corner.y = corners[1].y
        return first_corner, second_corner

    def compute_external_pose(self, first_corner, second_corner):
        """
        Compute an external pose relative to the container.

        :param first_corner: first corner of the container entry
        :param second_corner: second corner of the container entry
        :return: external pose
        """
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = (self.robot_pos.x + second_corner.x + first_corner.x) / 3
        pose.pose.position.y = (self.robot_pos.y + second_corner.y + first_corner.y) / 3
        angle = math.atan2(first_corner.y - second_corner.y, first_corner.x - second_corner.x)
        q = quaternion_from_euler(0, 0, angle)
        pose.pose.orientation = Quaternion(*q)
        return pose

    def execute(self, goal):
        """
        Executes the action server.

        :param goal: goal to be performed (dummy atm)
        """
        first_corner, second_corner = self.retrieve_corner_points()

        result = LocalizeResult()
        pose = self.compute_external_pose(first_corner, second_corner)
        result.station_pos = pose

        # visualize result
        point = Point()
        point.x = pose.pose.position.x
        point.y = pose.pose.position.y
        self.station_pub.publish(point)

        self.server.set_succeeded(result)

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
