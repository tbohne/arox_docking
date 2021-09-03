#!/usr/bin/env python
import math

import actionlib
import rospy
from arox_docking.msg import DetectAction, DetectGoal, LocalizeAction
from arox_docking.msg import LocalizeResult
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class LocalizationServer:
    """
    Simple action server that localizes the inductive charging station inside of the mobile container.
    """

    def __init__(self):
        self.server = actionlib.SimpleActionServer('localize_charging_station', LocalizeAction, self.execute, False)
        self.station_pub = rospy.Publisher("/publish_charging_station", Point, queue_size=1)
        # subscribe to robot pose (ground truth)
        self.pose_sub = rospy.Subscriber("/pose_ground_truth", Odometry, self.odom_callback, queue_size=1)
        self.robot_pos = None
        self.pose_sub = None
        self.server.start()

    def execute(self, goal):
        """
        Executes the action server.

        :param goal: goal to be performed (dummy atm)
        """

        # some position relative to the container would be good (not absolute on map)

        # 1.) get entry corners (points A and B)
        client = actionlib.SimpleActionClient('detect_container', DetectAction)
        client.wait_for_server()
        goal = DetectGoal()
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result().corners

        corners = res[:2]
        A = Point()
        A.x = corners[0].x
        A.y = corners[0].y

        B = Point()
        B.x = corners[1].x
        B.y = corners[1].y

        goal = Point()
        goal.x = (self.robot_pos.x + B.x) / 2
        goal.y = (self.robot_pos.y + B.y) / 2

        angle = math.atan2(A.y - B.y, A.x - B.x)

        result = LocalizeResult()
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y
        q = quaternion_from_euler(0, 0, angle)
        pose.pose.orientation = Quaternion(*q)

        point = Point()
        point.x = pose.pose.position.x
        point.y = pose.pose.position.y
        self.station_pub.publish(point)

        result.station_pos = pose
        self.server.set_succeeded(result)

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update robot pos with
        """
        self.robot_pos = odom.twist.twist.linear


def node():
    """
    Node to detect the charging station inside of the mobile container.
    """
    rospy.init_node("localize_charging_station")
    server = LocalizationServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
