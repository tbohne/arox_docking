#!/usr/bin/env python
import rospy
from arox_docking.msg import PointArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class Visualizer:

    def __init__(self):
        self.hough_line_pub = rospy.Publisher("/docking/hough_lines", Marker, queue_size=1)
        self.corner_pub = rospy.Publisher("/docking/corner_points", Marker, queue_size=1)
        self.center_pub = rospy.Publisher("/docking/center_point", Marker, queue_size=1)
        self.outdoor_pub = rospy.Publisher("/docking/outdoor_marker", Marker, queue_size=1)
        self.entry_pub = rospy.Publisher("/docking/entry_point", Marker, queue_size=1)
        self.dbg_pub = rospy.Publisher("/docking/dbg_points", Marker, queue_size=1)
        self.dbg_pub_base = rospy.Publisher("/docking/dbg_points_base", Marker, queue_size=1)
        self.charging_station_pub = rospy.Publisher("/docking/charging_station", Marker, queue_size=1)

        self.line_sub = rospy.Subscriber("/publish_lines", PointArray, self.publish_detected_lines, queue_size=1)
        self.corner_sub = rospy.Subscriber("/publish_corners", PointArray, self.publish_corners, queue_size=1)
        self.center_sub = rospy.Subscriber("/publish_center", Point, self.publish_center_marker, queue_size=1)
        self.outdoor_sub = rospy.Subscriber("/publish_outdoor", Point, self.publish_outdoor_marker, queue_size=1)
        self.entry_sub = rospy.Subscriber("/publish_entry", PoseStamped, self.publish_entry, queue_size=1)
        self.dbg_sub = rospy.Subscriber("/publish_dbg", PointArray, self.publish_dgb_points, queue_size=1)
        rospy.Subscriber("/publish_dbg_base", PointArray, self.publish_dbg_point_base, queue_size=1)
        self.charging_station_sub = rospy.Subscriber("/publish_charger", Point, self.publish_charger, queue_size=1)
        self.clear_sub = rospy.Subscriber("/clear_markers", String, self.clear_markers, queue_size=1)

        self.line_marker = None
        self.dbg_marker = None
        self.dbg_marker_base = None
        self.corner_marker = None
        self.center_marker = None
        self.outdoor_marker = None
        self.entry_marker = None
        self.charging_maker = None

        self.init_line_marker()
        self.init_dbg_marker()
        self.init_corner_marker()
        self.init_center_marker()
        self.init_outdoor_marker()
        self.init_entry_marker()
        self.init_charging_marker()

    def init_line_marker(self):
        """
        Initializes the line marker.
        """
        self.line_marker = Marker()
        self.line_marker.header.frame_id = "base_link"
        self.line_marker.ns = "hough"
        self.line_marker.id = 0
        self.line_marker.action = Marker.ADD
        self.line_marker.type = Marker.LINE_LIST
        self.line_marker.pose.orientation.w = 1.0
        self.line_marker.scale.x = 0.01
        self.line_marker.color.g = 1.0
        self.line_marker.color.a = 1.0

    def publish_detected_lines(self, points):
        """
        Updates and publishes a line marker based on the specified points.

        :param points: points to update line marker with
        """
        if len(points.points) == 0:
            self.line_marker.points = []
        else:
            self.line_marker.points.extend(points.points)
        self.hough_line_pub.publish(self.line_marker)

    def init_dbg_marker(self):
        """
        Initializes the debug points marker.
        """
        self.dbg_marker = Marker()
        self.dbg_marker_base = Marker()
        self.dbg_marker.header.frame_id = self.dbg_marker_base.header.frame_id = "base_link"
        self.dbg_marker.id = 1
        self.dbg_marker_base.id = 2
        self.dbg_marker.type = self.dbg_marker_base.type = Marker.POINTS
        self.dbg_marker.action = self.dbg_marker_base.action = Marker.ADD
        self.dbg_marker.pose.orientation.w = self.dbg_marker_base.pose.orientation.w = 1
        self.dbg_marker.scale.x = self.dbg_marker.scale.y = self.dbg_marker.scale.z = 0.08
        self.dbg_marker_base.scale.x = self.dbg_marker_base.scale.y = self.dbg_marker_base.scale.z = 0.1
        self.dbg_marker.color.a = self.dbg_marker.color.b = self.dbg_marker.color.g = 1.0
        self.dbg_marker_base.color.a = self.dbg_marker_base.color.r = self.dbg_marker_base.color.b = 1.0

    def publish_dgb_points(self, points):
        """
        Updates and publishes debugging markers to indicate line detection candidates.

        :param points: currently considered points in line detection
        """
        if len(points.points) == 0:
            self.dbg_marker.points = []
        else:
            self.dbg_marker.points.extend(points.points)
        self.dbg_pub.publish(self.dbg_marker)

    def publish_dbg_point_base(self, points):
        if len(points.points) == 0:
            self.dbg_marker_base.points = []
        else:
            self.dbg_marker_base.points.extend(points.points)
        self.dbg_pub_base.publish(self.dbg_marker_base)

    def init_corner_marker(self):
        """
        Initializes the corner marker.
        """
        self.corner_marker = Marker()
        self.corner_marker.header.frame_id = "base_link"
        self.corner_marker.id = 1
        self.corner_marker.type = Marker.POINTS
        self.corner_marker.action = Marker.ADD
        self.corner_marker.pose.orientation.w = 1
        self.corner_marker.scale.x = self.corner_marker.scale.y = self.corner_marker.scale.z = 0.4
        self.corner_marker.color.a = self.corner_marker.color.b = 1.0

    def publish_corners(self, intersections):
        """
        Generates and publishes markers for the detected container corners.

        :param intersections: container corners (where the sides intersect)
        """
        self.corner_marker.points = intersections.points
        self.corner_pub.publish(self.corner_marker)

    def init_center_marker(self):
        """
        Initializes the center marker.
        """
        self.center_marker = Marker()
        self.center_marker.header.frame_id = "base_link"
        self.center_marker.id = 1
        self.center_marker.type = Marker.POINTS
        self.center_marker.action = Marker.ADD
        self.center_marker.pose.orientation.w = 1
        self.center_marker.scale.x = self.center_marker.scale.y = self.center_marker.scale.z = 0.4
        self.center_marker.color.a = self.center_marker.color.g = self.center_marker.color.r = 1.0

    def publish_center_marker(self, center):
        """
        Generates and publishes a marker for the center point of the container.

        :param center: container center
        """
        # empty point is used as indicator for deletion
        if center.x == 0 and center.y == 0 and center.z == 0:
            self.center_marker.action = Marker.DELETE
        else:
            self.center_marker.action = Marker.ADD
            self.center_marker.points = [center]
        self.center_pub.publish(self.center_marker)

    def init_charging_marker(self):
        """
        Initializes the marker for the charging station.
        """
        self.charging_maker = Marker()
        self.charging_maker.header.frame_id = "base_link"
        self.charging_maker.id = 1
        self.charging_maker.type = Marker.POINTS
        self.charging_maker.action = Marker.ADD
        self.charging_maker.pose.orientation.w = 1
        self.charging_maker.scale.x = self.charging_maker.scale.y = self.charging_maker.scale.z = 0.4
        self.charging_maker.color.a = 1.0
        self.charging_maker.color.r = 0.33
        self.charging_maker.color.g = self.charging_maker.color.b = 1.0

    def publish_charger(self, point):
        """
        Generates and publishes a marker for the charging station inside the container.

        :param point: coordinates of the charging station
        """
        # empty point is used as indicator for deletion
        if point.x == 0 and point.y == 0 and point.z == 0:
            self.charging_maker.action = Marker.DELETE
        else:
            self.charging_maker.action = Marker.ADD
            self.charging_maker.points = [point]
        self.charging_station_pub.publish(self.charging_maker)

    def init_outdoor_marker(self):
        """
        Initializes the outdoor marker.
        """
        self.outdoor_marker = Marker()
        self.outdoor_marker.header.frame_id = "base_link"
        self.outdoor_marker.id = 1
        self.outdoor_marker.type = Marker.POINTS
        self.outdoor_marker.action = Marker.ADD
        self.outdoor_marker.pose.orientation.w = 1
        self.outdoor_marker.scale.x = self.outdoor_marker.scale.y = self.outdoor_marker.scale.z = 0.4
        self.outdoor_marker.color.a = 1.0
        self.outdoor_marker.color.r = 0.6
        self.outdoor_marker.color.g = 0.0
        self.outdoor_marker.color.b = 0.9

    def publish_outdoor_marker(self, outdoor):
        """
        Generates and publishes a marker for a point in front of the container entry.

        :param outdoor: point in front of the container
        """
        # empty point is used as indicator for deletion
        if outdoor.x == 0 and outdoor.y == 0 and outdoor.z == 0:
            self.outdoor_marker.action = Marker.DELETE
        else:
            self.outdoor_marker.action = Marker.ADD
            self.outdoor_marker.points = [outdoor]
        self.outdoor_pub.publish(self.outdoor_marker)

    def init_entry_marker(self):
        """
        Initializes the entry marker.
        """
        self.entry_marker = Marker()
        self.entry_marker.header.frame_id = "base_link"
        self.entry_marker.id = 1
        self.entry_marker.type = Marker.ARROW
        self.entry_marker.action = Marker.ADD
        self.entry_marker.scale.x = 0.75
        self.entry_marker.scale.y = 0.2
        self.entry_marker.scale.z = 0.0
        self.entry_marker.color.r = 0
        self.entry_marker.color.g = 0
        self.entry_marker.color.b = 1.0
        self.entry_marker.color.a = 1.0

    def publish_entry(self, pose):
        """
        Generates and publishes an arrow indicating the position and direction of the container entry.

        :param pose: position and orientation towards the container entry
        """
        # empty pose is used as indicator for deletion
        if pose.pose.position.x == 0 and pose.pose.position.y == 0 and pose.pose.position.z == 0:
            self.entry_marker.action = Marker.DELETE
        else:
            self.entry_marker.action = Marker.ADD
            self.entry_marker.pose.position = pose.pose.position
            self.entry_marker.pose.orientation = pose.pose.orientation
        self.entry_pub.publish(self.entry_marker)

    def clear_markers(self, str):
        """
        Resets the markers.

        :param str: optional message
        """
        lst = PointArray()
        lst.points = []
        self.publish_detected_lines(lst)
        self.publish_corners(lst)
        self.publish_center_marker(Point())
        self.publish_outdoor_marker(Point())
        self.publish_entry(PoseStamped())
        self.publish_dgb_points(lst)
        self.publish_charger(Point())


def node():
    """
    Node that provides visualization functionalities for certain features during the docking / undocking procedures.
    """
    rospy.init_node("visualizer")
    Visualizer()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
