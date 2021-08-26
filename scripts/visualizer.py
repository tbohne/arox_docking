#!/usr/bin/env python
import rospy
from arox_docking.msg import PointArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

HOUGH_LINE_PUB = None
CORNER_MARKER_PUB = None
CENTER_MARKER_PUB = None
OUTDOOR_MARKER_PUB = None
ENTRY_MARKER_PUB = None
DBG_PUB = None
LINE_SUB = None
CORNER_SUB = None
CENTER_SUB = None
OUTDOOR_SUB = None
ENTRY_SUB = None
DBG_SUB = None
CLEAR_SUB = None


def publish_detected_lines(points):
    global HOUGH_LINE_PUB
    """
    Generates and publishes a line marker based on the specified points.

    :param points: points to generate line marker with
    """
    lines = Marker()
    lines.header.frame_id = "base_link"
    lines.ns = "hough"
    lines.id = 0
    lines.action = lines.ADD
    lines.type = lines.LINE_LIST
    lines.pose.orientation.w = 1.0
    lines.scale.x = 0.01
    lines.color.g = 1.0
    lines.color.a = 1.0
    if len(points.points) == 0:
        lines.points = []
    else:
        lines.points.extend(points.points)
    HOUGH_LINE_PUB.publish(lines)


def publish_dgb_points(points):
    """
    Generates and publishes debugging markers to indicate line detection candidates.

    :param points: currently considered points in line detection
    """
    global DBG_PUB
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    if len(points.points) == 0:
        marker.points = []
    else:
        marker.points.extend(points.points)
    marker.scale.x = marker.scale.y = marker.scale.z = 0.08
    marker.color.a = marker.color.b = marker.color.g = 1.0
    DBG_PUB.publish(marker)


def publish_corners(intersections):
    """
    Generates and publishes markers for the detected container corners.

    :param intersections: container corners (where the sides intersect)
    """
    global CORNER_MARKER_PUB
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    marker.points = intersections.points
    marker.scale.x = marker.scale.y = marker.scale.z = 0.4
    marker.color.a = marker.color.b = 1.0
    CORNER_MARKER_PUB.publish(marker)


def publish_center_marker(center):
    """
    Generates and publishes a marker for the center point of the container.

    :param center: container center
    """
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
    """
    Generates and publishes a marker for a point in front of the container entry.

    :param outdoor: point in front of the container
    """
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


def publish_container_entry_arrow(pose):
    """
    Generates and publishes an arrow indicating the position and direction of the container entry.

    :param pose: position and orientation towards the container entry
    """
    global ENTRY_MARKER_PUB
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    if pose:
        marker.pose.position = pose.pose.position
        marker.pose.orientation = pose.pose.orientation

        marker.scale.x = 0.75
        marker.scale.y = 0.2
        marker.scale.z = 0.0

        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.color.a = 1.0

    ENTRY_MARKER_PUB.publish(marker)


def clear_markers(str):
    """
    Resets the markers.

    :param str: optional message
    """
    lst = PointArray()
    lst.points = []
    publish_corners(lst)
    publish_dgb_points(lst)
    publish_detected_lines(lst)


def node():
    """
    Node that provides visualization functionalities for certain features during the docking / undocking procedures.
    """
    global HOUGH_LINE_PUB, CORNER_MARKER_PUB, DBG_PUB, LINE_SUB, CORNER_SUB, DBG_SUB, CLEAR_SUB, CENTER_MARKER_PUB, CENTER_SUB, OUTDOOR_MARKER_PUB, OUTDOOR_SUB, ENTRY_MARKER_PUB, ENTRY_SUB

    rospy.init_node("visualizer")

    HOUGH_LINE_PUB = rospy.Publisher("/hough_lines", Marker, queue_size=1)
    CORNER_MARKER_PUB = rospy.Publisher("/corner_points", Marker, queue_size=1)
    DBG_PUB = rospy.Publisher("/dbg_points", Marker, queue_size=1)
    CENTER_MARKER_PUB = rospy.Publisher("/center_point", Marker, queue_size=1)
    OUTDOOR_MARKER_PUB = rospy.Publisher("/outdoor_marker", Marker, queue_size=1)
    ENTRY_MARKER_PUB = rospy.Publisher("/entry_point", Marker, queue_size=1)

    LINE_SUB = rospy.Subscriber("/publish_lines", PointArray, publish_detected_lines, queue_size=1)
    CORNER_SUB = rospy.Subscriber("/publish_corners", PointArray, publish_corners, queue_size=1)
    CENTER_SUB = rospy.Subscriber("/publish_center", Point, publish_center_marker, queue_size=1)
    OUTDOOR_SUB = rospy.Subscriber("/publish_outdoor", Point, publish_outdoor_marker, queue_size=1)
    ENTRY_SUB = rospy.Subscriber("/publish_entry", PoseStamped, publish_container_entry_arrow, queue_size=1)
    DBG_SUB = rospy.Subscriber("/publish_dbg", PointArray, publish_dgb_points, queue_size=1)
    CLEAR_SUB = rospy.Subscriber("/clear_markers", String, clear_markers, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
