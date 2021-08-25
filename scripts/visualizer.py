#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from arox_docking.msg import PointArray

HOUGH_LINE_PUB = None
CORNER_MARKER_PUB = None
DBG_PUB = None
LINE_SUB = None
CORNER_SUB = None
DBG_SUB = None
CLEAR_SUB = None


def generate_line_marker(points):
    """
    Initializes the marker for detected lines.

    :return: initialized line marker
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
    for p in list(points.points):
        lines.points.append(p)
    return lines


def publish_dgb_points(points):
    """
    Publishes the debugging points marker stored in DBG_POINTS.

    :param header: point cloud header
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
    Publishes the detected container corners as marker points.

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


def clear_markers(str):
    """
    Resets the outdated markers.

    :param header: point cloud header
    """
    lst = PointArray()
    lst.points = []
    publish_corners(lst)
    publish_dgb_points(lst)
    generate_line_marker(lst)


def node():
    """
    Node that provides visualization functionalities for certain features during the docking / undocking procedures.
    """
    global HOUGH_LINE_PUB, CORNER_MARKER_PUB, DBG_PUB, LINE_SUB, CORNER_SUB, DBG_SUB, CLEAR_SUB

    rospy.init_node("visualizer")

    HOUGH_LINE_PUB = rospy.Publisher("/hough_lines", Marker, queue_size=1)
    CORNER_MARKER_PUB = rospy.Publisher("/corner_points", Marker, queue_size=1)
    DBG_PUB = rospy.Publisher("/dbg_points", Marker, queue_size=1)

    LINE_SUB = rospy.Subscriber("/publish_lines", PointArray, generate_line_marker, queue_size=1)
    CORNER_SUB = rospy.Subscriber("/publish_corners", PointArray, publish_corners, queue_size=1)
    DBG_SUB = rospy.Subscriber("/publish_dbg", PointArray, publish_dgb_points, queue_size=1)
    CLEAR_SUB = rospy.Subscriber("/clear_markers", String, clear_markers, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
