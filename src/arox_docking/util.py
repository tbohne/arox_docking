#!/usr/bin/env python
import math

import rospy
import tf2_geometry_msgs
import tf2_ros
from arox_docking.msg import DockAction


def get_failure_msg():
    """
    Generates a failure message.

    :return: failure message
    """
    msg = DockAction
    msg.result_state = "failure"
    return msg


def get_success_msg():
    """
    Generates a success message.

    :return: success message
    """
    msg = DockAction
    msg.result_state = "success"
    return msg


def dist(p1, p2):
    """
    Computes the Euclidean distance between the specified points.

    :param p1: point one
    :param p2: point two
    :return: Euclidean distance
    """
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def transform_pose(tf_buffer, pose_stamped, target_frame):
    try:
        transform = tf_buffer.lookup_transform(target_frame,
                                               pose_stamped.header.frame_id,  # source frame
                                               rospy.Time(0),  # get tf at first available time
                                               rospy.Duration(1.0))  # wait for one second

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("Exception while trying to transform pose stamped from %s to %s", pose_stamped.header.frame_id,
              target_frame)
        raise

    return pose_transformed
