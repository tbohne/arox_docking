#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

hough_line_pub = None
DELTA_THETA = 2
THETA_MIN = 0
THETA_MAX = 360
# should be sufficiently precise to identify reasonable lines
DELTA_RADIUS = 0.05
# TODO: check sensor range
RADIUS_MIN = -20.0
RADIUS_MAX = 20.0
# TODO: distance to robot pos should be considered for the ACC_THRESH
ACC_THRESHOLD = 80


def get_theta_from_index(index):
    return index * DELTA_THETA + THETA_MIN


def get_radius_from_index(index):
    return index * DELTA_RADIUS + RADIUS_MIN


def generate_default_marker(header):
    lines = Marker()
    lines.header = header
    lines.ns = "hough"
    lines.id = 0
    lines.action = lines.ADD
    lines.type = lines.LINE_LIST
    lines.pose.orientation.w = 1.0
    lines.scale.x = 0.01
    lines.color.g = 1.0
    lines.color.a = 1.0
    return lines


def generate_marker(hough_space, header, corresponding_points):
    lines = generate_default_marker(header)
    cnt = 0

    for r_idx in range(len(hough_space)):
        for theta_idx in range(len(hough_space[r_idx])):

            # LINE CONDITION
            if hough_space[r_idx][theta_idx] > ACC_THRESHOLD:
                cnt += 1

                # ORTHOGONALITY CONDITION # TODO: improve performance
                orthogonal = False
                for r_idx_2 in range(len(hough_space)):
                    if orthogonal:
                        break
                    if r_idx_2 == r_idx:
                        continue
                    for theta_idx_2 in range(len(hough_space[r_idx_2])):
                        # also a line
                        if hough_space[r_idx_2][theta_idx_2] > ACC_THRESHOLD:
                            theta = get_theta_from_index(theta_idx)
                            theta2 = get_theta_from_index(theta_idx_2)
                            diff = abs(theta - theta2) - 90
                            if -1 < diff < 1:
                                orthogonal = True
                                break

                if not orthogonal:
                    continue

                # list of points on line
                lst = np.array(corresponding_points[(r_idx, theta_idx)])
                x_vals = np.array([p.x for p in lst])
                y_vals = np.array([p.y for p in lst])
                x_mean = np.mean(x_vals)
                y_mean = np.mean(y_vals)

                for i in range(len(lst)):
                    # only take the points near the mean
                    if abs(lst[i].x - x_mean) < 0.3 and abs(lst[i].y - y_mean) < 0.3:
                        lines.points.append(corresponding_points[(r_idx, theta_idx)][i])

    rospy.loginfo("line cnt: %s", cnt)
    return lines


def cloud_callback(cloud):

    rospy.loginfo("receiving cloud..")

    global hough_line_pub

    # convert point cloud to a generator of the individual points (2D)
    point_generator = point_cloud2.read_points(cloud, field_names=("x", "y"))
    theta_values = np.deg2rad(np.arange(THETA_MIN, THETA_MAX, DELTA_THETA, dtype=np.double))
    r_values = np.arange(RADIUS_MIN, RADIUS_MAX, DELTA_RADIUS, dtype=np.double)

    # precompute sines and cosines
    cosines = np.cos(theta_values)
    sines = np.sin(theta_values)

    # define discrete hough space
    hough_space = np.zeros([len(r_values), len(theta_values)], dtype=int)

    # init dictionary for corresponding points
    corresponding_points = {}
    for r in range(len(r_values)):
        for theta in range(len(theta_values)):
            corresponding_points[(r, theta)] = []

    # compute hough space
    for p_idx, (x, y) in enumerate(point_generator):
        if not np.isnan(x) and not np.isnan(y):
            for theta_idx, (cos_theta, sin_theta) in enumerate(zip(cosines, sines)):
                # distance to origin
                r = x * cos_theta + y * sin_theta
                if r < RADIUS_MIN or r > RADIUS_MAX:
                    # rospy.logerr("Radius out of range!")
                    continue

                r_idx = int((r - RADIUS_MIN) / DELTA_RADIUS)
                hough_space[r_idx][theta_idx] += 1

                p = Point()
                p.x = x
                p.y = y
                corresponding_points[(r_idx, theta_idx)].append(p)

    lines = generate_marker(hough_space, cloud.header, corresponding_points)
    rospy.loginfo(cloud.header)
    hough_line_pub.publish(lines)


def node():
    global hough_line_pub

    rospy.init_node('hough_container')
    hough_line_pub = rospy.Publisher("/houghlines", Marker, queue_size=1)
    rospy.Subscriber("/velodyne_points", PointCloud2, cloud_callback, queue_size=1, buff_size=2 ** 32)

    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
