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
DELTA_RADIUS = 0.01
# TODO: check sensor range
RADIUS_MIN = -20.0
RADIUS_MAX = 20.0
# TODO: distance to robot pos should be considered for the ACC_THRESH
ACC_THRESHOLD = 10


def get_theta_from_index(idx):
    return idx * DELTA_THETA + THETA_MIN


def get_radius_from_index(idx):
    return idx * DELTA_RADIUS + RADIUS_MIN


def calc_hough_y(theta, radius, x):
    return (radius - np.cos(theta * np.pi / 180.0) * x) / np.sin(theta * np.pi / 180.0)


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


def retrieve_best_line(hough_space):
    max_idx = hough_space.argmax()
    c, r = np.unravel_index(max_idx, hough_space.shape)
    assert hough_space.max() == hough_space[c][r]
    return c, r


def append_points(theta, c, lines):
    p1 = Point()
    p2 = Point()
    p1.x = -100
    p2.x = 100

    if theta == 0:
        p1.y = 0
        p2.y = 0
    else:
        p1.y = calc_hough_y(theta, get_radius_from_index(c), p1.x)
        p2.y = calc_hough_y(theta, get_radius_from_index(c), p2.x)
    lines.points.append(p1)
    lines.points.append(p2)


def generate_marker(hough_space, header, corresponding_points):
    lines = generate_default_marker(header)
    c, r = retrieve_best_line(hough_space)
    hough_space[c][r] = 0
    theta_best = get_theta_from_index(r)
    append_points(theta_best, c, lines)
    # point_lists.append(np.array(corresponding_points[(c, r)]))
    line_cnt = 1

    found_line_params = {get_radius_from_index(c): get_theta_from_index(r)}

    # container shape -> 3 lines (U-shape)
    while line_cnt < 3:

        # no peaks in hough space -> no lines
        if hough_space.max() <= ACC_THRESHOLD:
            break

        c, r = retrieve_best_line(hough_space)
        theta = get_theta_from_index(r)
        diff = abs(theta - theta_best)

        # should be either ~90° or ~270°
        if 85 < diff < 95 or 265 < diff < 275:

            # lst = np.array(corresponding_points[(c, r)])
            # x_vals = np.array([p.x for p in lst])
            # y_vals = np.array([p.y for p in lst])
            # x_mean = np.mean(x_vals)
            # y_mean = np.mean(y_vals)

            allowed = True
            for radius in found_line_params:
                rad = get_radius_from_index(c)
                # equal radius -> angle has to be different
                if -1 < abs(abs(radius) - abs(rad)) < 1:
                    # also equal angle -> forbidden
                    if -5 < abs(theta - found_line_params[radius]) < 5 or 175 < abs(
                            theta - found_line_params[radius]) < 185:
                        allowed = False
                        break

            if allowed:
                # TODO: REAL POINTS #############################################
                # for i in range(len(lst)):
                #     # only take the points near the mean
                #     #if abs(lst[i].x - x_mean) < 0.5 and abs(lst[i].y - y_mean) < 0.5:
                #     lines.points.append(lst[i])
                #################################################################
                append_points(theta, c, lines)
                found_line_params[get_radius_from_index(c)] = theta
                line_cnt += 1
        hough_space[c][r] = 0

    if line_cnt == 3:
        rospy.loginfo("parameters of detected lines: %s", found_line_params)

        radii = [r for r in found_line_params]
        thetas = [found_line_params[r] for r in radii]

        intersection12 = intersection((radii[0], thetas[0]), (radii[1], thetas[1]))
        intersection13 = intersection((radii[0], thetas[0]), (radii[2], thetas[2]))
        intersection23 = intersection((radii[1], thetas[1]), (radii[2], thetas[2]))

        p1 = Point(); p2 = Point(); p3 = Point()
        intersections = []
        if intersection12:
            p1.x, p1.y = intersection12
            intersections.append(p1)
        if intersection13:
            p2.x, p2.y = intersection13
            intersections.append(p2)
        if intersection23:
            p3.x, p3.y = intersection23
            intersections.append(p3)

        rospy.loginfo("INTER12: %s", intersection12)
        rospy.loginfo("INTER13: %s", intersection13)
        rospy.loginfo("INTER23: %s", intersection23)

        publish_intersections(intersections, header)

        return lines

    rospy.loginfo("NOTHING DETECTED!")
    return None


def publish_intersections(intersections, header):
    marker = Marker()
    marker.header = header
    marker.id = 1

    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1

    for point in intersections:
        marker.points.append(point)

    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    marker.color.a = 1.0
    marker.color.b = 1.0

    hough_line_pub.publish(marker)


def intersection(line1, line2):
    rho1, theta1 = line1
    rho2, theta2 = line2

    # parallel lines don't intersect
    if -5 < abs(theta1 - theta2) < 5 or 175 < abs(theta1 - theta2) < 185:
        return

    # to radians
    theta1 = theta1 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    A = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    return x0, y0


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
    if lines is not None:
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
