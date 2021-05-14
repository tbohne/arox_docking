#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

CONTAINER_WIDTH = 2.83
CONTAINER_LENGTH = 3.7
EPSILON = 0.2

HOUGH_LINE_PUB = None
CORNER_PUB = None
DBG_PUB = None

DELTA_THETA = 2
THETA_MIN = 0
THETA_MAX = 360
# should be sufficiently precise to identify reasonable lines
DELTA_RADIUS = 0.01
# TODO: check sensor range
RADIUS_MIN = -10.0
RADIUS_MAX = 10.0
# TODO: distance to robot pos should be considered for the ACC_THRESH
ACC_THRESHOLD = 10


def get_theta_from_index(idx):
    return idx * DELTA_THETA + THETA_MIN


def get_radius_from_index(idx):
    return idx * DELTA_RADIUS + RADIUS_MIN


def calc_hough_y(theta, radius, x):
    return (radius - np.cos(theta * np.pi / 180.0) * x) / np.sin(theta * np.pi / 180.0)


def generate_default_line_marker(header):
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


def append_line_points(theta, rad_idx, lines):
    p1 = Point()
    p2 = Point()
    p1.x = -100
    p2.x = 100

    if theta == 0:
        p1.y = 0
        p2.y = 0
    else:
        p1.y = calc_hough_y(theta, get_radius_from_index(rad_idx), p1.x)
        p2.y = calc_hough_y(theta, get_radius_from_index(rad_idx), p2.x)
    lines.points.append(p1)
    lines.points.append(p2)


def median_filter(points_on_line):
    # TODO: not really a median filter - just removing far away points
    median = Point()
    median.x = np.median(np.sort([p.x for p in points_on_line]))
    median.y = np.median(np.sort([p.y for p in points_on_line]))
    res = []
    for p in points_on_line:
        if dist(p, median) <= CONTAINER_LENGTH:
            res.append(p)
    return res


def compute_avg_and_max_distance(point_list):
    distances = [dist(i, j) for i in point_list for j in point_list if i != j]
    return np.average(distances), np.max(distances)


def reasonable_dist_to_already_detected_lines(point_list, avg_points):
    avg = Point()
    avg.x = np.average([p.x for p in point_list])
    avg.y = np.average([p.y for p in point_list])
    for p in avg_points:
        if dist(avg, p) > CONTAINER_LENGTH or dist(avg, p) < CONTAINER_WIDTH / 2:
            return False
    return True


def detected_reasonable_line(point_list, theta_best, theta, avg_points):
    # criteria to determine whether line could be container side
    diff = 90 if theta_best is None else abs(theta - theta_best)
    # should be either ~90° or ~270° if base is entry
    # (base is not necessarily the entry of the container -> ~0 and ~180 ok as well)
    orthogonal_to_base = diff < 2 or 88 < diff < 92 or 178 < diff < 182 or 268 < diff < 272

    # criteria to check whether line is probably something else
    avg_dist, max_dist = compute_avg_and_max_distance(point_list)
    reasonable_dist = reasonable_dist_to_already_detected_lines(point_list, avg_points)
    reasonable_len = CONTAINER_LENGTH + CONTAINER_LENGTH * EPSILON >= max_dist >= CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON
    reasonable_avg_distances = CONTAINER_WIDTH / 2 >= avg_dist >= 0.5

    return reasonable_len and reasonable_dist and reasonable_avg_distances and orthogonal_to_base


def compute_intersection_points(found_line_params):
    radii = [p[0] for p in found_line_params]
    thetas = [p[1] for p in found_line_params]

    intersections = []
    for i in range(len(radii)):
        for j in range(i + 1, len(radii)):
            inter = intersection((radii[i], thetas[i]), (radii[j], thetas[j]))
            if inter:
                p = Point()
                p.x, p.y = inter
                intersections.append(p)

    return intersections


def generate_marker(hough_space, header, corresponding_points):
    lines = generate_default_line_marker(header)
    theta_best = None
    line_cnt = 0
    found_line_params = []
    dbg_points = []
    avg_points = []

    # container shape -> 3 lines (U-shape)
    while line_cnt < 4:
        # no peaks in hough space -> no lines
        if hough_space.max() < ACC_THRESHOLD:
            break

        c, r = retrieve_best_line(hough_space)
        # actual points on the line
        point_list = np.array(corresponding_points[(c, r)])
        # remove 'outliers'
        point_list = median_filter(point_list)
        theta = get_theta_from_index(r)

        if len(point_list) > ACC_THRESHOLD and detected_reasonable_line(point_list, theta_best, theta, avg_points):
            allowed = True
            for radius, angle in found_line_params:
                rad = get_radius_from_index(c)
                # "equal" radius -> angle has to be different
                if abs(abs(radius) - abs(rad)) < DELTA_RADIUS * 10:
                    # also equal angle -> forbidden
                    if abs(theta - angle) < 60 or 120 < abs(theta - angle) < 240:
                        allowed = False
                        break

            if allowed:
                # check intersections
                tmp = found_line_params.copy()
                tmp.append((get_radius_from_index(c), theta))
                intersections = compute_intersection_points(tmp)
                for p in intersections:
                    for j in intersections:
                        if p != j:
                            d = dist(p, j)
                            width_eps = CONTAINER_WIDTH * EPSILON
                            length_eps = CONTAINER_LENGTH * EPSILON
                            tolerated_width = CONTAINER_WIDTH - width_eps < d < CONTAINER_WIDTH + width_eps
                            tolerated_length = CONTAINER_LENGTH - length_eps < d < CONTAINER_LENGTH + length_eps
                            if not tolerated_width and not tolerated_length:
                                allowed = False
                if allowed:
                    for i in point_list:
                        dbg_points.append(i)
                    publish_dgb_points(dbg_points, header)

                    append_line_points(theta, c, lines)
                    found_line_params.append((get_radius_from_index(c), theta))
                    line_cnt += 1

                    # save avg point for each line
                    avg = Point()
                    avg.x = np.average([p.x for p in point_list])
                    avg.y = np.average([p.y for p in point_list])
                    avg_points.append(avg)

                    if theta_best is None:
                        theta_best = theta

        hough_space[c][r] = 0

    if line_cnt >= 3:
        rospy.loginfo("parameters of detected lines: %s", found_line_params)

        intersections = compute_intersection_points(found_line_params)

        container_corners = []
        for p in intersections:
            for j in intersections:
                if p != j:
                    d = dist(p, j)
                    if container_side_detected(d):
                        rospy.loginfo("dist: %s", d)
                        container_corners.append(p)

        if len(container_corners) > 0:
            if len(container_corners) == 4:
                rospy.loginfo("CONTAINER DETECTED!")
            elif len(container_corners) >= 2:
                rospy.loginfo("CONTAINER FRONT OR BACK DETECTED!")

            publish_corners(container_corners, header)

        return lines

    # reset outdated markers
    publish_corners([], header)
    rospy.loginfo("NOTHING DETECTED!")
    return generate_default_line_marker(header)


def dist(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def container_side_detected(length):
    return length - CONTAINER_WIDTH * EPSILON <= CONTAINER_WIDTH <= length + CONTAINER_WIDTH * EPSILON


def publish_dgb_points(dbg, header):
    global DBG_PUB

    marker = Marker()
    marker.header = header
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    marker.points = dbg
    marker.scale.x = marker.scale.y = marker.scale.z = 0.08
    marker.color.a = marker.color.b = marker.color.g = 1.0

    DBG_PUB.publish(marker)


def publish_corners(intersections, header):
    global CORNER_PUB

    marker = Marker()
    marker.header = header
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    marker.points = intersections
    marker.scale.x = marker.scale.y = marker.scale.z = 0.4
    marker.color.a = marker.color.b = 1.0

    CORNER_PUB.publish(marker)


def intersection(line1, line2):
    rho1, theta1 = line1
    rho2, theta2 = line2

    # parallel lines don't intersect
    if -2 < abs(theta1 - theta2) < 2 or 178 < abs(theta1 - theta2) < 182:
        return

    # to radians
    theta1 = theta1 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    A = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    return x0, y0


def cloud_callback(cloud):
    global HOUGH_LINE_PUB

    rospy.loginfo("receiving cloud..")
    rospy.loginfo("SEQ: %s", cloud.header.seq)

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
    HOUGH_LINE_PUB.publish(lines)


def node():
    global HOUGH_LINE_PUB, CORNER_PUB, DBG_PUB

    rospy.init_node('hough_container')
    HOUGH_LINE_PUB = rospy.Publisher("/hough_lines", Marker, queue_size=1)
    CORNER_PUB = rospy.Publisher("/corner_points", Marker, queue_size=1)
    DBG_PUB = rospy.Publisher("/dbg_points", Marker, queue_size=1)
    rospy.Subscriber("/velodyne_points", PointCloud2, cloud_callback, queue_size=1, buff_size=2 ** 32)

    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
