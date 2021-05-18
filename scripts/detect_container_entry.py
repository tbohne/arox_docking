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

DBG_POINTS = []

DELTA_THETA = 2
THETA_MIN = 0
THETA_MAX = 180
# should be sufficiently precise to identify reasonable lines
DELTA_RADIUS = 0.01
# TODO: check sensor range
RADIUS_MIN = -10.0
RADIUS_MAX = 10.0
# TODO: distance to robot pos should be considered for the ACC_THRESH
ACC_THRESHOLD = 10


def generate_default_line_marker(header):
    """
    Initializes the marker for detected lines.

    :param header: point cloud header
    :return: initialized line marker
    """
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


def publish_dgb_points(header):
    """
    Publishes the debugging points marker stored in DBG_POINTS.

    :param header: point cloud header
    """
    global DBG_PUB, DBG_POINTS
    marker = Marker()
    marker.header = header
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    marker.points = DBG_POINTS
    marker.scale.x = marker.scale.y = marker.scale.z = 0.08
    marker.color.a = marker.color.b = marker.color.g = 1.0
    DBG_PUB.publish(marker)


def publish_corners(intersections, header):
    """
    Publishes the detected container corners as marker points.

    :param intersections: container corners (where the sides intersect)
    :param header: point cloud header
    """
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


def get_theta_by_index(idx):
    """
    Returns the theta value corresponding to the specified index (based on discretization).

    :param idx: index to retrieve theta value for
    :return: theta value for specified index
    """
    return idx * DELTA_THETA + THETA_MIN


def get_radius_by_index(idx):
    """
    Returns the radius value corresponding to the specified index (based on discretization).

    :param idx: index to retrieve radius value for
    :return: radius value for specified index
    """
    return idx * DELTA_RADIUS + RADIUS_MIN


def compute_y_coordinate(theta, radius, x):
    """
    Computes the y-coordinate of a given point based on the x-coordinate, theta, and radius.

    :param theta: theta value
    :param radius: radius value
    :param x: x-coordinate
    :return: y-coordinate
    """
    return (radius - np.cos(theta * np.pi / 180.0) * x) / np.sin(theta * np.pi / 180.0)


def retrieve_best_line(hough_space):
    """
    Retrieves the indices of the best line (peak) from the specified hough space.

    :param hough_space: hough space to retrieve peak from
    :return: indices of peak in hough space
    """
    max_idx = hough_space.argmax()
    c, r = np.unravel_index(max_idx, hough_space.shape)
    assert hough_space.max() == hough_space[c][r]
    return c, r


def append_line_points(theta, radius, lines):
    """
    Appends two points representing a detected line to the lines marker.

    :param theta: theta value of detected line
    :param radius: radius value of detected line
    :param lines: lines marker
    """
    p1 = Point()
    p2 = Point()
    p1.x = -100
    p2.x = 100

    if theta == 0:
        p1.y = 0
        p2.y = 0
    else:
        p1.y = compute_y_coordinate(theta, radius, p1.x)
        p2.y = compute_y_coordinate(theta, radius, p2.x)
    lines.points.append(p1)
    lines.points.append(p2)


def median_filter(points_on_line):
    """
    Filters the points on the specified line based on their distances to the median.
    [Not really a median filter - just removing far away points]

    :param points_on_line: list of points on a detected line
    :return: filtered list of points
    """
    median = Point()
    median.x = np.median(np.sort([p.x for p in points_on_line]))
    median.y = np.median(np.sort([p.y for p in points_on_line]))
    return [p for p in points_on_line if dist(p, median) <= CONTAINER_LENGTH]


def compute_avg_and_max_distance(point_list):
    """
    Computes the average and maximum distance between different points in the specified list.

    :param point_list: list of points to compute avg and max dist for
    :return: average distance, max distance
    """
    distances = [dist(i, j) for i in point_list for j in point_list if i != j]
    return np.average(distances), np.max(distances)


def reasonable_dist_to_already_detected_lines(point_list, avg_points):
    """
    Determines whether the detected line specified by the list of points has a reasonable distance
    to previously detected lines represented by the list of average points.
    Reasonable means that the distances do not contradict the dimensions of the container.

    :param point_list: points on the detected line
    :param avg_points: representing the average of previously detected lines
    :return: whether the detected line has a reasonable distance to previously detected lines
    """
    avg = Point()
    avg.x = np.average([p.x for p in point_list])
    avg.y = np.average([p.y for p in point_list])
    for p in avg_points:
        if dist(avg, p) > CONTAINER_LENGTH or dist(avg, p) < CONTAINER_WIDTH / 2:
            return False
    return True


def detect_jumps(points_on_line):
    """
    Detects jumps in the sorted lists of x- and y-values of a detected line.

    :param points_on_line: list of points representing a detected line
    :return: whether there are detected jumps in the sorted coordinates
    """
    x_vals = sorted([p.x for p in points_on_line])
    y_vals = sorted([p.y for p in points_on_line])
    jump_cnt = 0
    for i in range(len(x_vals) - 1):
        x_dist = abs(abs(x_vals[i]) - abs(x_vals[i + 1]))
        y_dist = abs(abs(y_vals[i]) - abs(y_vals[i + 1]))
        if x_dist > 2 or y_dist > 2:
            return True
        if x_dist > 0.7 or y_dist > 0.7:
            jump_cnt += 1
    return jump_cnt >= 3


def detected_reasonable_line(point_list, theta_base, theta, avg_points):
    """
    Determines whether the detected line represented by the given list of points fulfills several
    criteria checking for its plausibility in terms of being a feasible container side.

    :param point_list: list of points representing a detected line
    :param theta_base: theta value of the base line
    :param theta: theta value of the currently considered line
    :param avg_points: points representing the average of previously detected lines
    :return: whether the detected line is a plausible container side
    """
    diff = 90 if theta_base is None else abs(theta - theta_base)
    # should be either ~90° or ~270° if base is entry
    # (base is not necessarily the entry of the container -> ~0 and ~180 ok as well)
    orthogonal_to_base = diff < 2 or 88 < diff < 92 or 178 < diff < 182 or 268 < diff < 272
    avg_dist, max_dist = compute_avg_and_max_distance(point_list)
    reasonable_dist = reasonable_dist_to_already_detected_lines(point_list, avg_points)
    tolerated_length = CONTAINER_LENGTH + CONTAINER_LENGTH * EPSILON
    tolerated_width = CONTAINER_WIDTH - CONTAINER_WIDTH * EPSILON
    reasonable_len = tolerated_length >= max_dist >= tolerated_width
    reasonable_avg_distances = CONTAINER_WIDTH / 2 >= avg_dist >= 0.5
    jumps = False  # detect_jumps(point_list)
    return reasonable_len and reasonable_dist and reasonable_avg_distances and orthogonal_to_base and not jumps


def compute_intersection_points(found_line_params):
    """
    Computes intersection points for the specified lines.

    :param found_line_params: parameters of the lines to compute intersections for
    :return: intersection points
    """
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


def update_avg_points(avg_points, point_list):
    """
    Saves the average point for each line.

    :param avg_points: list of average points
    :param point_list: list of points representing currently detected line (to be added to the list)
    """
    avg = Point()
    avg.x = np.average([p.x for p in point_list])
    avg.y = np.average([p.y for p in point_list])
    avg_points.append(avg)


def already_tested_base_line(tested_base_lines, radius, theta):
    """
    Checks whether the line (or a very similar line) represented by radius and theta was
    considered as base line for the container before.

    :param tested_base_lines: parameters of already tested base lines
    :param radius: radius value of currently considered line
    :param theta: theta value of currently considered line
    :return: whether the line was already considered as base line
    """
    for r, t in tested_base_lines:
        diff_r = abs(abs(radius) - abs(r))
        diff_t = abs(abs(theta) - abs(t))
        if diff_r < DELTA_RADIUS * 10 and diff_t < 60:
            return True
    return False


def append_and_publish_dbg_points(point_list, header):
    """
    Appends the specified points to the list of debug points to be visualized as markers
    and publishes the list.

    :param point_list: list of points to be added to the dbg points
    :param header: point cloud header
    """
    global DBG_POINTS
    DBG_POINTS.extend(point_list)
    publish_dgb_points(header)


def dist(p1, p2):
    """
    Computes the Euclidean distance between the specified points.

    :param p1: point one
    :param p2: point two
    :return: Euclidean distance
    """
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def container_front_or_back_detected(length):
    """
    Returns whether the specified length corresponds to the container front / back.

    :param length: length to be compared to the container dimensions
    :return: whether the length corresponds to the container front / back
    """
    return length - CONTAINER_WIDTH * EPSILON <= CONTAINER_WIDTH <= length + CONTAINER_WIDTH * EPSILON


def intersection(line1, line2):
    """
    Computes the intersection between the specified lines.

    :param line1: line one
    :param line2: line two
    :return: x- and y-coordinate of the intersection point (or None if not intersecting)
    """
    rho1, theta1 = line1
    rho2, theta2 = line2
    # parallel lines don't intersect
    if -2 < abs(theta1 - theta2) < 2 or 178 < abs(theta1 - theta2) < 182:
        return

    # to radians
    theta1 = theta1 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    theta_matrix = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    rho_vector = np.array([[rho1], [rho2]])
    x, y = np.linalg.solve(theta_matrix, rho_vector)
    return x, y


def line_corresponds_to_base_line(point_list, theta_base, theta, avg_points, found_line_params, radius):
    """
    Determines whether the detected line corresponds to the base line in the sense that the combination
    of the two satisfies the shape criteria of the container.

    :param point_list: points of the detected line
    :param theta_base: theta value of the base line
    :param theta: theta value of the detected line
    :param avg_points: avg points representing previously detected lines
    :param found_line_params: parameters of previously detected lines
    :param radius: radius value of the detected line
    :return: whether the detected line corresponds to the base line
    """
    if len(point_list) <= ACC_THRESHOLD or not detected_reasonable_line(point_list, theta_base, theta, avg_points):
        return False

    for r, t in found_line_params:
        # "equal" radius -> angle has to be different
        if abs(abs(r) - abs(radius)) < DELTA_RADIUS * 10:
            # angle too close -> no container side
            if abs(theta - t) < 60 or 120 < abs(theta - t) < 240:
                return False

    # check intersections
    tmp = found_line_params.copy()
    tmp.append((radius, theta))
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
                    return False
    return True


def retrieve_container_corners(found_line_params):
    """
    Retrieves the container corners based on the found line parameters.

    :param found_line_params: parameters of detected lines
    :return: detected corner points of the container
    """
    intersections = compute_intersection_points(found_line_params)
    container_corners = []
    for p in intersections:
        for j in intersections:
            if p != j:
                d = dist(p, j)
                if container_front_or_back_detected(d):
                    container_corners.append(p)
    return container_corners


def publish_detected_container(found_line_params, header):
    """
    Publishes the detected corners of the container.

    :param found_line_params: parameters of detected lines
    :param header: point cloud header
    """
    rospy.loginfo("parameters of detected lines: %s", found_line_params)
    container_corners = retrieve_container_corners(found_line_params)
    if len(container_corners) > 0:
        if len(container_corners) == 4:
            rospy.loginfo("CONTAINER DETECTED!")
        elif len(container_corners) >= 2:
            rospy.loginfo("CONTAINER FRONT OR BACK DETECTED!")
        publish_corners(container_corners, header)


def detect_container(hough_space, header, corresponding_points):
    """
    Detects the container based on lines in the specified hough space.

    :param hough_space: result of the hough transform of the point cloud
    :param header: point cloud header
    :param corresponding_points: dictionary containing a list of points corresponding to each (rad, theta) combination
    :return: line marker representing the container sides
    """
    global DBG_POINTS

    base_lines_tested = 0
    base_attempts = 0
    tested_base_lines = []

    while hough_space.max() > ACC_THRESHOLD and base_lines_tested < 10 and base_attempts < 150:
        DBG_POINTS = []
        lines = generate_default_line_marker(header)
        rospy.loginfo("testing new base line with %s points", hough_space.max())
        # base line parameters
        c, r = retrieve_best_line(hough_space)
        point_list = median_filter(np.array(corresponding_points[(c, r)]))
        theta_base = get_theta_by_index(r)
        radius = get_radius_by_index(c)
        already_tested = already_tested_base_line(tested_base_lines, radius, theta_base)
        reasonable_line = detected_reasonable_line(point_list, None, theta_base, [])

        if already_tested or len(point_list) <= ACC_THRESHOLD or not reasonable_line:
            hough_space[c][r] = 0
            base_attempts += 1
            rospy.loginfo("base attempt: %s", base_attempts)
            continue

        # visualize base line
        append_and_publish_dbg_points(point_list, header)

        append_line_points(theta_base, radius, lines)
        found_line_params = [(radius, theta_base)]
        avg_points = []
        update_avg_points(avg_points, point_list)
        hough_space[c][r] = 0
        hough_copy = hough_space.copy()
        base_lines_tested += 1
        tested_base_lines.append((radius, theta_base))

        # detect remaining 2-3 sides
        while len(found_line_params) < 4 and hough_copy.max() > ACC_THRESHOLD:
            c, r = retrieve_best_line(hough_copy)
            radius = get_radius_by_index(c)
            point_list = median_filter(np.array(corresponding_points[(c, r)]))
            theta = get_theta_by_index(r)

            if line_corresponds_to_base_line(point_list, theta_base, theta, avg_points, found_line_params, radius):
                append_and_publish_dbg_points(point_list, header)
                append_line_points(theta, radius, lines)
                found_line_params.append((radius, theta))
                update_avg_points(avg_points, point_list)
            hough_copy[c][r] = 0

        if len(found_line_params) >= 3:
            publish_detected_container(found_line_params, header)
            return lines

    # reset outdated markers
    publish_corners([], header)
    rospy.loginfo("NOTHING DETECTED!")
    return generate_default_line_marker(header)


def cloud_callback(cloud):
    """
    Is called whenever a new point cloud arrives.
    Computes the 2D hough transform of the point cloud (x, y) and initiates the container detection.

    :param cloud: point cloud to detect container in
    """
    global HOUGH_LINE_PUB
    rospy.loginfo("receiving cloud..")
    rospy.loginfo("seq: %s", cloud.header.seq)

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
                r = x * cos_theta + y * sin_theta
                if r < RADIUS_MIN or r > RADIUS_MAX:
                    continue
                r_idx = int((r - RADIUS_MIN) / DELTA_RADIUS)
                hough_space[r_idx][theta_idx] += 1
                p = Point()
                p.x = x
                p.y = y
                corresponding_points[(r_idx, theta_idx)].append(p)

    lines = detect_container(hough_space, cloud.header, corresponding_points)
    rospy.loginfo(cloud.header)
    HOUGH_LINE_PUB.publish(lines)


def node():
    """
    Node to detect the container entry in a point cloud.
    """
    global HOUGH_LINE_PUB, CORNER_PUB, DBG_PUB

    rospy.init_node('detect_container_entry')
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
