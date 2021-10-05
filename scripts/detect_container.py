#!/usr/bin/env python
import actionlib
import numpy as np
import rospy
import tf2_ros
from arox_docking import config
from arox_docking.msg import DetectAction, DetectResult
from arox_docking.msg import PointArray
from arox_docking.util import dist, transform_pose
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

TF_BUFFER = None


class DetectionServer:
    """
    Simple action server that detects the container in a laser scan using a Hough transform.
    """

    def __init__(self):
        self.server = actionlib.SimpleActionServer('detect_container', DetectAction, self.execute, False)
        self.server.start()
        self.scan_sub = None
        self.pose_sub = None
        self.line_pub = None
        self.corner_pub = None
        self.dbg_pub = None
        self.clear_pub = None
        self.corners = None
        self.detecting = False
        self.robot_pos = None

    def reset(self):
        """
        Resets detection related values and initializes publishers and subscribers.
        """
        # reset detected corners
        self.corners = None
        self.detecting = False
        # subscribe to the scan that is the result of 'pointcloud_to_laserscan'
        self.scan_sub = rospy.Subscriber("/scanVelodyneFrame", LaserScan, self.scan_callback, queue_size=1)
        # subscribe to robot pose
        self.pose_sub = rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)

        # publishers for debugging markers (visualizations)
        self.line_pub = rospy.Publisher("/publish_lines", PointArray, queue_size=1)
        self.corner_pub = rospy.Publisher("/publish_corners", PointArray, queue_size=1)
        self.dbg_pub = rospy.Publisher("/publish_dbg", PointArray, queue_size=1)
        self.clear_pub = rospy.Publisher("/clear_markers", String, queue_size=1)

    def execute(self, goal):
        """
        Executes the action server.

        :param goal: goal to be performed (dummy atm)
        """
        self.reset()
        result = DetectResult()

        # give it some time to detect the container, but check after every CHECK_FREQUENCY whether it's done
        for i in range(int(config.TIME_LIMIT / config.CHECK_FREQUENCY)):
            if self.corners:
                break
            rospy.sleep(config.CHECK_FREQUENCY)

        if self.corners:
            rospy.loginfo("CORNER DETECTION SUCCEEDED..")
            self.scan_sub.unregister()
            self.pose_sub.unregister()
            result.corners = self.corners
            self.server.set_succeeded(result)
        else:
            rospy.loginfo("CORNER DETECTION FAILED..")
            self.scan_sub.unregister()
            self.pose_sub.unregister()
            self.server.set_aborted()

    def line_dist_to_robot(self, point_list):
        """
        Computes the distance between the considered line (avg point) and the robot.

        :param point_list: points of the considered line
        :return: distance between line and robot
        """
        return dist(self.robot_pos, compute_avg_point(point_list)) if len(point_list) > 0 else config.DEFAULT_DIST

    def detect_container(self, hough_space, points):
        """
        Detects the container based on lines in the specified hough space.

        :param hough_space: result of the hough transform of the laser scan
        :param points: dictionary containing a list of points corresponding to each (rad, theta) combination
        """
        c, r = retrieve_best_line(hough_space)
        point_list = median_filter(np.array(points[(c, r)]))
        dynamic_acc_thresh_based_on_dist = determine_thresh_based_on_dist_to_robot(self.line_dist_to_robot(point_list))

        base_attempts = 0
        tested_base_lines = []

        while hough_space.max() >= config.LINE_LB and len(
                tested_base_lines) < config.BASE_TEST_LIMIT and base_attempts < config.BASE_ATTEMPT_LIMIT:
            # clear markers of previous detection run
            self.dbg_pub.publish([])
            self.line_pub.publish([])
            self.corner_pub.publish([])

            # base line parameters
            c, r = retrieve_best_line(hough_space)
            point_list = median_filter(np.array(points[(c, r)]))
            dist_to_robot = self.line_dist_to_robot(point_list)
            dynamic_acc_thresh_based_on_dist = determine_thresh_based_on_dist_to_robot(dist_to_robot)

            theta_base = get_theta_by_index(r)
            radius = get_radius_by_index(c)
            already_tested = already_tested_line(tested_base_lines, radius, theta_base)
            reasonable_line = detected_reasonable_line(point_list, None, theta_base, [])

            if already_tested or len(point_list) <= dynamic_acc_thresh_based_on_dist or not reasonable_line:
                hough_space[c][r] = 0
                base_attempts += 1
                rospy.loginfo("base attempt: %s", base_attempts)
                continue

            # visualize base line
            self.dbg_pub.publish(point_list)
            p1, p2 = compute_line_points(theta_base, radius)
            self.line_pub.publish([p1, p2])

            found_line_params = [(radius, theta_base)]
            _, length = compute_avg_and_max_distance(point_list)
            line_lengths = [length]
            avg_points = []
            update_avg_points(avg_points, point_list)
            hough_space[c][r] = 0
            hough_copy = np.copy(hough_space)
            rospy.loginfo("base lines tested: %s", len(tested_base_lines))
            tested_base_lines.append((radius, theta_base))

            # TODO: potential problem: if something didn't work out as base line, it is set to 0 and not used at all
            base_avg = compute_avg_point(point_list)
            tested_lines = []

            # detect remaining 2-3 sides
            while len(found_line_params) < 4 and hough_copy.max() > dynamic_acc_thresh_based_on_dist:
                c, r = retrieve_best_line(hough_copy)
                radius = get_radius_by_index(c)
                theta = get_theta_by_index(r)
                point_list = median_filter(np.array(points[(c, r)]))
                already_used = already_tested_line(found_line_params, radius, theta) or already_tested_line(
                    tested_lines, radius, theta)
                too_far_away = dist(compute_avg_point(point_list),
                                    base_avg) > config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON
                tested_lines.append((radius, theta))

                if already_used or too_far_away:
                    hough_copy[c][r] = 0
                    continue

                dist_to_robot = self.line_dist_to_robot(point_list)
                dynamic_acc_thresh_based_on_dist = determine_thresh_based_on_dist_to_robot(dist_to_robot)

                # if we already found two lines, we are not that strict for the last two
                # -> accept lines with fewer points (LB) if they correspond to the others
                if len(found_line_params) >= 2 and dynamic_acc_thresh_based_on_dist > config.LINE_LB:
                    dynamic_acc_thresh_based_on_dist = config.LINE_LB

                infeasible_line = len(point_list) <= dynamic_acc_thresh_based_on_dist or not detected_reasonable_line(
                    point_list, theta_base, theta, avg_points)

                if not infeasible_line and line_corresponds_to_already_detected_lines(theta, found_line_params, radius):
                    _, length = compute_avg_and_max_distance(point_list)
                    if feasible_line_length(line_lengths, length):
                        self.dbg_pub.publish(point_list)
                        p1, p2 = compute_line_points(theta, radius)
                        self.line_pub.publish([p1, p2])
                        found_line_params.append((radius, theta))
                        line_lengths.append(length)
                        update_avg_points(avg_points, point_list)
                hough_copy[c][r] = 0

            # at least two corners found
            rospy.loginfo("FOUND LINE PARAMS: %s", len(found_line_params))
            if len(found_line_params) >= 3:
                tmp = self.publish_detected_container(found_line_params)
                if len(tmp) >= 2:
                    # always adding the corners AND the avg points for each line
                    self.corners = tmp
                    for p in avg_points:
                        self.corners.append(p)
                break
        rospy.loginfo("STOPPING - HOUGH MAX: %s, ACC_THRESH: %s", hough_space.max(), dynamic_acc_thresh_based_on_dist)
        self.detecting = False

    def scan_callback(self, scan):
        """
        Is called whenever a new laser scan arrives.
        Computes the 2D hough transform of the laser scan and initiates the container detection.

        :param scan: laser scan to detect container in
        """
        # rospy.loginfo("receiving laser scan..")
        # rospy.loginfo("seq: %s", scan.header.seq)

        # only detecting container if it's not already detected
        if self.corners is None and not self.detecting:
            self.detecting = True
            theta_values = np.deg2rad(
                np.arange(config.THETA_MIN, config.THETA_MAX, config.DELTA_THETA, dtype=np.double))
            r_values = np.arange(config.RADIUS_MIN, config.RADIUS_MAX, config.DELTA_RADIUS, dtype=np.double)
            # precompute sines and cosines
            cosines = np.cos(theta_values)
            sines = np.sin(theta_values)
            # define discrete hough space
            hough_space = np.zeros([len(r_values), len(theta_values)], dtype=int)

            # init dictionary for corresponding points
            points = {}
            for r in range(len(r_values)):
                for theta in range(len(theta_values)):
                    points[(r, theta)] = []

            # compute hough space
            for p_idx, (x, y) in enumerate(get_points_from_scan(scan)):
                if not np.isnan(x) and not np.isnan(y):
                    for theta_idx, (cos_theta, sin_theta) in enumerate(zip(cosines, sines)):
                        r = x * cos_theta + y * sin_theta
                        if r < config.RADIUS_MIN or r > config.RADIUS_MAX:
                            continue
                        r_idx = int((r - config.RADIUS_MIN) / config.DELTA_RADIUS)
                        hough_space[r_idx][theta_idx] += 1
                        p = Point()
                        p.x = x
                        p.y = y
                        points[(r_idx, theta_idx)].append(p)

            self.detect_container(hough_space, points)

    def publish_detected_container(self, found_line_params):
        """
        Publishes the detected corners of the container and returns them.

        :param found_line_params: parameters of detected lines
        :return: detected corners of the container
        """
        # rospy.loginfo("parameters of detected lines: %s", found_line_params)
        container_corners = retrieve_container_corners(found_line_params)
        self.corner_pub.publish(container_corners)
        return container_corners

    def odom_callback(self, odom):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update the robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        self.robot_pos = pose_stamped.pose.position


def get_theta_by_index(idx):
    """
    Returns the theta value corresponding to the specified index (based on discretization).

    :param idx: index to retrieve theta value for
    :return: theta value for specified index
    """
    return idx * config.DELTA_THETA + config.THETA_MIN


def get_radius_by_index(idx):
    """
    Returns the radius value corresponding to the specified index (based on discretization).

    :param idx: index to retrieve radius value for
    :return: radius value for specified index
    """
    return idx * config.DELTA_RADIUS + config.RADIUS_MIN


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


def compute_line_points(theta, radius):
    """
    Computes two points representing a detected line.

    :param theta: theta value of detected line
    :param radius: radius value of detected line
    :return: two points representing the detected line
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
    return p1, p2


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
    res = [p for p in points_on_line if dist(p, median) <= config.CONTAINER_LENGTH]
    if len(res) > 300:
        filtered = []
        size = 5
        for i in range(size, len(res), size):
            avg = Point()
            avg.x = np.average([res[j].x for j in range(i - size, i)])
            avg.y = np.average([res[j].y for j in range(i - size, i)])
            filtered.append(avg)
        res = filtered
    return res


def compute_avg_point(point_list):
    """
    Computes the average of the specified list of points.

    :param point_list: list of points to compute average for
    :return: avg point
    """
    avg = Point()
    avg.x = np.average([p.x for p in point_list])
    avg.y = np.average([p.y for p in point_list])
    return avg


def compute_avg_and_max_distance(point_list):
    """
    Computes the average and maximum distance between different points in the specified list.

    :param point_list: list of points to compute avg and max dist for
    :return: average distance, max distance
    """
    distances = [dist(i, j) for i in point_list for j in point_list if i != j]
    if len(distances) > 0:
        return np.average(distances), np.max(distances)
    return 0, 0


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
        eps_length = config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON
        # TODO: the LB is quite dangerous here -> could lead to missed sides (but also kinda necessary)
        if dist(avg, p) > eps_length or dist(avg, p) < config.CONTAINER_WIDTH / 3:
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
    # should be either ~90 or ~270 if base is entry
    # (base is not necessarily the entry of the container -> ~0 and ~180 ok as well)
    orthogonal_to_base = diff < 2 or 88 < diff < 92 or 178 < diff < 182 or 268 < diff < 272
    avg_dist, max_dist = compute_avg_and_max_distance(point_list)
    reasonable_dist = reasonable_dist_to_already_detected_lines(point_list, avg_points)
    tolerated_UB = config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON * 2
    # shouldn't be too restrictive at the lower bound (partially detected lines etc.)
    tolerated_LB = 0.5
    reasonable_len = tolerated_UB >= max_dist >= tolerated_LB

    # TODO: perhaps not so useful
    reasonable_avg_distances = True  # config.CONTAINER_WIDTH / 2 >= avg_dist >= 0.5
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


def already_tested_line(tested_lines, radius, theta):
    """
    Checks whether the line (or a very similar line) represented by radius and theta was considered before.

    :param tested_lines: parameters of already tested lines
    :param radius: radius value of currently considered line
    :param theta: theta value of currently considered line
    :return: whether the line was already considered
    """
    for r, t in tested_lines:
        diff_r = abs(abs(radius) - abs(r))
        diff_t = abs(abs(theta) - abs(t))
        # TODO: check whether 10 * RAD actually makes sense
        if diff_r < config.DELTA_RADIUS_CHECK * 2 and diff_t < 80:
            return True
    return False


def container_front_or_back_detected(length):
    """
    Returns whether the specified length corresponds to the container front / back.

    :param length: length to be compared to the container dimensions
    :return: whether the length corresponds to the container front / back
    """
    return length - config.CONTAINER_WIDTH * config.EPSILON <= config.CONTAINER_WIDTH <= length + config.CONTAINER_WIDTH * config.EPSILON


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


def feasible_distances(detected, theta, radius):
    """
    Measures the distances between the newly detected line and the previously detected ones parallel to the new one
    and determines feasibility based on the polar coordinate distances.

    :param detected: parameters of previously detected lines
    :param theta: theta value of the detected line
    :param radius: radius value of the detected line
    :return: whether the newly detected line corresponds to the others
    """
    for r, t in detected:
        # parallel to an already detected line (parallel lines have the same angle)
        if t == theta:
            already_detected = Point()
            already_detected.x = r * np.cos(t)
            already_detected.y = r * np.sin(t)

            new = Point()
            new.x = radius * np.cos(theta)
            new.y = radius * np.sin(theta)

            d = dist(already_detected, new)

            if d < (config.CONTAINER_WIDTH - config.CONTAINER_WIDTH * config.EPSILON) or d > (
                    config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON):
                return False
    return True


def feasible_angles(detected, theta):
    """
    There can only be two lines with roughly the same angle belonging to the container. This fact is used in order
    to determine whether the newly considered line could still be part of the container.

    :param detected: parameters of previously detected lines
    :param theta: theta value of the detected line
    :return: whether the newly detected line could still be part of the container
    """
    same_angle_lines = 0
    for _, t in detected:
        if abs(theta - t) <= 10:
            same_angle_lines += 1
    # already >= 2 lines with roughly the same angle -> not part of the container
    if same_angle_lines >= 2:
        return False
    return True


def feasible_orientation(detected, radius, theta):
    """
    Determines whether the newly detected line has a feasible orientation towards the previously detected ones.

    :param detected: parameters of previously detected lines
    :param radius: radius value of the detected line
    :param theta: theta value of the detected line
    :return: whether the newly detected line has a feasible orientation
    """
    for r, t in detected:
        # "equal" radius -> angle has to be different
        # TODO: check whether 4 is appropriate
        if abs(abs(r) - abs(radius)) < config.DELTA_RADIUS_CHECK * 4:
            # angle too close -> no container side
            if abs(theta - t) < 60 or 120 < abs(theta - t) < 240:
                return False
    return True


def feasible_intersections(detected, radius, theta):
    """
    Determines whether the newly detected line has feasible intersections with the previously detected ones.

    :param detected: parameters of previously detected lines
    :param radius: radius value of the detected line
    :param theta: theta value of the detected line
    :return: whether the newly detected line has feasible intersections with the previous ones
    """
    tmp = list(detected)
    tmp.append((radius, theta))
    intersections = compute_intersection_points(tmp)
    for p in intersections:
        for j in intersections:
            if p != j:
                d = dist(p, j)
                width_eps = config.CONTAINER_WIDTH * config.EPSILON
                length_eps = config.CONTAINER_LENGTH * config.EPSILON
                tolerated_width = config.CONTAINER_WIDTH - width_eps < d < config.CONTAINER_WIDTH + width_eps
                # TODO: check hard coded additional length_eps -> was necessary occasionally
                tolerated_length = config.CONTAINER_LENGTH - length_eps < d < config.CONTAINER_LENGTH + length_eps * 2
                if not tolerated_width and not tolerated_length:
                    return False
    return True


def feasible_line_length(line_lengths, curr_length):
    """
    Determines whether the length of the currently considered container side candidate corresponds to
    the already detected ones (i.e. for each side, CONTAINER_LENGTH and CONTAINER_WIDTH, we only allow 2)

    :param line_lengths: lengths of already detected lines
    :param curr_length: length of currently considered line candidate
    :return: whether length of currently considered line feasibly corresponds to already detected lines
    """
    length_eps = config.CONTAINER_LENGTH * config.EPSILON
    # width epsilon at upper bound should be very low because of partially detected long sides
    width_eps = config.CONTAINER_WIDTH * 0.05
    # can't be too strict here due to partially detected lines
    if 0.5 < curr_length < config.CONTAINER_WIDTH + width_eps:
        # check whether we have at most one short side detected yet
        cnt = 0
        for length in line_lengths:
            if 0.5 < length < config.CONTAINER_WIDTH + width_eps:
                cnt += 1
        if cnt > 1:
            return False
    elif config.CONTAINER_WIDTH + width_eps < curr_length < config.CONTAINER_LENGTH + length_eps * 2:
        # check whether we have at most one long side detected yet
        cnt = 0
        for length in line_lengths:
            if config.CONTAINER_WIDTH + width_eps < length < config.CONTAINER_LENGTH + length_eps * 2:
                cnt += 1
        if cnt > 1:
            return False
    else:
        return False
    return True


def line_corresponds_to_already_detected_lines(theta, detected, radius):
    """
    Determines whether the newly detected line corresponds to the previously detected lines in the sense that the
    combination of them satisfies the shape criteria of the container.

    :param theta: theta value of the detected line
    :param detected: parameters of previously detected lines
    :param radius: radius value of the detected line
    :return: whether the newly detected line corresponds to the previously detected ones
    """
    return feasible_distances(detected, theta, radius) and feasible_angles(detected, theta) and feasible_orientation(
        detected, radius, theta) and feasible_intersections(detected, radius, theta)


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
                    if p not in container_corners:
                        container_corners.append(p)
    return container_corners


def determine_thresh_based_on_dist_to_robot(dist_to_robot):
    """
    Determines a dynamic threshold for the accumulator array based on the robot's distance to the considered shape.

    :param dist_to_robot: distance of the shape (line candidate) to the robot
    :return: accumulator threshold
    """
    # TODO: to be refined based on experiments
    if dist_to_robot < 1:
        return 50
    elif dist_to_robot < 2:
        return 25
    elif dist_to_robot < 3:
        return 20
    elif dist_to_robot < 4:
        return 15
    elif dist_to_robot < 6:
        return 10
    else:
        return 5


def get_points_from_scan(scan):
    """
    Retrieves the points from the specified laser scan.

    :param scan: laser scan to retrieve points from
    :return: points from scan
    """
    points = []
    for index, point in enumerate(scan.ranges):
        if point < scan.range_min or point >= scan.range_max:
            continue

        # transform points from polar to cartesian coordinates
        x = point * np.cos(scan.angle_min + index * scan.angle_increment)
        y = point * np.sin(scan.angle_min + index * scan.angle_increment)
        points.append((x, y))

    return points


def node():
    """
    Node to detect the container entry in a laser scan.
    """
    global TF_BUFFER
    rospy.init_node("detect_container")
    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)
    server = DetectionServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
