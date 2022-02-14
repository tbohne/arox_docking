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
    Simple action server that performs a Hough transform to detect lines in a laser scan and uses a large number of
    tailored rules to find a reasonable combination of line segments that could represent the container. Essentially,
    the whole approach is centered around the idea that the container is a relatively simple geometric shape that can be
    detected in a laser scan - a combination of line segments.
    """

    def __init__(self):
        self.server = actionlib.SimpleActionServer('detect_container', DetectAction, self.execute, False)
        self.server.start()
        self.pose_sub = None
        self.corners = None
        self.robot_pos = None

        # publishers for debugging markers (visualizations)
        self.line_pub = rospy.Publisher("/publish_lines", PointArray, queue_size=1)
        self.corner_pub = rospy.Publisher("/publish_corners", PointArray, queue_size=1)
        self.dbg_pub = rospy.Publisher("/publish_dbg", PointArray, queue_size=1)
        self.dbg_pub_base = rospy.Publisher("/publish_dbg_base", PointArray, queue_size=1)
        self.clear_pub = rospy.Publisher("/clear_markers", String, queue_size=1)

    def reset(self):
        """
        Resets detection related values and renews robot pose subscription.
        """
        # reset detected corners
        self.corners = None
        # subscribe to robot pose
        self.pose_sub = rospy.Subscriber("/odometry/filtered_odom", Odometry, self.odom_callback, queue_size=1)

    def execute(self, goal: DetectAction):
        """
        Executes the container detection action server.

        :param goal: goal to be performed - contains information of whether to use full of partial scan
        """
        self.reset()
        result = DetectResult()
        scan = None

        if goal.scan_mode == "full":
            try:
                # create a new subscription to the topic, receive one message, then unsubscribe
                # --> specially configured scan, cf. pointcloud_to_laserscan_sim_frame.launch in "arox_indoor_navi"
                scan = rospy.wait_for_message("/scanVelodyneFrame", LaserScan, timeout=10)
                rospy.loginfo("performing container detection on full laser scan [-100, 100]")
            except rospy.ROSException as e:
                rospy.loginfo("problem retrieving full laser scan: %s", e)
        elif goal.scan_mode == "partial":
            try:
                # create a new subscription to the topic, receive one message, then unsubscribe
                # --> specially configured scan, cf. pointcloud_to_laserscan_sim_frame_partial.launch
                scan = rospy.wait_for_message("/scanVelodyneFramePartial", LaserScan, timeout=10)
                rospy.loginfo(
                    "performing container detection on partial laser scan [-45, 45] -- already aligned in front of it")
            except rospy.ROSException as e:
                rospy.loginfo("problem retrieving partial laser scan: %s", e)
        else:
            rospy.loginfo("unknown scan mode: %s", goal.scan_mode)

        self.prepare_container_detection(scan)

        if self.corners:
            rospy.loginfo("CORNER DETECTION SUCCEEDED..")
            self.pose_sub.unregister()
            result.corners = self.corners
            self.server.set_succeeded(result)
        else:
            rospy.loginfo("CORNER DETECTION FAILED..")
            self.pose_sub.unregister()
            self.server.set_aborted()

    def prepare_container_detection(self, scan: LaserScan):
        """
        Computes the 2D Hough transform of the laser scan and initiates the container detection.

        :param scan: laser scan to detect container in
        """
        theta_values = np.deg2rad(np.arange(config.THETA_MIN, config.THETA_MAX, config.DELTA_THETA, dtype=np.double))
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

    def line_dist_to_robot(self, point_list: list) -> float:
        """
        Computes the distance between the considered line (avg point) and the robot.

        :param point_list: points of the considered line
        :return: distance between line and robot
        """
        return dist(self.robot_pos, compute_avg_point(point_list)) if len(point_list) > 0 else config.DEFAULT_DIST

    def clear_markers(self):
        """
        Clears the markers of the previous detection run.
        """
        self.dbg_pub.publish([])
        self.line_pub.publish([])
        self.corner_pub.publish([])

    def visualize_baseline(self, point_list: list, theta_base: int, radius: int):
        """
        Visualizes the baseline candidate in rviz.

        :param point_list: list of points on the considered line
        :param theta_base: theta value of considered line
        :param radius: r value of considered line
        """
        self.dbg_pub_base.publish([])
        self.dbg_pub_base.publish(point_list)
        p1, p2 = compute_line_points(theta_base, radius)
        self.line_pub.publish([p1, p2])

    def detect_remaining_sides(self, line_params: list, hough: np.array, points: dict, tested_lines: list,
                               base_avg: Point, theta_base: int, avg_points: list, line_lengths: list,
                               dynamic_acc_thresh: int):
        """
        Detects the remaining 2-3 container sides that correspond to the considered baseline.

        :param line_params: parameters of found lines (r, theta)
        :param hough: hough space copy
        :param points: dictionary containing a list of points corresponding to each (r, theta) combination
        :param tested_lines: list of already tested lines
        :param base_avg: avg point on baseline
        :param theta_base: theta value of baseline
        :param avg_points: avg points of previously detected lines
        :param line_lengths: lengths of already detected lines
        :param dynamic_acc_thresh: threshold for accumulator array
        """
        while len(line_params) < 4 and hough.max() > dynamic_acc_thresh:
            c, r = retrieve_best_line(hough)
            radius = get_radius_by_index(c)
            theta = get_theta_by_index(r)
            point_list = median_filter(np.array(points[(c, r)]))

            already_used = already_found_line(line_params, radius, theta) \
                           or already_tested_line(tested_lines, radius, theta)

            too_far_away = dist(compute_avg_point(point_list), base_avg) \
                           > config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON

            tested_lines.append((radius, theta))

            if already_used or too_far_away:
                hough[c][r] = 0
                continue

            dist_to_robot = self.line_dist_to_robot(point_list)
            dynamic_acc_thresh = determine_thresh_based_on_dist_to_robot(dist_to_robot)

            # if we already found two lines, we are not that strict for the last two
            # -> accept lines with fewer points (LB) if they correspond to the others
            if len(line_params) >= 2 and dynamic_acc_thresh > config.LINE_LB:
                dynamic_acc_thresh = config.LINE_LB

            infeasible_line = len(point_list) <= dynamic_acc_thresh or not detected_reasonable_line(
                point_list, theta_base, theta, avg_points)

            self.dbg_pub.publish([])
            self.dbg_pub.publish(point_list)

            if not infeasible_line and line_corresponds_to_already_detected_lines(theta, line_params, radius):
                _, length = compute_avg_and_max_distance(point_list)
                if feasible_line_length(line_lengths, length):
                    p1, p2 = compute_line_points(theta, radius)
                    self.line_pub.publish([p1, p2])
                    line_params.append((radius, theta))
                    line_lengths.append(length)
                    update_avg_points(avg_points, point_list)
                elif config.VERBOSE_LOGGING:
                    rospy.loginfo("infeasible line length..")
            else:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("-------------------------")
                    rospy.loginfo("infeasible..")
                    matches = line_corresponds_to_already_detected_lines(theta, line_params, radius)
                    rospy.loginfo("line matches already detected ones: %s", matches)
                    rospy.loginfo("reasonable line: %s", infeasible_line)
                    rospy.loginfo("-------------------------")
                detected_reasonable_line(point_list, theta_base, theta, avg_points)
            hough[c][r] = 0

    def detect_container(self, hough_space: np.array, points: dict):
        """
        Detects the container based on lines in the specified Hough space.

        :param hough_space: result of the Hough transform of the laser scan
        :param points: dictionary containing a list of points corresponding to each (r, theta) combination
        """
        base_attempts = 0
        tested_base_lines = []

        while hough_space.max() >= config.LINE_LB and len(
                tested_base_lines) < config.BASE_TEST_LIMIT and base_attempts < config.BASE_ATTEMPT_LIMIT:

            self.clear_markers()

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
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("base attempt: %s", base_attempts)
                continue

            self.visualize_baseline(point_list, theta_base, radius)

            found_line_params = [(radius, theta_base)]
            _, length = compute_avg_and_max_distance(point_list)
            line_lengths = [length]
            avg_points = []
            update_avg_points(avg_points, point_list)
            hough_space[c][r] = 0
            hough_copy = np.copy(hough_space)

            if config.VERBOSE_LOGGING:
                rospy.loginfo("base lines tested: %s", len(tested_base_lines))
            tested_base_lines.append((radius, theta_base))
            base_avg = compute_avg_point(point_list)
            tested_lines = []

            self.detect_remaining_sides(found_line_params, hough_copy, points, tested_lines, base_avg, theta_base,
                                        avg_points, line_lengths, dynamic_acc_thresh_based_on_dist)

            # at least two corners found (three lines)
            if len(found_line_params) >= 3:
                tmp = self.publish_detected_container(found_line_params)
                if len(tmp) >= 2:
                    # always adding the corners AND the avg points for each line
                    self.corners = tmp
                    for p in avg_points:
                        self.corners.append(p)
                break
        rospy.loginfo("STOPPING - HOUGH MAX: %s, ACC THRESH: %s", hough_space.max(), dynamic_acc_thresh_based_on_dist)

    def publish_detected_container(self, found_line_params: list) -> list:
        """
        Publishes the detected corners of the container and returns them.

        :param found_line_params: parameters of detected lines
        :return: detected corners of the container
        """
        container_corners = retrieve_container_corners(found_line_params)
        self.corner_pub.publish(container_corners)
        return container_corners

    def odom_callback(self, odom: Odometry):
        """
        Is called whenever new odometry data arrives.

        :param odom: odometry data to update the robot pos with
        """
        pose = PoseStamped()
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        pose_stamped = transform_pose(TF_BUFFER, pose, 'base_link')
        self.robot_pos = pose_stamped.pose.position


def get_theta_by_index(idx: int) -> int:
    """
    Returns the theta value corresponding to the specified index (based on discretization).

    :param idx: index to retrieve theta value for
    :return: theta value for specified index
    """
    return idx * config.DELTA_THETA + config.THETA_MIN


def get_radius_by_index(idx: int) -> float:
    """
    Returns the radius value corresponding to the specified index (based on discretization).

    :param idx: index to retrieve radius value for
    :return: radius value for specified index
    """
    return idx * config.DELTA_RADIUS + config.RADIUS_MIN


def compute_y_coordinate(theta: int, radius: float, x: float) -> float:
    """
    Computes the y-coordinate of a given point based on the x-coordinate, theta, and radius.

    :param theta: theta value
    :param radius: radius value
    :param x: x-coordinate
    :return: y-coordinate
    """
    return (radius - np.cos(theta * np.pi / 180.0) * x) / np.sin(theta * np.pi / 180.0)


def retrieve_best_line(hough_space: np.array) -> (int, int):
    """
    Retrieves the indices of the best line (peak) from the specified hough space.

    :param hough_space: Hough space to retrieve peak from
    :return: indices of peak in hough space
    """
    max_idx = hough_space.argmax()
    c, r = np.unravel_index(max_idx, hough_space.shape)
    assert hough_space.max() == hough_space[c][r]
    return c, r


def compute_line_points(theta: int, radius: float) -> (Point, Point):
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


def median_filter(points_on_line: np.array) -> list:
    """
    Filters the points on the specified line based on their distances to the median.

    :param points_on_line: list of points on a detected line
    :return: filtered list of points
    """
    median = Point()
    median.x = np.median(np.sort([p.x for p in points_on_line]))
    median.y = np.median(np.sort([p.y for p in points_on_line]))
    res = [p for p in points_on_line if dist(p, median) <= config.CONTAINER_LENGTH]
    if len(res) > config.LINE_POINT_LIMIT:
        filtered = []
        for i in range(config.WINDOW_SIZE, len(res), config.WINDOW_SIZE):
            avg = Point()
            avg.x = np.average([res[j].x for j in range(i - config.WINDOW_SIZE, i)])
            avg.y = np.average([res[j].y for j in range(i - config.WINDOW_SIZE, i)])
            filtered.append(avg)
        res = filtered
    return res


def compute_avg_point(point_list: list) -> Point:
    """
    Computes the average of the specified list of points.

    :param point_list: list of points to compute average for
    :return: avg point
    """
    avg = Point()
    if len(point_list) > 0:
        avg.x = np.average([p.x for p in point_list])
        avg.y = np.average([p.y for p in point_list])
    return avg


def compute_avg_and_max_distance(point_list: list) -> (float, float):
    """
    Computes the average and maximum distance between different points in the specified list.

    :param point_list: list of points to compute avg and max dist for
    :return: average distance, max distance
    """
    distances = [dist(i, j) for i in point_list for j in point_list if i != j]
    if len(distances) > 0:
        return np.average(distances), np.max(distances)
    return 0.0, 0.0


def reasonable_dist_to_already_detected_lines(point_list: list, avg_points: list) -> bool:
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
        # the LB is quite dangerous here -> could lead to missed (covered) sides - but also kinda necessary
        if dist(avg, p) > eps_length or dist(avg, p) < config.CONTAINER_WIDTH / 4:
            return False
    return True


def detect_jumps(points_on_line: list) -> bool:
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
        if x_dist > config.JUMP_DIST or y_dist > config.JUMP_DIST:
            return True
        if x_dist > config.MINOR_JUMP_DIST or y_dist > config.MINOR_JUMP_DIST:
            jump_cnt += 1
    return jump_cnt >= config.JUMP_LIMIT


def detected_reasonable_line(point_list: list, theta_base: int, theta: int, avg_points: list) -> bool:
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
    # should be either parallel or orthogonal to base
    orthogonal_or_parallel_to_base = diff <= 2 or 88 <= diff <= 92 or 178 <= diff <= 182 or 268 <= diff <= 272
    avg_dist, max_dist = compute_avg_and_max_distance(point_list)
    reasonable_dist = reasonable_dist_to_already_detected_lines(point_list, avg_points)
    tolerated_upper_bound = config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON * 2

    # shouldn't be too restrictive at the lower bound (partially detected lines etc.)
    # however, for the base line it should
    if theta_base is None:
        tolerated_lower_bound = config.CONTAINER_WIDTH - config.CONTAINER_WIDTH * config.STRICT_EPSILON
    else:
        tolerated_lower_bound = config.LENGTH_LB
    reasonable_len = tolerated_upper_bound >= max_dist >= tolerated_lower_bound

    # TODO: avg distances perhaps not so useful - for now not considered
    reasonable_avg_dist = True  # config.CONTAINER_WIDTH / 2 >= avg_dist >= 0.5

    # for the base line there should be no jumps
    jumps = detect_jumps(point_list) if theta_base is None else False

    if config.VERBOSE_LOGGING:
        rospy.loginfo("reasonable len: %s, len: %s", reasonable_len, max_dist)
        rospy.loginfo("reasonable dist: %s", reasonable_dist)
        rospy.loginfo("orthogonal or parallel to base: %s, diff: %s", orthogonal_or_parallel_to_base, diff)
        rospy.loginfo("jumps: %s", jumps)

    return reasonable_len and reasonable_dist and reasonable_avg_dist and orthogonal_or_parallel_to_base and not jumps


def compute_intersection_points(found_line_params: list) -> list:
    """
    Computes intersection points for the specified lines.

    :param found_line_params: parameters of the lines to compute intersections for
    :return: list of intersection points
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


def update_avg_points(avg_points: list, point_list: list):
    """
    Saves the average point for each line.

    :param avg_points: list of average points
    :param point_list: list of points representing currently detected line (to be added to the list)
    """
    avg = Point()
    avg.x = np.average([p.x for p in point_list])
    avg.y = np.average([p.y for p in point_list])
    avg_points.append(avg)


def already_found_line(found_lines: list, radius: float, theta: int) -> bool:
    """
    Checks whether the line (or a very similar line) represented by radius and theta was found before.

    :param found_lines: parameters of already found lines
    :param radius: radius value of currently considered line
    :param theta: theta value of currently considered line
    :return: whether the line was already found
    """
    for r, t in found_lines:
        diff_r = abs(abs(radius) - abs(r))
        diff_t = abs(abs(theta) - abs(t))
        # if a line was found at a similar radius, it should be rotated by roughly 90 degrees
        if diff_r < config.DELTA_RADIUS_CHECK * config.RADIUS_FACTOR_FOUND_LINES \
                and diff_t < 90 - config.ORIENTATION_TOLERANCE_FOUND_LINES:
            return True
    return False


def already_tested_line(tested_lines: list, radius: float, theta: int) -> bool:
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
        # unlike the `already_found_line` method, here we only compare to already tested lines such that a similar
        # radius should imply a different rotation, but not necessarily one by ~90 degrees
        if diff_r < config.DELTA_RADIUS and diff_t < config.ORIENTATION_DIFF_CONS_LINES:
            return True
    return False


def container_front_or_back_detected(length: float) -> bool:
    """
    Returns whether the specified length corresponds to the container front / back.

    :param length: length to be compared to the container dimensions
    :return: whether the length corresponds to the container front / back
    """
    return length - config.CONTAINER_WIDTH * config.EPSILON <= config.CONTAINER_WIDTH \
           <= length + config.CONTAINER_WIDTH * config.EPSILON


def intersection(line_one: (float, float), line_two: (float, float)) -> (float, float):
    """
    Computes the intersection between the specified lines.

    :param line_one: line one
    :param line_two: line two
    :return: x- and y-coordinate of the intersection point (or None if not intersecting)
    """
    rho1, theta1 = line_one
    rho2, theta2 = line_two
    # parallel lines don't intersect
    if -2 <= abs(theta1 - theta2) <= 2 or 178 <= abs(theta1 - theta2) <= 182:
        return

    # to radians
    theta1 = theta1 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    theta_matrix = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    rho_vector = np.array([[rho1], [rho2]])
    x, y = np.linalg.solve(theta_matrix, rho_vector)
    return x, y


def feasible_distances(detected: list, theta: int, radius: float) -> bool:
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

            if d < (config.CONTAINER_WIDTH - config.CONTAINER_WIDTH * config.EPSILON) or \
                    d > (config.CONTAINER_LENGTH + config.CONTAINER_LENGTH * config.EPSILON):
                return False
    return True


def feasible_angles(detected: list, theta: int) -> bool:
    """
    There can only be two lines with roughly the same angle belonging to the container. This fact is used in order
    to determine whether the newly considered line could still be part of the container candidate.

    :param detected: parameters of previously detected lines
    :param theta: theta value of the detected line
    :return: whether the newly detected line could still be part of the container candidate
    """
    same_angle_lines = 0
    for _, t in detected:
        if abs(theta - t) <= config.ORIENTATION_TOLERANCE_FOUND_LINES:
            same_angle_lines += 1
    # already >= 2 lines with roughly the same angle -> not part of the container
    if same_angle_lines >= 2:
        return False
    return True


def feasible_orientation(detected: list, radius: float, theta: int) -> bool:
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


def feasible_intersections(detected: list, radius: float, theta: int) -> bool:
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

    # only feasible options
    if not len(intersections) in [0, 1, 2, 4]:
        if config.VERBOSE_LOGGING:
            rospy.loginfo("INFEASIBLE INTERSECTION NUMBER..: %s", len(intersections))
        return False

    # no distances to be checked
    if len(intersections) == 1 or len(intersections) == 0:
        return True

    width_eps = config.CONTAINER_WIDTH * config.STRICT_EPSILON
    length_eps = config.CONTAINER_LENGTH * config.STRICT_EPSILON

    if len(intersections) == 2:
        d = dist(intersections[0], intersections[1])
        tolerated_width = config.CONTAINER_WIDTH - width_eps < d < config.CONTAINER_WIDTH + width_eps
        tolerated_length = config.CONTAINER_LENGTH - length_eps < d < config.CONTAINER_LENGTH + length_eps

        if not tolerated_width and not tolerated_length:
            if config.VERBOSE_LOGGING:
                rospy.loginfo("dist: %s", d)
                rospy.loginfo("tolerated width: %s, tolerated length: %s", tolerated_width, tolerated_length)
                rospy.loginfo("INFEASIBLE WIDTH / LENGTH OF THE TWO INTERSECTIONS")
            return False

    if len(intersections) == 4:
        for p in intersections:
            len_cnt = wid_cnt = 0
            for j in intersections:
                if p != j:
                    d = dist(p, j)
                    tolerated_width = config.CONTAINER_WIDTH - width_eps < d < config.CONTAINER_WIDTH + width_eps
                    tolerated_length = config.CONTAINER_LENGTH - length_eps < d < config.CONTAINER_LENGTH + length_eps

                    if tolerated_length:
                        len_cnt += 1
                    elif tolerated_width:
                        wid_cnt += 1

            if len_cnt != 1 or wid_cnt != 1:
                if config.VERBOSE_LOGGING:
                    rospy.loginfo("4 INTERSECTIONS, BUT CNT INCORRECT..")
                return False
    return True


def feasible_line_length(line_lengths: list, curr_length: float) -> bool:
    """
    Determines whether the length of the currently considered container side candidate corresponds to
    the already detected ones and could reasonably represent a side of the container.

    The precise determination of whether the combination of detected sides makes sense, i.e. 2x long, 2x short,
    is performed in a later step based on intersections, this is just a rough estimation that allows partially
    detected lines.

    :param line_lengths: lengths of already detected lines
    :param curr_length: length of currently considered line candidate
    :return: whether length of currently considered line feasibly corresponds to already detected lines
    """
    length_eps = config.CONTAINER_LENGTH * config.STRICT_EPSILON
    width_eps = config.CONTAINER_WIDTH * config.STRICT_EPSILON

    # can't be too strict here due to partially detected lines
    if config.LENGTH_LB < curr_length < config.CONTAINER_WIDTH + width_eps:
        # not considering short sides as they could also be partially detected long sides
        pass
    elif config.CONTAINER_WIDTH + width_eps < curr_length < config.CONTAINER_LENGTH + length_eps:
        # check whether we have at most one long side detected yet
        cnt = 0
        for length in line_lengths:
            if config.CONTAINER_WIDTH + width_eps < length < config.CONTAINER_LENGTH + length_eps:
                cnt += 1
        if cnt > 1:
            return False
    else:
        return False
    return True


def line_corresponds_to_already_detected_lines(theta: int, detected: list, radius: float) -> bool:
    """
    Determines whether the newly detected line corresponds to the previously detected lines in the sense that the
    combination of them satisfies the shape criteria of the container.

    :param theta: theta value of the detected line
    :param detected: parameters of previously detected lines
    :param radius: radius value of the detected line
    :return: whether the newly detected line corresponds to the previously detected ones
    """
    if config.VERBOSE_LOGGING:
        rospy.loginfo("###############################################################")
        rospy.loginfo("feasible dist: %s", feasible_distances(detected, theta, radius))
        rospy.loginfo("feasible angles: %s", feasible_angles(detected, theta))
        rospy.loginfo("feasible orientation: %s", feasible_orientation(detected, radius, theta))
        rospy.loginfo("feasible intersections: %s", feasible_intersections(detected, radius, theta))
        rospy.loginfo("###############################################################")

    return feasible_distances(detected, theta, radius) and feasible_angles(detected, theta) and feasible_orientation(
        detected, radius, theta) and feasible_intersections(detected, radius, theta)


def retrieve_container_corners(found_line_params: list) -> list:
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


def determine_thresh_based_on_dist_to_robot(dist_to_robot: float) -> int:
    """
    Determines a dynamic threshold for the accumulator array based on the robot's distance to the considered shape.

    :param dist_to_robot: distance of the shape (line candidate) to the robot
    :return: dynamic accumulator threshold
    """
    # TODO: could be refined based on experiments
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


def get_points_from_scan(scan: LaserScan) -> list:
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
    Node to detect the container shape in a laser scan.
    """
    global TF_BUFFER
    rospy.init_node("detect_container")
    TF_BUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TF_BUFFER)
    DetectionServer()
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
