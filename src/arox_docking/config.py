#!/usr/bin/env python

############################ META #############################
VERBOSE_LOGGING = False
TEST_RUNS = 10
###############################################################

########################## CONTAINER ##########################
CONTAINER_WIDTH = 2.55
CONTAINER_LENGTH = 3.7

CHARGING_STATION_POS_X = 1.25
CHARGING_STATION_POS_Y = 0.85
###############################################################

DIST_TO_CONTAINER = 3.5

EXTERNAL_POINT_DIST = 1.5

#################### CONTAINER DETECTION ######################
# --- hough transform ---
DELTA_THETA = 2
THETA_MIN = 0
THETA_MAX = 180
DELTA_RADIUS = 0.05
DELTA_RADIUS_CHECK = 0.02
# TODO: check sensor range
RADIUS_MIN = -10.0
RADIUS_MAX = 10.0
# --- detection procedure ---
TIME_LIMIT = 60  # in seconds
CHECK_FREQUENCY = 3  # in seconds
DETECTION_ATTEMPTS = 5
BASE_TEST_LIMIT = 500
BASE_ATTEMPT_LIMIT = 1000
DEFAULT_DIST = 10  # in meters
LINE_LB = 5  # lower bound of points forming a line

LINE_POINT_LIMIT = 300
WINDOW_SIZE = 5

EPSILON = 0.2
STRICT_EPSILON = 0.05
LENGTH_LB = 1.0

JUMP_LIMIT = 3
JUMP_DIST = 2
MINOR_JUMP_DIST = 0.7

RADIUS_FACTOR_FOUND_LINES = 8
ORIENTATION_TOLERANCE_FOUND_LINES = 10
ORIENTATION_DIFF_CONS_LINES = 30
###############################################################

####################### MBF ERROR CODES #######################
MBF_FAILURE = 50
MBF_PAT_EXCEEDED = 103
###############################################################

