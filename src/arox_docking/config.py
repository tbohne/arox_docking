#!/usr/bin/env python

############################ META #############################
VERBOSE_LOGGING = False
###############################################################

########################## CONTAINER ##########################
CONTAINER_WIDTH = 2.55
CONTAINER_LENGTH = 3.7
EPSILON = 0.2

CHARGING_STATION_POS_X = 1.25
CHARGING_STATION_POS_Y = 0.85
###############################################################

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
###############################################################

####################### MBF ERROR CODES #######################
MBF_FAILURE = 50
MBF_PAT_EXCEEDED = 103
###############################################################

