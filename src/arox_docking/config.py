#!/usr/bin/env python

########################## CONTAINER ##########################
CONTAINER_WIDTH = 2.83
CONTAINER_LENGTH = 3.7
EPSILON = 0.2
###############################################################

#################### CONTAINER DETECTION ######################
# --- hough transform ---
DELTA_THETA = 2
THETA_MIN = 0
THETA_MAX = 180
DELTA_RADIUS = 0.02
# TODO: check sensor range
RADIUS_MIN = -10.0
RADIUS_MAX = 10.0
# --- detection procedure ---
TIME_LIMIT = 60  # in seconds
CHECK_FREQUENCY = 3  # in seconds
DETECTION_ATTEMPTS = 10
BASE_TEST_LIMIT = 500
BASE_ATTEMPT_LIMIT = 1000
DEFAULT_DIST = 10  # in meters
###############################################################

####################### MBF ERROR CODES #######################
MBF_FAILURE = 50
MBF_PAT_EXCEEDED = 103
###############################################################

