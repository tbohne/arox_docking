#!/usr/bin/env python

########################## CONTAINER ##########################
CONTAINER_WIDTH = 2.83
CONTAINER_LENGTH = 3.7
EPSILON = 0.2
###############################################################

###################### HOUGH TRANSFORM ########################
DELTA_THETA = 2
THETA_MIN = 0
THETA_MAX = 180
# should be sufficiently precise to identify reasonable lines
DELTA_RADIUS = 0.02
# TODO: check sensor range
RADIUS_MIN = -10.0
RADIUS_MAX = 10.0
###############################################################

####################### MBF ERROR CODES #######################
MBF_FAILURE = 50
MFB_PAT_EXCEEDED = 103
###############################################################

