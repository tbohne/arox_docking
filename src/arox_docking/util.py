#!/usr/bin/env python
import math

def dist(p1, p2):
    """
    Computes the Euclidean distance between the specified points.

    :param p1: point one
    :param p2: point two
    :return: Euclidean distance
    """
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)