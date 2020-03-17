"""

This module creates segmentation to represent track by less param.

Copyright (C) 2020 Hankel Bao <hankelbao@outlook.com>

"""
import numpy as np
import matplotlib.pyplot as plt

from control_utilities.track import Track
from control_utilities.path import Path


class Segmentations:
    """Segmentations model the track by representative segmentations regardless of num_points

    It is done by ensure that the sk_percision and max_distance within two segmentations
    would not exceed given number.
    sk_percision = curvature * distance.
    """
    def __init__(self, track, percision=0.500, max_distance=15):
        """Init of segmentations

        Note:
        1. It is generally ok to stay with the default param.
        2. max_distance is to ensure that segmentations have info for a extremely long, striaght part.
        3. If you don't want to create additional segmentation by distance, set max_distance to np.inf.
           However, it might lead to path outside of track!

        Args:
            percision: cumulative limit for curvature * distance
            max_distance: max cumulative distance for a new segmentation
        """
        self.left = []
        self.right = []
        self.width = []
        self.track = track
        self.size = 0
        self.precision = percision
        self.max_distance = max_distance

    def create_segmentations(self):
        """Create the segmentations"""
        sk_sum = 0.0
        cum_distance = 0.0

        right_points = []
        left_points = []
        width = []
        for i in range(self.track.center.length):
            distance = 0 if i == 0 else self.track.center.ps[i-1]
            cum_distance += distance
            k = np.abs(self.track.center.k[i])
            sk_sum += distance * k

            # The first segmentation wouldn't be recorded until condition is meet,
            # so we need to ensure the last point is recorded.
            if sk_sum >= self.precision or \
                    cum_distance >= self.max_distance or \
                    i == self.track.center.length-1:
                sk_sum = 0
                cum_distance = 0

                left = self.track.left_waypoints[i]
                right = self.track.right_waypoints[i]
                left_points.append(left)
                right_points.append(right)

                width.append(
                    np.sqrt(np.square(left[0]-right[0])+np.square(left[1]-right[1])))
                self.size += 1

        self.left = Path(left_points, closed=True, raw_mode=True)
        self.right = Path(right_points, closed=True, raw_mode=True)
        self.width = np.array(width)

    def get_point(self, index, a):
        """Get a point by relative distance in segmentation

        Args:
            index (int): index of segmentation
            a (float): the relative offset from left
        """
        x1 = self.left.x[index]
        y1 = self.left.y[index]
        x2 = self.right.x[index]
        y2 = self.right.y[index]
        x = x1 + (x2-x1)/self.width[index]*a
        y = y1 + (y2-y1)/self.width[index]*a
        return [x, y]

    def get_points(self, a):
        """Get all points by a

        Note:
            a must have same size as self.size

        Args:
            a (array): array of offset distance to left

        Returns:
            points (array)
        """
        points = []
        for i in range(self.size):
            points.append(self.get_point(i, a[i]))
        return points

    def plot(self):
        """Plot segmentations"""
        for i in range(self.size):
            plt.plot([self.left.x[i], self.right.x[i]], [
                     self.left.y[i], self.right.y[i]], "g-")
