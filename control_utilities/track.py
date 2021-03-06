from control_utilities.path import Path, RandomPathGenerator
import numpy as np
import sys

import pychrono as chrono

class Track:
    """
    Track class that has a center, left and right path

    ...

    Attributes
    ----------
    center : Path
        the path object that describes the centerline
    right : Path
        the path object that describes the right boundary
    left : Path
        the path object that describes the left boundary
    width : int
        constant width of the track
    num_points : int
        number of points to interpolate along path

    Methods
    -------
    generateTrack()
        generates the left and right boundaries from the centerline
    plot(show=True)
        plots the track using matplotlib

    """
    def __init__(self, center, width=10, num_points=1000, closed=True, raw_mode=False, s=0.0, x_min=None, x_max=None, y_min=None, y_max=None):
        """
        Parameters
        ----------
        center : ChBezierCurve
            the centerline
        width : int, optional
            constant distance from the centerline to each boundary
        num_points : int, optional
            num points to interpolate along path
        """
        self.raw_mode = raw_mode
        self.closed = closed
        self.center = Path(center, num_points, closed=closed, raw_mode=raw_mode, s=s)
        self.width = width
        self.num_points = num_points

        if x_max == None:
            self.x_max = max(self.center.x) + self.width
        else:
            self.x_max = x_max
        if y_max == None:
            self.y_max = max(self.center.y) + self.width
        else:
            self.y_max = y_max

        if x_min == None:
            self.x_min = min(self.center.x) - self.width
        else:
            self.x_max = x_min
        if y_max == None:
            self.y_min = min(self.center.y) - self.width
        else:
            self.y_min = y_min

    def generateTrack(self, z=0.0):
        """Generates the left and right boundaries from the centerline"""
        left, right = [], []

        for i in range(self.center.length-1):
            ix, iy = self.center.x[i], self.center.y[i]
            dx, dy = self.center.dx[i], self.center.dy[i]
            length = np.linalg.norm(np.array([dx,dy]))
            dx = dx * self.width / (2*length)
            dy = dy * self.width / (2*length)
            left.append([ix-dy, iy+dx])
            right.append([ix+dy, iy-dx])

        if self.closed:
            if left[0] != left[-1] and right[0] != right[-1]:
                left.append(left[0])
                right.append(right[0])

        self.left = Path(left, self.num_points, closed=self.closed, raw_mode=self.raw_mode)
        self.right = Path(right, self.num_points, closed=self.closed, raw_mode=self.raw_mode)

        self.left_waypoints = left
        self.right_waypoints = right

    def checkBoundary(self, pos, n=20):
        """Checks if current position is within boundaries or not"""
        track_pos = self.center.calcClosestPoint(pos, n=n)
        dist = (track_pos - pos).Length()
        return dist < (width / 2)

    def setBoundaries(self, left, right):
        self.left = left
        self.right = right

    @staticmethod
    def FromBoundaries(left, right, width=10, num_points=1000, closed=True, raw_mode=False):
        """ Generates Track from left and right Path's """
        center = []
        for i, lp in enumerate(left.points):
            rp = min(right.points, key = lambda p: (p-lp).Length())
            mid = [(rp.x + lp.x)/2, (rp.y + lp.y)/2]
            center.append(mid)
        track = Track(center, width=width, num_points=num_points, closed=closed, raw_mode=raw_mode)
        track.setBoundaries(left, right)
        return track

    def plot(self, show=True, centerline=True):
        """Plots track using matplotlib

        Parameters
        ----------
        show : bool, optional
            if plot.show() be called
        """
        import matplotlib.pyplot as plt
        plt.axis('equal')
        if centerline:
            self.center.plot(color='-r', show=False)
        self.right.plot(color='-k', show=False)
        self.left.plot(color='-k', show=False)
        if show:
            plt.show()

class RandomTrack(Track):
    """
    RandomTrack class that generates a Track object from a random centerline

    ...

    Attributes
    ----------
    generator : RandomPathGenerator
        generates a random path given a certain seed value
    x_max : int
        maximum x value for the randomly generated path
    y_max : int
        maximum y value for the randomly generated path
    width : int
        constant distance from the centerline to the outer boundaries

    Methods
    -------
    generateTrack()
        generate track from a random centerline
    plot(show=True)
        plots the track using matplotlib and adds sliders to interact with seed of random

    """
    def __init__(self, x_max=100, y_max=100, width=10):
        """
        Parameters
        ----------
        x_max : int, optional
            maximum x value for the randomly generated path
        y_max : int, optional
            maximum y value for the randomly generated path
        width : int, optional
            constant distance from the centerline to the outer boundaries
        """
        self.x_max = x_max
        self.y_max = y_max
        self.width = width
        self.generator = RandomPathGenerator(x_max=self.x_max, y_max=self.y_max)

    def generateTrack(self, seed=1.0, reversed=0, num_points=1000):
        """Generates Track object from new random centerline path

        Parameters
        ----------
        seed : int, optional
            random seed to be used in pythons pseudo random functions
            (See: https://docs.python.org/3/library/random.html)
        reversed : int, optional
            used to reverse the direction the path is created
        """
        self.points = self.generator.generatePath(seed=seed,reversed=reversed)
        Track.__init__(self, self.points, width=self.width, num_points=num_points, x_max=self.x_max, y_max=self.y_max)
        super(RandomTrack, self).generateTrack()

    def plot(self, seed=1.0, show=True):
        """Plots track using matplotlib and has interactive sliders

        Parameters
        ----------
        seed : int, optional
            seed to be used for random function
        show : bool, optional
            if plot.show() be called
        """
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        plot_ax = plt.axes([0.1, 0.2, 0.8, 0.75])
        seed_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
        seed_slider = Slider(
            seed_axes, "Seed", 0, 100, valinit=int(seed), valstep=1
        )
        plt.sca(plot_ax)

        def update(val):
            self.generateTrack(seed=val)
            plt.cla()
            super(RandomTrack, self).plot(show=False)

        update(seed)

        seed_slider.on_changed(update)
        if show:
            plt.show()
