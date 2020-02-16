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
    def __init__(self, center, width=5, num_points=1000):
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
        self.center = Path(center, num_points)
        self.width = width
        self.num_points = num_points

    def generateTrack(self, z=0.5):
        """Generates the left and right boundaries from the centerline"""
        left, right = [], []

        i = 0
        for t in self.center.s:
            ix, iy = self.center.x[i], self.center.y[i]
            dx, dy = self.center.dx[i], self.center.dy[i]
            len = np.linalg.norm(np.array([dx,dy]))
            dx, dy = dx / len, dy / len
            dx, dy = dx * self.width, dy * self.width
            left.append([ix-dy, iy+dx])
            right.append([ix+dy, iy-dx])
            i+=1

        if left[0] != left[-1] and right[0] != right[-1]:
            left.append(left[0])
            right.append(right[0])

        self.left = Path(left)
        self.right = Path(right)

    def plot(self, show=True):
        """Plots track using matplotlib

        Parameters
        ----------
        show : bool, optional
            if plot.show() be called
        """
        import matplotlib.pyplot as plt
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
    def __init__(self, x_max=100, y_max=100, width=5):
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

    def generateTrack(self, seed=1.0, reversed=0):
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
        Track.__init__(self, self.points)
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
