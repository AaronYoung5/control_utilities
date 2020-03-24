import random
import math
import sys
import numpy as np

from scipy.interpolate import splprep, splev
from matplotlib import pyplot as plt
import scipy
import warnings

import pychrono as chrono

class Path():

    """
    Path class that generates a spline through the provided points. Spline is defined by a discrete number of points.

    ...

    Attributes
    ----------
    x, y : 1D lists
        lists that hold the x and y values of the path at each point
    dx, dy : 1D lists
        lists that hold the dx and dy values of the path at each point
    ddx, ddy : 1D lists
        lists that hold the ddx and ddy values of the path at each point
    k, s : 1D lists
        lists that hold the curvature and distance along the path at each point
    points : vector_ChVectorD
        points passed into the class but as ChVectorD's
    last_index : int
        index of the most recent calculate index
        see calcIndex()
    last_dist : float
        progression along the path from the most recent calculation
        see calcIndex()
    track_length : float
        variable defined as the length of the track approximated as the linear distance between each point
    times_looped : int
        times the track has been looped. For progression purposes
    length : int
        number of points defined by the spline approximation

    Methods
    -------
    curvature(dx, dy, ddx, ddy)
        Computes curvature at points given the first an second derivative
    distance(x, y)
        Compute the distance between the given points
    calcIndex(pos, n=10)
        Calculates the index of the closest point on the path given a position
    calcClosestPoint(pos)
        Calculates the location of the closest point on the path given a position
    getPoint(i)
        Gets the point on the path given an index
    calcDistance(pos)
        Determines the distance progressed along the path
    getDistance(i)
        Gets the distance on the path given an index
    calcCurvature(pos)
        Calculates the curvature at a given point on the path given a position
    getCurvature(i)
        Gets the curvature on the path given an index
    calcPosition(s)
        Calculates the position on the path given a distance along the path
    plot(color, show=True)
        plots the path using matplotlib

    Static Methods
    --------------
    calcPose(p1, p2, reversed=False):
        calculates pose (position and orientation) at a point along the path

    """
    def __init__(self, points, num_points=1000, closed=True, raw_mode=False, z=0.0, brake=1.0, acc=1.0, s=0.0):
        self.u_s = .25
        self.g = 9.81
        self.speed_max = 100

        self.waypoints = points
        points = np.array(points)
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=RuntimeWarning)
            tck, u = splprep(points.T, s=s, per=closed)

        if raw_mode:
            u_new = u
        else:
            u_new = np.linspace(u.min(), u.max(), num_points)

        self.x, self.y = splev(u_new, tck, der=0)
        self.dx, self.dy = splev(u_new, tck, der=1)
        self.ddx, self.ddy = splev(u_new, tck, der=2)
        self.length = len(self.x)
        self.k = self.curvature(self.dx, self.dy, self.ddx, self.ddy)
        self.ps = self.point_distance(self.x, self.y)
        self.s = self.distance(self.x, self.y)
        self.yaw = self.yaw(self.dx, self.dy)
        self.v_max = self.speed(self.x, self.y, self.k)

        self.points = []
        for x,y in zip(self.x, self.y):
            self.points.append(chrono.ChVectorD(x,y,z))

        self.last_index = 0
        self.last_dist = 0
        self.track_length = self.s[-1]
        self.times_looped = 0

        self.brake = brake
        self.acc = acc

    def update_vmax(self):
        for i in range(self.length):
            v_max = self.v_max[i]

            s = 0
            for j in range(1, i+1):
                # Check v_max of all points before it
                index = i-j
                s += self.ps[index]
                local_v_max = np.sqrt(2*self.brake*s + np.square(v_max))
                if local_v_max < self.v_max[index]:
                    self.v_max[index] = local_v_max

    def update_profile(self):
        v = []
        t = []

        current_speed = 0
        for i in range(len(self.ps)):
            v.append(current_speed)

            ps = self.ps[i]
            v_max = self.v_max[i+1]

            if current_speed <= v_max:
                top_speed = np.sqrt(2*self.acc*ps + current_speed**2)
                v_end = v_max if top_speed > v_max else top_speed
                acc_distance = (v_end**2 - current_speed**2) / (2 * self.acc)
                time = (top_speed-current_speed)/self.acc + (ps-acc_distance)/v_end

                t.append(time)
                current_speed = v_end
            else:
                v_end = v_max
                brk_distance = (current_speed**2 - v_end**2) / (2 * self.brake)
                time = (ps-brk_distance)/current_speed + (current_speed-v_end)/self.brake
                t.append(time)

                current_speed = v_end

        self.v = np.array(v)
        self.t = np.array(t)

    def plot_speed_profile(self):
        # Speed
        plt.plot(self.s, self.v, "g-")

        # Time
        plt.plot(self.s, self.t, "y-")

        # Curvature
        # plt.plot(np.abs(self.k*100), "b-")

        # Max Speed based on global curvature
        # plt.plot(np.abs(self.v_max), "k-")

        # Throttle
        # plt.plot(np.array(self.adjust_a)*10+5, "c+")

        plt.show()

    def plot_path_speed(self):
        # last_v = 0

        for i in range(self.length-1):
            plt.plot(self.x[i], self.y[i], c=str(self.v[i]/10/1.3), marker="o")
            # if self.v[i] >= last_v:
            #     plt.plot(self.x[i], self.y[i], "g+")
            # else:
            #     plt.plot(self.x[i], self.y[i], "r+")
            # last_v = self.v[i]

    def curvature(self, dx, dy, ddx, ddy):
        """
        Compute curvature at one point given first and second derivatives.
        """
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)

    def point_distance(self, x, y):
        """
        Computer distance between points
        """
        return np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)

    def distance(self, x, y):

        """
        Compute distance from beginning given two points.
        """
        return np.cumsum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))

    def yaw(self, dx, dy):
        """
        Compute yaw along the given path
        """
        yaw = []
        for i in range(len(dx)):
            yaw.append(math.atan2(dy[i], dx[i]))
        return yaw

    def speed(self, x, y, k):
        """
        Compute speed profile for the given path
        """
        k = np.true_divide(1, k)
        speed = []
        for kk in k:
            speed.append(math.sqrt(abs(kk * self.u_s * self.g)))
        speed = np.clip(speed, 0, self.speed_max)
        return speed


    def calcIndex(self, pos, n=20):
        """
        Calculates the index of the closest point on the path
        """
        besti = self.last_index
        bestd = self.point_distance([pos.x, self.x[self.last_index]],[pos.y, self.y[self.last_index]])
        for i in range(max(0, self.last_index-n), self.last_index+n):
            if i >= self.length-1:
                ii = i-self.length
            else:
                ii = i
            temp = self.point_distance([pos.x, self.x[ii]],[pos.y, self.y[ii]])
            if temp <= bestd:
                bestd = temp
                besti = ii

        self.last_index = besti
        if self.last_dist > self.s[besti]:
            self.times_looped += 1
        self.last_dist = self.s[besti]
        return besti

    def calcClosestPoint(self, pos, n=20):
        """
        Determines closest point on the path from a pos
        """
        i = self.calcIndex(pos, n=n)
        return self.points[i]

    def getPoint(self, i):
        """
        Gets the point on the path given an index
        """
        return self.points[i]

    def calcDistance(self, pos, n=20):
        """
        Determines the distance progressed along the path
        """
        i = self.calcIndex(pos, n=n)
        return self.s[i] + self.track_length * self.times_looped

    def getDistance(self, i):
        """
        Gets the distance on the path given an index
        """
        return self.s[i]

    def calcCurvature(self, pos, n=20):
        """
        Determines the curvature at the closest point along the path
        """
        i = self.calcIndex(pos, n=n)
        return self.k[i]

    def getCurvature(self, i):
        """
        Gets curvature on the path given an index
        """
        return self.k[i]

    def calcSpeed(self, pos, n=20):
        """
        Determines the speed at the closest point along the path
        """
        i = self.calcIndex(pos, n=n)
        return self.v[i]

    def getSpeed(self, i):
        """
        Gets speed on the path given an index
        """
        return self.v[i]

    def calcPosition(self, s):
        """
        Determines the position and curvature based off the current progression along the path
        """
        array = np.asarray(self.s)
        i = (np.abs(array-s)).argmin()

        return self.points[i], self.k[i]

    def calcPositionFromPoint(self, pos, dist):
        """
        Determines the position based off a position and a distance from that position
        """
        i = self.calcIndex(pos)
        s = self.s[i] + dist
        return calcPosition(s)

    def setIndex(self, i):
        self.last_index = i

    def plot(self, color, show=True):
        """Plots path using matplotlib

        Parameters
        ----------
        color : str
            color formating str for matplotlib
        show : bool, optional
            if plot.show() be called
        """
        import matplotlib.pyplot as plt
        plt.axis('equal')

        plt.plot(self.x, self.y, color)
        # plt.scatter(self.x,self.y, c=self.v)

        if show:
            plt.show()

    def __len__(self):
        return len(self.x)

class RandomPathGenerator():
    """
    RandomPathGenerator class that generates a random path using convex hulls

    Attributes
    ----------
    x_max : int
        maximum x value for the randomly generated path
    y_max : int
        maximum y value for the randomly generated path
    num_points : int
        number of points randomly generated at the very beginning
    min_distance : int
        minimum distance between points when randomly generated
    min_angle : int
        minimum angle between three points before a correction heuristic is performed
    hull : vector_ChVectorD
        the convex hull that is used to generate the random path

    Methods
    -------
    generatePath(seed=1.0, reversed=0)
        generate a random track using a convex hull
    generatePoints(z=0.0)
        generates a set of random points
    calcConvexHull(points)
        calculates the convex hull that around the random points generated
    calcAngle(v1, v2)
        calculates the angle between two vectors

    """
    def __init__(self, x_max, y_max, num_points=25, min_distance=17, min_angle=120):
        """
        Parameters
        ----------
        x_max : ChBezierCurve
            maximum x value for the randomly generated path
        y_max : RandomPathGenerator
            maximum y value for the randomly generated path
        num_points : int
            number of points randomly generated at the very beginning
        min_distance : int
            minimum distance between points when randomly generated
        min_angle : int
            minimum angle between three points before a correction heuristic is performed

        """
        self.x_max = x_max
        self.y_max = y_max
        self.num_points = num_points
        self.min_distance = min_distance
        self.min_angle = min_angle

    def generatePath(self, seed=1.0, reversed=0):
        """Caller method that generates a random path through other method calls

        Parameters
        ----------
        seed : int, optional
            random seed to be used in pythons pseudo random functions
            (See: https://docs.python.org/3/library/random.html)
        reversed : int, optional
            used to reverse the direction the path is created

        Returns
        -------
        self.hull : vector_ChVectorD
            the random generated path in the form of a vector of points
        """
        random.seed(seed)
        self.hull = self.calcConvexHull(self.generatePoints())
        self.hull.insert(0, self.hull[-1])
        if reversed:
            self.hull.reverse()
        return self.hull

    def generatePoints(self, z=0.0):
        """Generates a set of random points

        Parameters
        ----------
        z : int, optional
            the height that each point should be off the terrain

        Return
        ------
        points : [ChVectorD]
            the random ChVectorD points generated by the function in a python list

        """
        def checkDistances(points, point):
            """Checks if the distance is greater than the minimum distance"""
            for p in points:
                if math.hypot(*(np.array(p)-np.array(point))) < self.min_distance:
                    return False
            return True

        points = []
        for i in range(self.num_points):
            x = (random.random() - 0.5) * self.x_max
            y = (random.random() - 0.5) * self.y_max
            point = [x,y]
            if checkDistances(points, point):
                points.append(point)

        return np.array(points)

    def calcConvexHull(self, points):
        """Calculates the convex hull that around the random points generated

        Parameters
        ----------
        points : [ChVectorD]
            array of ChVectorD's that are used to generate a convex hull around

        Returns
        -------
        [ChVectorD]
            the convex hull
        """

        points = sorted(points, key=lambda item : item[0])

        upper = points[0:2]		# Initialize upper part

        # Compute the upper part of the hull
        for i in range(2, len(points)):
            upper.append(points[i])
            while len(upper) > 2 and self.calcAngle(upper[-2] - upper[-1], upper[-2] - upper[-3]) > 0:
                del upper[-2]

        lower = list(reversed(points[-2:]))  # Initialize the lower part
        # Compute the lower part of the hull
        for i in range(len(points) - 3, -1, -1):
            lower.append(points[i])
            while len(lower) > 2 and self.calcAngle(lower[-2] - lower[-1], lower[-2] - lower[-3]) > 0:
                del lower[-2]
        del lower[0]
        del lower[-1]
        return upper + lower		# Build the full hull

    def calcAngle(self, v1, v2):
        """Calculates the angle between two vectors

        Parameters
        ----------
        v1 : ChVectorD
            vector 1
        v2 : ChVectorD
            vector 2

        Returns
        -------
        float
            angle between v1 and v2 in degrees
        """
        return math.degrees(math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1]))
