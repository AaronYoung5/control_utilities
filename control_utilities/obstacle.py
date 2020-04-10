from random import randint
import numpy as np

class Obstacle:
    def __init__(self, p1, p2, i, dist, num, width=1, length=2, movement_rate=-1, movement_distance=0, ch_time=0.0, num_movements=0):
        self.p1 = p1
        self.p2 = p2
        self.i = i
        self.dist = dist

        self.width = width
        self.length = length

        self.movement_rate = movement_rate
        self.ch_time = ch_time

        self.movement_distance = movement_distance
        self.num_movements = num_movements

        self.num = num

    def Update(self, step):
        if self.movement_rate == -1:
            print('Please set the movement_rate')
            return
        self.ch_time += step
        if self.ch_time*self.movement_rate + 1e-9 > self.num_movements:
            self.i += self.movement_distance
            self.num_movements += 1
            return True
        else:
            return False

    def copy(self, p1, p2, new_i):
        return Obstacle(self.p1, self.p2, new_i, self.dist, self.num, self.width, self.length, self.movement_rate, self.ch_time)

class RandomObstacleGenerator:
    @staticmethod
    def generateObstacles(path, i_min, i_max, n=1, width=3, length=4, dist=10, seed=0, reversed=0, movement_rate=-1, movement_distance=0):
        if i_min > i_max:
            raise Exception("i_min must be less than i_max")
        elif i_max > len(path):
            print("i_max must be less than the length of the path\nsetting i_max to len(path)")
            i_max = len(path)-1
        obstacles = []
        for num in range(n):
            i = randint(i_min, i_max)
            if i+dist < len(path):
                p1 = path.points[i]
                p2 = path.points[i+dist]
            else:
                p1 = path.points[i]
                p2 = path.points[i+dist - len(path)]
            obstacles.append((i, Obstacle(p1, p2, i=i, dist=dist, num=num, width=width, length=length, movement_rate=movement_rate, movement_distance=movement_distance)))
        return dict(obstacles)

    @staticmethod
    def moveObstacle(path, obstacles, obstacle, i_old):
        if obstacle.i > len(path)-1:
            obstacle.i = obstacle.i - len(path)
        i = obstacle.i
        if i+obstacle.dist < len(path):
            p1 = path.points[i]
            p2 = path.points[i+obstacle.dist]
        else:
            p1 = path.points[i]
            p2 = path.points[i+obstacle.dist - len(path)]
        obstacles[i] = Obstacle(p1, p2, i, obstacle.dist, num=obstacle.num, width=obstacle.width, length=obstacle.length, movement_rate=obstacle.movement_rate,movement_distance=obstacle.movement_distance, ch_time=obstacle.ch_time, num_movements=obstacle.num_movements)
        del obstacles[i_old]
        return obstacles

# Evaluates obstacle's boundary box dimensions x, y, z
def getObstacleBoundaryDim(mesh):
    verts = mesh.getCoordsVertices()
    x = []
    y = []
    z = []
    for vert in verts:
        x.append(vert.x)
        y.append(vert.y)
        z.append(vert.z)
    x = np.asarray(x)
    y = np.asarray(y)
    z = np.asarray(z)
    return np.amax(x) - np.amin(x), np.amax(y) - np.amin(y), np.amax(z) - np.amin(z)