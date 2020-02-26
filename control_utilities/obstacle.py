from random import randint

class Obstacle:
    def __init__(self, p1, p2, width=3, length=4):
        self.p1 = p1
        self.p2 = p2

        self.width = width
        self.length = length

class RandomObstacleGenerator:
    @staticmethod
    def generateObstacles(path, i_min, i_max, n=1, dist=10, seed=0, reversed=0):
        if i_min > i_max:
            raise Exception("i_min must be less than i_max")
        elif i_max > len(path):
            raise Exception("i_max must be less than the length of the path")
        obstacles = []
        for _ in range(n):
            i = randint(i_min, i_max)
            if i+dist < len(path):
                p1 = path.points[i]
                p2 = path.points[i+dist]
            else:
                p1 = path.points[i]
                p2 = path.points[i+dist - len(path)]
            obstacles.append((i, Obstacle(p1, p2)))
        return dict(obstacles)
