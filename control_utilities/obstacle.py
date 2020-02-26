from random import randint

class Obstacle:
    def __init__(self, p1, p2, width=3, length=4):
        self.p1 = p1
        self.p2 = p2

        self.width = width
        self.length = length

class RandomObstacleGenerator:
    @staticmethod
    def generateObstacles(path, n=1, dist=10, seed=0):
        obstacles = []
        for _ in range(n):
            i = randint(0, len(path))
            if i+dist < len(path):
                obstacles.append((i, Obstacle(path.points[i], path.points[i+dist])))
            else:
                obstacles.append((i, Obstacle(path.points[i], path.points[i+dist - len(path)])))
        return dict(obstacles)
