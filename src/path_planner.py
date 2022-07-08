from dataclasses import dataclass
import math 


@dataclass(init=True, repr=True)
class Point():
    x:float = 0
    y:float = 0

    def distance(self, point):
        return math.sqrt((self.x - point.x)**2 + (self.y - point.y)**2)

@dataclass(init=True, repr=True)
class Obstacle():
    pass

class PathPlanner():
    def __init__(self):
        pass


def __test__():

    waypoints = [Point(0,0), Point(0,10),Point(10,10)]

    p = PathPlanner(waypoints)

    print("hello world")


__test__()