from typing import List
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
from dataclasses import dataclass, field


@dataclass(init=True, repr=True)
class Point:
    """
    Point class is datastructure to handle a point
    """

    x: float = 0
    y: float = 0

    def __hash__(self):
        return hash((self.x, self.y))

    def __lt__(self, val):
        return True


@dataclass(init=True, repr=True)
class Edge:
    """
    Point class is datastructure to handle a vertex
    """

    p: Point
    q: Point


@dataclass(init=True, repr=True)
class OrderedPoints:
    ordered_points: List[Point] = field(default_factory=lambda: [])
    polygon: bool = False


def draw_world(entities: List[OrderedPoints], VG):
    def getPlot(ordered_points: List[Point], polygon: bool, color="black", lw=3):
        verts = [(p.x, p.y) for p in ordered_points]
        codes: int = [Path.MOVETO] + [Path.LINETO for _ in verts[1:]]

        if polygon:
            codes += [Path.CLOSEPOLY]
            verts += [verts[0]]

        # print(codes, verts)

        path = Path(verts, codes)

        patch = patches.PathPatch(path, facecolor="none", edgecolor=color, lw=lw)

        return patch

    plt.axes()

    for e in entities:
        p = getPlot(e.ordered_points, e.polygon)
        plt.gca().add_patch(p)

    for k, v in VG.items():
        for l in v:
            points = [k, l]
            p = getPlot(points, False, "red", 1)
            plt.gca().add_patch(p)

    plt.axis("scaled")
    plt.savefig("foo.png", bbox_inches="tight")


def doIntersect(e1: Edge, e2: Edge) -> bool:

    p1: Point = e1.p
    q1: Point = e1.q
    p2: Point = e2.p
    q2: Point = e2.q
    """
    The main function that returns true if line segment 'p1q1' 
    and 'p2q2' intersect. 
    """

    def onSegment(p: Point, q: Point, r: Point) -> bool:
        """
        Given three colinear points p, q, r, the function checks if
        point q lies on line segment 'pr
        """
        if (
            q.x <= max(p.x, r.x)
            and q.x >= min(p.x, r.x)
            and q.y <= max(p.y, r.y)
            and q.y >= min(p.y, r.y)
        ):
            return True

        return False

    def orientation(p: Point, q: Point, r: Point) -> int:
        """
        To find orientation of ordered triplet (p, q, r).
        The function returns following values
        0 --> p, q and r are colinear
        1 --> Clockwise
        2 --> Counterclockwise
        See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        for details of below formula.
        """
        val = round((float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)))

        if val > 0:  # Clockwise orientation
            return 1
        elif val < 0:  # Counterclockwise orientation
            return -1
        else:  # Collinear orientation
            return 0

    # Find the four orientations needed for general and
    # special cases
    o1: int = orientation(p1, q1, p2)
    o2: int = orientation(p1, q1, q2)
    o3: int = orientation(p2, q2, p1)
    o4: int = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and onSegment(p1, p2, q1):
        return True

    # p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and onSegment(p1, q2, q1):
        return True

    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and onSegment(p2, p1, q2):
        return True

    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and onSegment(p2, q1, q2):
        return True

    return False  # Doesn't fall in any of the above cases


def intersection_test():
    # Driver program to test above functions:
    p1 = Point(1, 1)
    q1 = Point(10, 1)
    p2 = Point(1, 2)
    q2 = Point(10, 2)

    e1, e2 = Edge(p1, q1), Edge(p2, q2)
    if doIntersect(e1, e2):
        print("Yes")
    else:
        print("No")

    p1 = Point(10, 0)
    q1 = Point(0, 10)
    p2 = Point(0, 0)
    q2 = Point(10, 10)

    e1, e2 = Edge(p1, q1), Edge(p2, q2)
    if doIntersect(e1, e2):
        print("Yes")
    else:
        print("No")

    p1 = Point(-5, -5)
    q1 = Point(0, 0)
    p2 = Point(1, 1)
    q2 = Point(10, 10)

    e1, e2 = Edge(p1, q1), Edge(p2, q2)
    if doIntersect(e1, e2):
        print("Yes")
    else:
        print("No")

    p1 = Point(5, 5)
    q1 = Point(0, 0)
    p2 = Point(0, 0)
    q2 = Point(0, 5)

    e1, e2 = Edge(p1, q1), Edge(p2, q2)
    if doIntersect(e1, e2):
        print("Yes")
    else:
        print("No")
