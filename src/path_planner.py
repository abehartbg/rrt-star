from __future__ import annotations  # necessary for self referential type in <python3.10
from dataclasses import dataclass, field
from typing import List, Dict, Set
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
from collections import defaultdict
import math
import heapq

from util import Point, Edge, OrderedPoints, draw_world

from shapely.geometry.polygon import Polygon, LineString


class PathPlanner:
    def __init__(self, waypoints: OrderedPoints, obstacles: List[OrderedPoints]):
        self._obstacles = obstacles
        self._obstacle_polygons = [
            Polygon([(o.x, o.y) for o in obstacle.ordered_points])
            for obstacle in obstacles
        ]

    def make_visibility_graph(self, start: Point, end: Point):
        """
        E -> all edges of obstacles
        G -> all verticies of obstacles

        naive approach
            for v in G
                for w in G-v
                    for e in E
                        if v->w doesn't intersect e
                            add to list
        """

        # def same_end(pe, e):
        #     return pe.p == e.p or pe.p == e.q or pe.q == e.p or pe.q == e.q

        # def intersect_obstacle(obstacle, edge):
        #     ctr = 0
        #     s = obstacle.ordered_points[0]
        #     for m in obstacle.ordered_points[1:]:
        #         e = Edge(s, m)
        #         if doIntersect(edge, e) and not same_end(edge, e):
        #             ctr += 1

        #         s = m
        #     e = Edge(s, obstacle.ordered_points[0])
        #     if doIntersect(edge, e) and not same_end(edge, e):
        #         ctr += 1

        #     print(ctr)
        #     if ctr > 0:
        #         return True

        #     return False

        G: List[Point] = [start, end]
        E: List[Edge] = []

        # Create G and E lists
        for o in self._obstacles:
            s = o.ordered_points[0]
            G.append(s)
            for m in o.ordered_points[1:]:
                G.append(m)
                e = Edge(s, m)
                E.append(e)
                s = m
            e = Edge(s, o.ordered_points[0])
            E.append(e)

        self._VG = defaultdict(set)

        # G <-- all vertices of S
        # VG <-- empty visibility graph
        # for each vertex v in G                                            #O(n)
        #     for each vertex w in {G - v}                                  #O(n)
        #         for each edge e in S                                      #O(e)
        #             if the arc from v to w does not intersect any edge e then
        #             vertex v and w are visible to each other
        #                 VG <-- edge v to w
        for v in G:
            for w in G:
                if v == w:
                    continue
                pe = Edge(v, w)

                c = True

                c = self.clears_obstacles(pe)

                # for o in self._obstacles:
                #     if intersect_obstacle(o, pe):
                #         c=False
                #         break

                # for e in E:
                #     if doIntersect(pe, e) and not same_end(pe, e):
                #         c = False
                #         break

                if c:
                    self._VG[v].add(w)

    def clears_obstacles(self, vertex: Edge):

        # polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        # path = LineString([(0.0, 0.0), (1, 1)])

        path = LineString([(vertex.p.x, vertex.p.y), (vertex.q.x, vertex.q.y)])
        # print(path)

        for polygon in self._obstacle_polygons:
            if path.intersects(polygon) and not path.touches(polygon):
                return False

        return True

    def make_path(self):
        pass


def A_star(graph: Dict[Point, Set[Point]], start: Point, end: Point):
    """
    takes in a graph and a start and end node
    finds the shortest path from start to end

    """
    # // A* finds a path from start to goal.
    # function A_Star(start, goal, h)

    def h(node: Point):
        # // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
        # hurisitic function
        # euclidian distance from node to end
        # we can skip the squareroot
        return (node.x + end.x) ** 2 + (node.y + end.y) ** 2

    def d(current: Point, neighbor: Point):
        # cost function
        # in this case cost is just distance between two points
        return (current.x + neighbor.x) ** 2 + (current.y + neighbor.y) ** 2

    def reconstruct_path(cameFrom, current):
        # function reconstruct_path(cameFrom, current)
        #     total_path := {current}
        #     while current in cameFrom.Keys:
        #         current := cameFrom[current]
        #         total_path.prepend(current)
        #     return total_path
        total_path = [current]

        while current in cameFrom.keys():
            current = cameFrom[current]
            total_path.append(current)

        return total_path[::-1]

    # // The set of discovered nodes that may need to be (re-)expanded.
    # // Initially, only the start node is known.
    # // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    # openSet := {start}
    openSet = [(0.0, start)]

    #     // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    #     // to n currently known.
    #     cameFrom := an empty map
    cameFrom = {}

    #     // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    #     gScore := map with default value of Infinity
    #     gScore[start] := 0
    gScore = defaultdict(lambda: float("inf"))
    gScore[start] = 0

    #     // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    #     // how cheap a path could be from start to finish if it goes through n.
    #     fScore := map with default value of Infinity
    #     fScore[start] := h(start)
    fScore = defaultdict(lambda: float("inf"))
    fScore[start] = h(start)

    # while openSet is not empty
    #     // This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
    #     current := the node in openSet having the lowest fScore[] value
    #     if current = goal
    #         return reconstruct_path(cameFrom, current)

    #     openSet.Remove(current)
    #     for each neighbor of current
    #         // d(current,neighbor) is the weight of the edge from current to neighbor
    #         // tentative_gScore is the distance from start to the neighbor through current
    #         tentative_gScore := gScore[current] + d(current, neighbor)
    #         if tentative_gScore < gScore[neighbor]
    #             // This path to neighbor is better than any previous one. Record it!
    #             cameFrom[neighbor] := current
    #             gScore[neighbor] := tentative_gScore
    #             fScore[neighbor] := tentative_gScore + h(neighbor)
    #             if neighbor not in openSet
    #                 openSet.add(neighbor)

    # // Open set is empty but goal was never reached
    # return failure

    while openSet:
        _, current = heapq.heappop(openSet)

        if current == end:
            return reconstruct_path(cameFrom, current)

        for neighbor in graph[current]:
            tentative_gScore = gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + h(neighbor)
                if neighbor not in openSet:
                    heapq.heappush(openSet, (fScore[neighbor], neighbor))

    return None


def __main__():
    waypoints: OrderedPoints = OrderedPoints([Point(1, 1), Point(10, 10)])
    obstacle1: OrderedPoints = OrderedPoints(
        [Point(5, 5), Point(5, 6), Point(6, 6), Point(6, 5)], True
    )

    obstacle2: OrderedPoints = OrderedPoints(
        [Point(9, 6), Point(10, 8), Point(10, 10), Point(8, 10)], True
    )

    p = PathPlanner(waypoints, [obstacle1, obstacle2])

    start = Point(0, 0)
    end = Point(13, 8)

    p.make_visibility_graph(start, end)

    path = A_star(p._VG, start, end)

    print("Best path:", path)

    linepath = OrderedPoints(path, False)

    # for k, v in p._VG.items():
    #     print(k, v)

    # v = Edge(Point(0, 0), Point(10, 10))
    # print(p.clears_obstacles(v))

    draw_world([linepath, obstacle1, obstacle2], p._VG)

    # print("hello world")

    # intersection_test()


def A_star_test():
    pass


__main__()
