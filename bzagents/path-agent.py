#!/usr/bin/python -tt

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

#################################################################
# NOTE TO STUDENTS
# This is a starting point for you.  You will need to greatly
# modify this code if you want to do anything useful.  But this
# should help you to know how to interact with BZRC in order to
# get the information you need.
#
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#
# Often this translates to something like the following (with the
# port name being printed out by the bzrflag server):
# python agent0.py localhost 49857
################################################################

import sys
import math
import time
import operator

import uuid

from collections import defaultdict
from bzrc import BZRC, Command


current_id = 0

def generate_id():
    return uuid.uuid4()

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.x == other.x and self.y == other.y
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.x.__hash__() + self.y.__hash__()

    def __repr__(self):
        return "<Point x: %d y: %d>" % (self.x, self.y)

    def __str__(self):
        return self.__repr__()

class Node:
    def __init__(self, point):
        self.point = point
        self.neighbors = set()
        self.cost = 0
        self.id = generate_id()

    def addNeighbor(self, node):
        self.neighbors.add(node)

    def removeNeighbor(self, node):
        self.neighbors.remove(node)

    def removeIntersectingEdges(self, obstacles):
        # given four vertices, an edge intersects with an obstacle if the edge intersects with any edge that doesn't include the endpoint
        # print "Obstacles: %r"
        points = []
        obstacle_points = []
        for obstacle in obstacles:
            obstacle_points.append([])
            for corner in obstacle:
                points.append(Point(corner[0], corner[1]))
                obstacle_points[-1].append(points[-1])

        #construct edges to use to check for intersections - make sure to only include edges within obstacles, through
        edges = set()

        for obstacle in obstacle_points:
            for start_point in obstacle:
                for end_point in obstacle:
                    if start_point != end_point:
                        edges.add(Edge(start_point, end_point))

        # print edges
        # print self.neighbors

        nodes_to_remove = set()
        for end_point in self.neighbors:
            for edge in edges:
                if closed_segment_intersect((self.point.x, self.point.y), (end_point.point.x, end_point.point.y), (edge.point_a.x, edge.point_a.y), (edge.point_b.x, edge.point_b.y)):
                    nodes_to_remove.add(end_point)
                    break

        for node in nodes_to_remove:
            self.neighbors.remove(node)

        # print "Final neighbors:"
        # print self.neighbors


        # WORKING CODE ENDS HERE

        # # for start_point in points:
        # #     for end_point in points:
        # #         if start_point != end_point:
        # #             edges[start_point].append(end_point)
        #
        # for endpoint in points:
        #     # the edge is self.point to endpoint
        #     # get the other edges to check it against
        #     for other_start_point in edges.keys():
        #         if other_start_point == endpoint:
        #             # print "Skipping line from %r to %r" % (other_start_point, endpoint)
        #             continue
        #         for other_end_point in edges[other_start_point]:
        #             if other_end_point == endpoint:
        #                 # print "Skipping line from %r to %r" % (other_end_point, endpoint)
        #                 continue
        #             # and finally, we can do the comparison to the original lines
        #             if intersect(self.point, endpoint, other_start_point, other_end_point):
        #                 # print "An intersecting line segment was found and removed from %r to %r" % (self.point, endpoint)
        #                 # remove the offending endpoint from the set of neighbors
        #                 if endpoint in self.neighbors:
        #                     self.neighbors.remove(endpoint)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.point == other.point   # TODO - we may potentially want to care about the neighbors set
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.point.__hash__()

    def __repr__(self):
        return "<Node id: %d point: %r cost: %d>" % (self.id, self.point, self.cost)

    def __str__(self):
        return self.__repr__()

class Edge:
    def __init__(self, point_a, point_b):
        self.point_a = point_a
        self.point_b = point_b

    def __hash__(self):
        return Point(self.point_a.x + self.point_b.x, self.point_a.y + self.point_b.y).__hash__()

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (self.point_a == other.point_a and self.point_b == other.point_b) or (self.point_a == other.point_b and self.point_b == other.point_a)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq(other)

    def __repr__(self):
        return "<Edge point_a: %r, point_b: %r>" % (self.point_a, self.point_b)

    def __str__(self):
        return self.__repr__()

def side(a,b,c):
    """ Returns a position of the point c relative to the line going through a and b
        Points a, b are expected to be different
    """
    d = (c[1]-a[1])*(b[0]-a[0]) - (b[1]-a[1])*(c[0]-a[0])
    return 1 if d > 0 else (-1 if d < 0 else 0)

def is_point_in_closed_segment(a, b, c):
    """ Returns True if c is inside closed segment, False otherwise.
        a, b, c are expected to be collinear
    """
    if a[0] < b[0]:
        return a[0] < c[0] and c[0] < b[0]
    if b[0] < a[0]:
        return b[0] < c[0] and c[0] < a[0]

    if a[1] < b[1]:
        return a[1] < c[1] and c[1] < b[1]
    if b[1] < a[1]:
        return b[1] < c[1] and c[1] < a[1]

    return a[0] == c[0] and a[1] == c[1]

#
def closed_segment_intersect(a,b,c,d):
    """ Verifies if closed segments a, b, c, d do intersect."""
    # print a[0], a[1], b[0], b[1], c[0], c[1], d[0], d[1]
    debug = False
    if b[0] == 15 and b[1] == 9 and c[0] == 15 and c[1] == 8:
        debug = True
    else:
        debug = False
    # print debug

    if a == b:
        if debug:
            print "DEBUG A == B"
        return False
    if c == d:
        if debug:
            print "DEBUG C == D"
        return False

    s1 = side(a,b,c)
    s2 = side(a,b,d)

    # All points are collinear
    if s1 == 0 and s2 == 0:
        if debug:
            print "DEBUG s1 == 0 and s2 == 0"
        return \
            is_point_in_closed_segment(a, b, c) or is_point_in_closed_segment(a, b, d) or \
            is_point_in_closed_segment(c, d, a) or is_point_in_closed_segment(c, d, b)

    if s1 == 0 or s2 == 0:
        if debug:
            print "DEBUG s1 == 0 or s2 == 0"
        return not ((a[0] == c[0] and a[1] == c[1]) or (a[0] == d[0] and a[1] == d[1]) or (b[0] == c[0] and b[1] == c[1]) or (b[0] == d[0] and b[1] == d[1]))

    # No touching and on the same side
    if s1 and s1 == s2:
        if debug:
            print "DEBUG s1 and s1 == s2"
        return False

    s1 = side(c,d,a)
    s2 = side(c,d,b)

    # No touching and on the same side
    if s1 and s1 == s2:
        if debug:
            print "DEBUG second s1 and s1 == s2"
        return False

    if debug:
        print "DEBUG default"
    return True

def crappity_graphity(start, end, obstacles, algorithm="a*"):
    points = [start]

    for obstacle in obstacles:
        for corner in obstacle:
            points.append(Point(corner[0], corner[1]))

    points.append(end)

    nodes = {}

    for point in points:
        node = Node(point)
        nodes[point] = node

    for point in points:
        for point2 in points:
            if point == point2:
                continue
            else:
                nodes[point].addNeighbor(nodes[point2])

    # nodes = []
    # for point1 in points:
    #     node = Node(point)
    #     nodes.append(node)
    #     for point2 in points:
    #         if point1 == point2:
    #             continue
    #         else:
    #             node.addNeighbor(Node(point2))

    for point in nodes.keys():
        nodes[point].removeIntersectingEdges(obstacles)
        print "\tNode %r has neighbors %r" % (nodes[point], nodes[point].neighbors)

    if algorithm == "a*":
        points_to_visit = aStarSearch(nodes[start], nodes[end])
    elif algorithm == "dfs":
        points_to_visit, cost = depth_first_search(nodes[start], nodes[end])
        print "DFS Path: %r" % (points_to_visit)
    else:
        points_to_visit, cost = breadth_first_search(nodes[start], nodes[end])
    print "Returning from crappity_graphity:"
    print points_to_visit
    return points_to_visit

class BSSF:
    def __init__(self):
        self.value = float("inf")

    def update_value(self, new_value):
        if new_value < self.value:
            self.value = new_value
            return True
        return False

    def is_bssf(self, value_to_check):
        return value_to_check == self.value


def depth_first_search(start, goal, visited_nodes=set(), bssf=BSSF(), cost_so_far=0):
    has_unvisited_neighbor = False
    visited_nodes.add(start)

    if start == goal:
        bssf.update_value(cost_so_far)
        return [start], cost_so_far

    bssf_path = []
    for neighbor in start.neighbors:
        if neighbor not in visited_nodes:
            has_unvisited_neighbor = True
            path, cost = depth_first_search(neighbor, goal, visited_nodes, bssf, cost_so_far + dist_between(start, neighbor))
            if bssf.is_bssf(cost):
                bssf_path = [start] + path

    if not has_unvisited_neighbor:
        return [], float("inf")
    else:
        return bssf_path, bssf.value

def breadth_first_search(start, goal):
    visited = set()
    queue = [start]
    solutions = []

    if not start.neighbors and start != goal:
        return [], float("inf") #error

    while queue:
        current = queue.pop(0)
        visited.add(current)
        if current == goal:
            # do something, we're done searching but we need to see if there's a better solution
            pass
        else:
            for neighbor in current.neighbors:
                if neighbor not in visited:
                    neighbor.cost = current.cost + dist_between(current, neighbor)
                    queue.append(neighbor)
    return [], float("inf")

def dist_between(node1, node2):
    return math.sqrt((node1.point.x - node2.point.x)**2 + (node1.point.y - node2.point.y)**2)

def heuristic_cost_estimate(start, goal):
    return dist_between(start, goal)

def aStarSearch (start,goal):
    closedset = set()    # The set of nodes already evaluated.
    openset = set()    # The set of tentative nodes to be evaluated, initially containing the start node
    openset.add(start)
    came_from = {}    # The map of navigated nodes.

    g_score = defaultdict(lambda: float("inf")) #map with default value of Infinity
    g_score[start.id] = 0    # Cost from start along best known path.
    # Estimated total cost from start to goal through y.
    f_score = defaultdict(lambda: float("inf"))
    f_score[start.id] = heuristic_cost_estimate(start, goal)

    while openset: #this means there are nodes inthe openset still
        current = sorted(openset, key = lambda x: f_score[x.id])[0] #TODO - make this sort correctly
        print "A* Visiting Node: %r" % current
        if current == goal:
            print "A* goal state reached"
            print "\tCame from: %r" % came_from
            return reconstruct_path(came_from, goal)

        #remove current from openset
        openset.remove(current)
        #add current to closedset
        closedset.add(current)
        #expanding the horizon here
        print "Neighbors: %r" % current.neighbors
        for neighbor in current.neighbors:
            print "Considering %r" % neighbor
            if neighbor in closedset:
                continue

            tentative_g_score = g_score[current.id] + dist_between(current,neighbor)
            # print "\tTentative g score is %f" % tentative_g_score

            if tentative_g_score < g_score[neighbor.id]:
                came_from[neighbor.id] = current
                g_score[neighbor.id] = tentative_g_score
                f_score[neighbor.id] = g_score[neighbor.id] + heuristic_cost_estimate(neighbor, goal)
                if neighbor not in openset:
                    # add neighbor to openset
                    openset.add(neighbor)
    return []

#TODO does this work out of the box? It just might...
def reconstruct_path(came_from,current):
    total_path = [current]
    while current.id in came_from:
        print "\tRetracing: %r" % total_path
        current = came_from[current.id]
        total_path.append(current)
    print "Retraced: %r" % total_path
    return total_path[::-1]

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.paths = {}

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.obstacles = self.bzrc.get_obstacles()
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []

        for tank in self.mytanks:
            if (not tank.index in self.paths) or (not self.paths[tank.index]):
                starting_point = Point(tank.x, tank.y)

                # find the closest enemy flag for the goal point
                min_distance = float("inf")
                for flag in self.flags:
                    if self.constants["team"] == flag.color:
                        continue

                    if math.sqrt((tank.x - flag.x)**2 + (tank.y - flag.y)**2) < min_distance:
                        min_distance = math.sqrt((tank.x - flag.x)**2 + (tank.y - flag.y)**2)
                        closest_flag = flag
                goal_point = Point(closest_flag.x, closest_flag.y)
                self.paths[tank.index] = crappity_graphity(starting_point, goal_point, self.obstacles, "dfs")

            # check to see if we're within a threshold distance of the next point
            if math.sqrt((tank.x - self.paths[tank.index][0].point.x)**2 + (tank.y - self.paths[tank.index][0].point.y)**2) < 25:
                # print "Tank %d has arrived at one point and is moving on to the next point in its path." % tank.index
                self.paths[tank.index].pop(0)

            # go to the next point on the path
            if self.paths[tank.index]:
                # print "Tank %d moving to position %r from (%d, %d)" % (tank.index, self.paths[tank.index][0].point, tank.x, tank.y)
                self.move_to_position(tank, self.paths[tank.index][0].point.x, self.paths[tank.index][0].point.y)
            break
        results = self.bzrc.do_commands(self.commands)

    def move_to_position(self, tank, target_x, target_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(target_y - tank.y,
                                  target_x - tank.x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, 1, 2 * relative_angle, True)
        self.commands.append(command)

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int (angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc)

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    # start = Node(Point(4, 0))
    # a = Node(Point(2, 5))
    # b = Node(Point(6, 5))
    # c = Node(Point(6, 7))
    # d = Node(Point(2, 7))
    #
    # e = Node(Point(3, 8))
    # f = Node(Point(15, 8))
    # g = Node(Point(15, 9))
    # h = Node(Point(3, 9))
    # end = Node(Point(4,10))
    #
    # start.addNeighbor(a)
    # start.addNeighbor(b)
    # start.addNeighbor(c)
    # start.addNeighbor(d)
    # start.addNeighbor(e)
    # start.addNeighbor(f)
    # start.addNeighbor(g)
    # start.addNeighbor(h)
    # start.addNeighbor(end)
    #
    # obstacles = [[(2,5), (6,5), (6,7), (2,7)], [(3,8), (15,8), (15,9), (3,9)]]
    #
    # start.removeIntersectingEdges(obstacles)

    main()


# vim: et sw=4 sts=4
