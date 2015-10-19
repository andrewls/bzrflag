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
#################################################################

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
        points = []
        obstacle_points = []
        for obstacle in obstacles:
            obstacle_points.append([])
            for corner in obstacle:
                points.append(Point(corner[0], corner[1]))
                obstacle_points[-1].append(points[-1])

        #construct edges to use to check for intersections - make sure to only include edges within obstacles, through
        edges = defaultdict(list)
        for obstacle in obstacle_points:
            for start_point in obstacle:
                for end_point in obstacle:
                    if start_point != end_point:
                        edges[start_point].append(end_point)
        print obstacles
        print edges

        # for start_point in points:
        #     for end_point in points:
        #         if start_point != end_point:
        #             edges[start_point].append(end_point)


        #debug
        goal_point = Point(0,370)
        #/debug

        for endpoint in points:
            # the edge is self.point to endpoint
            # get the other edges to check it against
            if end_point == goal_point:
                debug = True
            else:
                debug = False
            for other_start_point in edges.keys():
                if other_start_point == endpoint:
                    if debug:
                        print "Skipping line from %r to %r" % (other_start_point, endpoint)
                    continue
                for other_end_point in edges[other_start_point]:
                    if other_end_point == endpoint:
                        if debug:
                            print "Skipping line from %r to %r" % (other_end_point, endpoint)
                        continue
                    # and finally, we can do the comparison to the original lines
                    if intersect(self.point, endpoint, other_start_point, other_end_point):
                        if debug:
                            print "An intersecting line segment was found and removed from %r to %r" % (self.point, endpoint)
                        # remove the offending endpoint from the set of neighbors
                        if endpoint in self.neighbors:
                            self.neighbors.remove(endpoint)

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


def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def crappity_graphity(start, end, obstacles, algorithm="a*"):
    points = [start]

    for obstacle in obstacles:
        for corner in obstacle:
            points.append(Point(corner[0], corner[1]))

    points.append(end)

    nodes = []
    for point in points:
        node = Node(point)
        nodes.append(node)
        for point2 in points:
            if point == point2:
                continue
            else:
                node.addNeighbor(Node(point2))

    for node in nodes:
        node.removeIntersectingEdges(obstacles)
    print "After removing intersecting edges, start node has neighbors %r" % nodes[0].neighbors
    if algorithm == "a*":
        points_to_visit = aStarSearch(nodes[0], nodes[-1])
    elif algorithm == "dfs":
        points_to_visit = depth_first_search(nodes[0], nodes[-1])
    else:
        points_to_visit = breadth_first_search(nodes[0], nodes[-1])
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
            return reconstruct_path(came_from, goal)

        #remove current from openset
        openset.remove(current)
        #add current to closedset
        closedset.add(current)
        #expanding the horizon here
        for neighbor in current.neighbors:
            print "Considering %r" % neighbor
            if neighbor in closedset:
                continue

            tentative_g_score = g_score[current.id] + dist_between(current,neighbor)
            print "\tTentative g score is %f" % tentative_g_score

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
    while current in came_from:
        current = came_from[current.id]
        total_path.append(current)
    return total_path

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

        print "Ticking"
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
                self.paths[tank.index] = crappity_graphity(starting_point, goal_point, self.obstacles, "a*")

            # check to see if we're within a threshold distance of the next point
            if math.sqrt((tank.x - self.paths[tank.index][0].point.x)**2 + (tank.y - self.paths[tank.index][0].point.y)**2) < 10:
                print "Tank %d has arrived at one point and is moving on to the next point in its path." % tank.index
                self.paths[tank.index].pop(0)

            # go to the next point on the path
            if self.paths[tank.index]:
                print "Tank %d moving to position %r from (%d, %d)" % (tank.index, self.paths[tank.index][0].point, tank.x, tank.y)
                self.move_to_position(tank, self.paths[tank.index][0].point.x, self.paths[tank.index][0].point.y)
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
    main()

# vim: et sw=4 sts=4
