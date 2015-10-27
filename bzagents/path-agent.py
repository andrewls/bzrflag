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
import datetime
import operator

import uuid

from collections import defaultdict
from bzrc import BZRC, Command
from Queue import PriorityQueue

from matplotlib import pyplot as plot

current_id = 0
bssf_cost = float("inf")
bssf_path = []
current_path = []

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

        nodes_to_remove = set()
        for end_point in self.neighbors:
            for edge in edges:
                if closed_segment_intersect((self.point.x, self.point.y), (end_point.point.x, end_point.point.y), (edge.point_a.x, edge.point_a.y), (edge.point_b.x, edge.point_b.y)):
                    nodes_to_remove.add(end_point)
                    break
                elif (abs(self.point.x) == 400 and end_point.point.x == self.point.x) or (abs(self.point.y) == 400 and end_point.point.y == self.point.y):
                    nodes_to_remove.add(end_point)
                    break

        for node in nodes_to_remove:
            self.neighbors.remove(node)

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
    # if a[0] == 0 and a[1] == -140 and b[0] == 80 and b[1] == -100:
    if False:
        debug = True
        print "A: %r, B: %r, C: %r, D: %r" % (a, b, c, d)
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
        if ((a[0] == c[0] and a[1] == c[1] and b[0] == d[0] and b[1] == d[1]) or (a[0] == d[0] and a[1] == d[1] and b[0] == c[0] and b[1] == c[1])):
            return False
        return \
            is_point_in_closed_segment(a, b, c) or is_point_in_closed_segment(a, b, d) or \
            is_point_in_closed_segment(c, d, a) or is_point_in_closed_segment(c, d, b)

    if s1 == 0 or s2 == 0:
        if debug:
            print "DEBUG s1 == 0 or s2 == 0"
            print "s1 = %d, s2 = %d" % (s1, s2)
        if s1 == 0:
            return is_point_in_closed_segment(a, b, c)
            # print not ((a[0] == c[0] and a[1] == c[1]) or (b[0] == c[0] and b[1] == c[1]))
            # return not ((a[0] == c[0] and a[1] == c[1]) or (b[0] == c[0] and b[1] == c[1]))
        else:
            return is_point_in_closed_segment(a, b, d)
            # return not ((a[0] == d[0] and a[1] == d[1]) or (b[0] == d[0] and b[1] == d[1]))
        # return not ((a[0] == c[0] and a[1] == c[1]) or (a[0] == d[0] and a[1] == d[1]) or (b[0] == c[0] and b[1] == c[1]) or (b[0] == d[0] and b[1] == d[1]))

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
    global bssf_path
    points = [start]

    for obstacle in obstacles:
        obstacles_x = 0
        obstacles_y = 0
        for corner in obstacle:
            obstacles_x += corner[0]
            obstacles_y += corner[1]
        obstacle_center = (float(obstacles_x) / len(obstacle), float(obstacles_y) / len(obstacle))

        for corner in obstacle:
            x_coordinate, y_coordinate = corner
            if abs(corner[0]) != 400:
                position = side(obstacle_center, (obstacle_center[0], obstacle_center[1] + 1), corner)
                # print "x position of %r relative to %r is %d" % (corner, obstacle_center, position)
                if position > 0:
                    x_coordinate -= 4
                elif position < 0:
                    x_coordinate += 4
            if abs(corner[1]) != 400:
                position = side(obstacle_center, (obstacle_center[0] + 1, obstacle_center[1]), corner)
                # print "y position of %r relative to %r is %d" % (corner, obstacle_center, position)
                if position < 0:
                    y_coordinate -= 4
                elif position > 0:
                    y_coordinate += 4
            points.append(Point(x_coordinate, y_coordinate))

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

    for point in nodes.keys():
        nodes[point].removeIntersectingEdges(obstacles)


    ################### PLOTTING CODE
    # x_points = [-400,-400,400,400,-400]
    # y_points = [400,-400,-400,400,400]
    # plot.plot(x_points, y_points)
    ##################################

    if algorithm == "a*":
        points_to_visit = aStarSearch(nodes[start], nodes[end], obstacles)
    elif algorithm == "dfs":
        # points_to_visit = depth_first_search(graph, nodes[start], nodes[end])
        points_to_visit = depth_first_search(nodes[start], nodes[end], obstacles)
        # points_to_visit = bssf_path
        # print "DFS Path: %r" % (points_to_visit)
    else:
        # print "Running breadth first search"
        points_to_visit = breadth_first_search(nodes[start], nodes[end], obstacles)
    print "Returning from crappity_graphity:"
    print points_to_visit
    return points_to_visit


class BSSF:
    def __init__(self):
        self.value = float("inf")

    def update_value(self, new_value):
        print "UPDATING BSSF VALUE TO %f" % new_value
        if new_value < self.value:
            self.value = new_value
            return True
        return False

    def is_bssf(self, value_to_check):
        return value_to_check == self.value

bssf = BSSF()


def depth_first_search(start, goal, obstacles, visited_nodes=set(), cost_so_far=0):
    global bssf_cost
    global bssf_path
    global current_path

    x_points = [-400,-400,400,400,-400]
    y_points = [400,-400,-400,400,400]
    plot.plot(x_points, y_points)

    # graph the obstacles as well
    for obstacle in obstacles:
        x_points = []
        y_points = []
        for corner in obstacle:
            x_points.append(corner[0])
            y_points.append(corner[1])
        x_points.append(x_points[0])
        y_points.append(y_points[0])
        plot.plot(x_points, y_points, color="black")

    # plot the visited nodes
    x_points = []
    y_points = []
    for node in visited_nodes:
        x_points.append(node.point.x)
        y_points.append(node.point.y)
    plot.plot(x_points, y_points, 'bs')

    # the current frontier are the neighbors of the current node
    x_points = []
    y_points = []
    for node in start.neighbors:
        x_points.append(node.point.x)
        y_points.append(node.point.y)
    if x_points:
        plot.plot(x_points, y_points, 'r^')

    ts = time.time()
    time_string = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    # plot.savefig("dfs-%s.png" % time_string)
    #close the plot so that there's a new one on the next iteration
    plot.close()

    # print "Visiting node %r" % start
    visited_nodes.add(start)
    current_path.append(start)

    if start == goal:
        # print "Goal state reached. cost_so_far: %f, bssf_cost: %f" % (cost_so_far, bssf_cost)
        if cost_so_far < bssf_cost:
            bssf_path = current_path[:]
            # print "BSSF path updated: %r" % bssf_path
            bssf_cost = cost_so_far
            current_path.pop()
            return bssf_path

    for neighbor in start.neighbors:
        if neighbor not in visited_nodes:
            depth_first_search(neighbor, goal, obstacles, visited_nodes, cost_so_far + dist_between(start, neighbor))
    current_path.pop()
    return bssf_path

def breadth_first_search(start, goal, obstacles):
    global bssf_cost
    global bssf_path

    visited = set()
    queue = PriorityQueue()
    queue.put((0, start, [start]))

    while not queue.empty():
        current = queue.get()
        current_node, cost, current_path = current[1], current[0], current[2]
        if current_node in visited:
            continue


        x_points = [-400,-400,400,400,-400]
        y_points = [400,-400,-400,400,400]
        plot.plot(x_points, y_points)

        # graph the obstacles as well
        for obstacle in obstacles:
            x_points = []
            y_points = []
            for corner in obstacle:
                x_points.append(corner[0])
                y_points.append(corner[1])
            x_points.append(x_points[0])
            y_points.append(y_points[0])
            plot.plot(x_points, y_points, color="black")

        # plot the visited nodes
        x_points = []
        y_points = []
        for node in visited:
            x_points.append(node.point.x)
            y_points.append(node.point.y)
        plot.plot(x_points, y_points, 'bs')

        # the current frontier are the nodes still in the queue
        x_points = []
        y_points = []
        queue_copy = []
        # print "Entering queue loop"
        # print queue
        while queue.qsize():
            # print "Right before the get, length is %d" % queue.qsize()
            node = queue.get()
            # print "Got cost %r with node %r and path %r" % node
            x_points.append(node[1].point.x)
            y_points.append(node[1].point.y)
            queue_copy.append(node)
        # print "Done getting nodes"
        for node in queue_copy:
            queue.put(node)
        if x_points:
            plot.plot(x_points, y_points, 'r^')

        ts = time.time()
        time_string = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        # plot.savefig("bfs-%s.png" % time_string)
        #close the plot so that there's a new one on the next iteration
        plot.close()


        visited.add(current_node)
        if current_node == goal:
            if cost < bssf_cost:
                bssf_path = current_path[:]
        for neighbor in current_node.neighbors:
            if neighbor not in visited:
                queue.put((cost + dist_between(current_node, neighbor), neighbor, current_path[:] + [neighbor]))
    return bssf_path

def dist_between(node1, node2):
    return math.sqrt((node1.point.x - node2.point.x)**2 + (node1.point.y - node2.point.y)**2)

def heuristic_cost_estimate(start, goal):
    return dist_between(start, goal)

def aStarSearch (start, goal, obstacles):


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
        # print "Creating plot for a *"
        current = sorted(openset, key = lambda x: f_score[x.id])[0] #TODO - make this sort correctly

        # PLOT STUFF
        x_points = [-400,-400,400,400,-400]
        y_points = [400,-400,-400,400,400]
        plot.plot(x_points, y_points)

        # graph the obstacles as well
        for obstacle in obstacles:
            x_points = []
            y_points = []
            for corner in obstacle:
                x_points.append(corner[0])
                y_points.append(corner[1])
            x_points.append(x_points[0])
            y_points.append(y_points[0])
            plot.plot(x_points, y_points, color="black")

        #plot all of the visited nodes
        x_points = []
        y_points = []
        for node in closedset:
            x_points.append(node.point.x)
            y_points.append(node.point.y)
        plot.plot(x_points, y_points, 'bs')

        # also plot the search frontier
        x_points = []
        y_points = []
        for node in openset:
            x_points.append(node.point.x)
            y_points.append(node.point.y)
        plot.plot(x_points, y_points, 'r^')

        # get a timestamp so that the figures all have unique names and don't overwrite each other
        ts = time.time()
        time_string = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')





        if current == goal:
            return_path = reconstruct_path(came_from, goal)
            x_points = []
            y_points = []
            for node in return_path:
                x_points.append(node.point.x)
                y_points.append(node.point.y)
            plot.plot(x_points, y_points, color="blue")
            # plot.savefig("a-star-%s.png" % time_string)
            #close the plot so that there's a new one on the next iteration
            plot.close()
            return reconstruct_path(came_from, goal)


        # plot.savefig("a-star-%s.png" % time_string)
        #close the plot so that there's a new one on the next iteration
        plot.close()


        #remove current from openset
        openset.remove(current)
        #add current to closedset
        closedset.add(current)
        #expanding the horizon here
        for neighbor in current.neighbors:
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
    total_distance = 0
    while current.id in came_from:
        total_distance += dist_between(current, came_from[current.id])
        current = came_from[current.id]
        total_path.append(current)
    print "The total path cost was %f" % total_distance
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
                self.paths[tank.index] = crappity_graphity(starting_point, goal_point, self.obstacles, "a*")

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
    # start = Node(Point(2 - 400, 0))
    # a = Node(Point(0 - 400, 5))
    # b = Node(Point(6 - 400, 5))
    # c = Node(Point(6 - 400, 7))
    # d = Node(Point(0 - 400, 7))
    #
    # e = Node(Point(3 - 400, 8))
    # f = Node(Point(15 - 400, 8))
    # g = Node(Point(15 - 400, 9))
    # h = Node(Point(3 - 400, 9))
    # end = Node(Point(4 - 400,10))

    # start.addNeighbor(a)
    # start.addNeighbor(b)
    # start.addNeighbor(c)
    # start.addNeighbor(d)
    # start.addNeighbor(e)
    # start.addNeighbor(f)
    # start.addNeighbor(g)
    # start.addNeighbor(h)
    # start.addNeighbor(end)

    # obstacles = [[(0 - 400,5), (6 - 400,5), (6 - 400,7), (0 - 400,7)], [(3 - 400,8), (15 - 400,8), (15 - 400,9), (3 - 400,9)]]

    # print crappity_graphity(start.point, end.point, obstacles, "dfs")

    # start.removeIntersectingEdges(obstacles)

    main()


# vim: et sw=4 sts=4
