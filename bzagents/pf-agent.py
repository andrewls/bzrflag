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
import pprint
# from bzrflag.constants import FLAGRADIUS
from bzrc import BZRC, Command

sys.path.insert(0, "/Users/andrew/Code/school/470/bzrflag/potential")
import show_field as graph

FLAGRADIUS = 2.5
FLAGSPREAD = 100

OBSTACLESPREAD = 20

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]
        self.obstacles = self.bzrc.get_obstacles()
        self.potential_fields = {}

        def attractive_fields_func(x, y, res):
            # determine how far from point to flags
            # calculate attractive fields to closest flag
            alpha = 0.9
            closest_flag = None
            distance = float("inf")
            # find closest flag
            for flag in self.flags:
                if self.constants["team"] == flag.color:
                    continue
                distance_to_flag = math.sqrt((flag.x - x)**2 + (flag.y - y)**2)
                if distance_to_flag < distance:
                    distance = distance_to_flag
                    closest_flag = flag
            # calculate angle between closest_flag and tank
            angle = math.atan2(closest_flag.y - y, closest_flag.x - x)
            # calculate dx and dy based off of distance and angle
            if distance < FLAGRADIUS:
                return 0,0
            elif distance < FLAGRADIUS + FLAGSPREAD:
                return alpha * (distance - FLAGRADIUS) * math.cos(angle), alpha * (distance - FLAGRADIUS) * math.sin(angle)
            else:
                return alpha * FLAGSPREAD * math.cos(angle), alpha * FLAGSPREAD * math.sin(angle)

        def repulsive_fields_func(x, y, res):
            alpha = 2
            INFINITY = 1000000

            potential_fields = []
            for obstacle in self.obstacles:
                xavg = 0
                yavg = 0
                for corner in obstacle:
                    xavg += corner[0]
                    yavg += corner[1]
                xavg = xavg / len(obstacle)
                yavg = yavg / len(obstacle)
                radius = math.sqrt((obstacle[0][0] - xavg)**2 + (obstacle[0][1] - yavg)**2)
                distance = math.sqrt((xavg - x)**2 + (yavg - y)**2) #tank distance from center
                angle = math.atan2(yavg - y, xavg - x)
                if distance < radius + OBSTACLESPREAD:
                    potential_fields.append((-alpha * (OBSTACLESPREAD + radius - distance) * math.cos(angle), -alpha * (OBSTACLESPREAD + radius - distance) * math.sin(angle)))

            # merge potential fields
            return self.merge_potential_fields(potential_fields)

        def tangential_fields_func(x, y, res):
            alpha = 1

            potential_fields = []

            for obstacle in self.obstacles:
                xavg = 0
                yavg = 0
                for corner in obstacle:
                    xavg += corner[0]
                    yavg += corner[1]
                xavg = xavg / len(obstacle)
                yavg = yavg / len(obstacle)
                radius = math.sqrt((obstacle[0][0] - xavg)**2 + (obstacle[0][1] - yavg)**2)
                distance = math.sqrt((xavg - x)**2 + (yavg - y)**2) #tank distance from center
                angle = math.atan2(yavg - y, xavg - x) + math.pi/2
                if distance < radius + OBSTACLESPREAD:
                    potential_fields.append((-alpha * (OBSTACLESPREAD + radius - distance) * math.cos(angle), -alpha * (OBSTACLESPREAD + radius - distance) * math.sin(angle)))

            return self.merge_potential_fields(potential_fields)

        def super_tab(x, y, res):
            potential_fields = [attractive_fields_func(x, y, res)]
            potential_fields = potential_fields + [repulsive_fields_func(x, y, res)]
            potential_fields = potential_fields + [tangential_fields_func(x, y, res)]
            merged = self.merge_potential_fields(potential_fields)
            return merged[0], merged[1]

        self.attractive_fields_func = attractive_fields_func
        self.repulsive_fields_func = repulsive_fields_func
        self.tangential_fields_func = tangential_fields_func
        self.super_tab = super_tab

        bases = self.bzrc.get_bases()
        for base in bases:
            if base.color == self.constants['team']:
                self.base = base

        self.commands = []

        for tank in mytanks:
            self.potential_fields[tank.index] = self.calculate_attractive_fields(tank)
            self.potential_fields[tank.index] = self.potential_fields[tank.index] + self.calculate_repulsive_fields(tank, self.obstacles, mytanks + othertanks)
            self.potential_fields[tank.index] = self.potential_fields[tank.index] + self.calculate_tangential_fields(tank)

        for key in self.potential_fields.keys():
            # reduce potential fields to one
            # move in direction based off of dx and dy
            self.potential_fields[key] = self.merge_potential_fields(self.potential_fields[key])

        for tank in mytanks:
            self.move_to_position(tank, tank.x + self.potential_fields[tank.index][0], tank.y + self.potential_fields[tank.index][1])
        results = self.bzrc.do_commands(self.commands)

        # graph.plot_single(self.attractive_fields_func, self.obstacles, "attractive-four-ls.png")
        # graph.plot_single(self.repulsive_fields_func, self.obstacles, "repulsive-four-ls.png")
        # graph.plot_single(self.tangential_fields_func, self.obstacles, "tangential-four-ls.png")
        # graph.plot_single(self.super_tab, self.obstacles, "all-rotated_box.png")

    def return_to_base(self, tank):
        # get the coordinates of the center of the base
        alpha = 0.9
        x = (self.base.corner1_x + self.base.corner2_x + self.base.corner3_x + self.base.corner4_x)/4
        y = (self.base.corner1_y + self.base.corner2_y + self.base.corner3_y + self.base.corner4_y)/4
        distance = math.sqrt((x - tank.x)**2 + (y - tank.y)**2)
        angle = math.atan2(y - tank.y, x - tank.x)
        if distance < FLAGRADIUS + FLAGSPREAD:
            return alpha * (distance - FLAGRADIUS) * math.cos(angle), alpha * (distance - FLAGRADIUS) * math.sin(angle)
        else:
            return alpha * FLAGSPREAD * math.cos(angle), alpha * FLAGSPREAD * math.sin(angle)

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

    def merge_potential_fields(self, fields):
        print fields
        dx = 0
        dy = 0
        for field in fields:
            dx += field[0]
            dy += field[1]
        return (dx, dy)

    def calculate_attractive_fields(self, tank):
        dx = 0
        dy = 0
        if tank.flag == '-':
            dx, dy = self.attractive_fields_func(tank.x, tank.y, 20)
        else:
            dx, dy = self.return_to_base(tank)
        return[(dx, dy)]

    def calculate_repulsive_fields(self, tank, obstacles, tanks):
        dx, dy = self.repulsive_fields_func(tank.x, tank.y, 20)
        return [(dx, dy)]

        # TANKRADIUS = 2
        # TANKSPREAD = 5
        # TANKALPHA = 0.1
        # for other_tank in tanks:
        #     distance = math.sqrt((other_tank.x - tank.x)**2 + (other_tank.y - tank.y)**2)
        #     angle = math.atan2(other_tank.y - tank.y, other_tank.x - tank.x)
        #
        #     if distance < OBSTACLERADIUS:
        #         self.potential_fields[tank.index].append((- math.copysign(1, math.cos(angle)) * INFINITY, -math.copysign(1, math.sin(angle)) * INFINITY))
        #     elif distance < OBSTACLERADIUS + OBSTACLESPREAD:
        #         self.potential_fields[tank.index].append((-TANKALPHA * (OBSTACLESPREAD + OBSTACLERADIUS - distance) * math.cos(angle), -TANKALPHA * (OBSTACLESPREAD + OBSTACLERADIUS - distance) * math.sin(angle)))
        # print self.potential_fields[tank.index][-1]

    def calculate_tangential_fields(self, tank):
        dx, dy = self.tangential_fields_func(tank.x, tank.y, 20)
        return [(dx, dy)]


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
