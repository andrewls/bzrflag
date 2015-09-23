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
# from bzrflag.constants import FLAGRADIUS
from bzrc import BZRC, Command

FLAGRADIUS = 2.5
FLAGSPREAD = 100

OBSTACLERADIUS = 1
OBSTACLESPREAD = 10

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

        bases = self.bzrc.get_bases()
        for base in bases:
            if base.color == self.constants['team']:
                self.base = base

        self.commands = []

        for tank in mytanks:
            self.potential_fields[tank.index] = []
            self.calculate_attractive_fields(tank, flags)
            self.calculate_repulsive_fields(tank, self.obstacles, mytanks + othertanks)
            self.calculate_tangential_fields()

        for key in self.potential_fields.keys():
            # reduce potential fields to one
            # move in direction based off of dx and dy
            self.merge_potential_fields(self.potential_fields[key], key)

        for tank in mytanks:
            self.move_to_position(tank, tank.x + self.potential_fields[tank.index][0], tank.y + self.potential_fields[tank.index][1])
        results = self.bzrc.do_commands(self.commands)

    def return_to_base(self, tank):
        # get the coordinates of the center of the base
        alpha = 0.9
        x = (self.base.corner1_x + self.base.corner2_x + self.base.corner3_x + self.base.corner4_x)/4
        y = (self.base.corner1_y + self.base.corner2_y + self.base.corner3_y + self.base.corner4_y)/4
        distance = math.sqrt((x - tank.x)**2 + (y - tank.y)**2)
        angle = math.atan2(y - tank.y, x - tank.x)
        if distance < FLAGRADIUS + FLAGSPREAD:
            self.potential_fields[tank.index].append((alpha * (distance - FLAGRADIUS) * math.cos(angle), alpha * (distance - FLAGRADIUS) * math.sin(angle)))
        else:
            self.potential_fields[tank.index].append((alpha * FLAGSPREAD * math.cos(angle), alpha * FLAGSPREAD * math.sin(angle)))

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

    def merge_potential_fields(self, fields, key):
        dx = 0
        dy = 0
        print "Premerge"
        print fields
        for field in fields:
            dx += field[0]
            dy += field[1]
        self.potential_fields[key] = (dx, dy)
        print "Postmerger"
        print self.potential_fields[key]

    def calculate_attractive_fields(self, tank, flags):
        alpha = 0.9

        closest_flag = None
        distance = float("inf")
        # find closest flag
        if tank.flag != '-':
            self.return_to_base(tank)
        else:
            for flag in flags:
                if self.constants["team"] == flag.color:
                    continue
                distance_to_flag = math.sqrt((flag.x - tank.x)**2 + (flag.y - tank.y)**2)
                if distance_to_flag < distance:
                    distance = distance_to_flag
                    closest_flag = flag

            # get flag location
            # calculate distance between flag and tank
            distance = math.sqrt((closest_flag.x - tank.x)**2 + (closest_flag.y - tank.y)**2)
            # calculate angle between closest_flag and tank
            angle = math.atan2(closest_flag.y - tank.y, closest_flag.x - tank.x)
            # calculate dx and dy based off of distance and angle
            if distance < FLAGRADIUS:
                self.potential_fields[tank.index].append((0, 0))
            elif distance < FLAGRADIUS + FLAGSPREAD:
                self.potential_fields[tank.index].append((alpha * (distance - FLAGRADIUS) * math.cos(angle), alpha * (distance - FLAGRADIUS) * math.sin(angle)))
            else:
                self.potential_fields[tank.index].append((alpha * FLAGSPREAD * math.cos(angle), alpha * FLAGSPREAD * math.sin(angle)))

    def calculate_repulsive_fields(self, tank, obstacles, tanks):
        alpha = 0.9
        INFINITY = 1000000
        for obstacle in obstacles:
            for corner in obstacle:
                distance = math.sqrt((corner[0] - tank.x)**2 + (corner[1] - tank.y)**2)
                angle = math.atan2(corner[1] - tank.y, corner[0] - tank.x)
                print "Tank %d is %f from obstacle" % (tank.index, distance)

                if distance < OBSTACLERADIUS:
                    self.potential_fields[tank.index].append((- math.copysign(1, math.cos(angle)) * INFINITY, -math.copysign(1, math.sin(angle)) * INFINITY))
                elif distance < OBSTACLERADIUS + OBSTACLESPREAD:
                    self.potential_fields[tank.index].append((-alpha * (OBSTACLESPREAD + OBSTACLERADIUS - distance) * math.cos(angle), -alpha * (OBSTACLESPREAD + OBSTACLERADIUS - distance) * math.sin(angle)))
                print "Potential field that was calculated:"

        TANKRADIUS = 2
        TANKSPREAD = 5
        TANKALPHA = 0.1
        # for other_tank in tanks:
        #     distance = math.sqrt((other_tank.x - tank.x)**2 + (other_tank.y - tank.y)**2)
        #     angle = math.atan2(other_tank.y - tank.y, other_tank.x - tank.x)
        #
        #     if distance < OBSTACLERADIUS:
        #         self.potential_fields[tank.index].append((- math.copysign(1, math.cos(angle)) * INFINITY, -math.copysign(1, math.sin(angle)) * INFINITY))
        #     elif distance < OBSTACLERADIUS + OBSTACLESPREAD:
        #         self.potential_fields[tank.index].append((-TANKALPHA * (OBSTACLESPREAD + OBSTACLERADIUS - distance) * math.cos(angle), -TANKALPHA * (OBSTACLESPREAD + OBSTACLERADIUS - distance) * math.sin(angle)))
        print self.potential_fields[tank.index][-1]

    def calculate_tangential_fields(self):
        pass

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
