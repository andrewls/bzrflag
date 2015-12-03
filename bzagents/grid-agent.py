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
import random

import numpy
import grid_filter_gl

from bzrc import BZRC, Command

FLAGRADIUS = 2.5
FLAGSPREAD = 100

OBSTACLESPREAD = 20
SEARCH_GRID_SIZE = 20

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.constants["truepositive"] = float(self.constants["truepositive"])
        self.constants["truenegative"] = float(self.constants["truenegative"])
        self.constants["falsepositive"] = 1 - self.constants["truepositive"]
        self.constants["falsenegative"] = 1 - self.constants["truenegative"]
        self.constants["worldsize"] = int(self.constants["worldsize"])
        self.commands = []
        self.obstacles = []
        self.iterations = 0

        # initialize the global occupancy grid
        self.grid = [[0.4 for i in range(int(self.constants["worldsize"]))] for j in range(int(self.constants["worldsize"]))]

        self.search_grid = [[False for i in range(self.constants["worldsize"]/SEARCH_GRID_SIZE)] for j in range(self.constants["worldsize"]/SEARCH_GRID_SIZE)]

        grid_filter_gl.init_window(int(self.constants["worldsize"]), int(self.constants["worldsize"]))
        print self.constants

    def update_occupancy_grid_at_point(self, x, y, probability):
        offset = int(self.constants["worldsize"]) / 2
        self.grid[x + offset][y + offset] = probability

    def get_occupancy_grid_at_point(self, x, y):
        offset = int(self.constants["worldsize"]) / 2
        return self.grid[x + offset][y + offset]

    def get_search_grid_region_center_points(self):
        # each region has a center
        search_grid_squares = self.constants["worldsize"] / SEARCH_GRID_SIZE
        offset = search_grid_squares / 2
        center_values = [i * search_grid_squares + offset for i in range(SEARCH_GRID_SIZE)]
        points = []
        for i in range(SEARCH_GRID_SIZE):
            for j in range(SEARCH_GRID_SIZE):
                points.append((center_values[i], center_values[j]))
        return points
        # return [[(center_values[i], center_values[j]) for j in range(SEARCH_GRID_SIZE)] for i in range(SEARCH_GRID_SIZE)]

    def get_search_grid_unsearched_regions(self):
        search_grid_squares = self.constants["worldsize"] / SEARCH_GRID_SIZE
        center_points = self.get_search_grid_region_center_points()
        grid_to_world_offset = self.constants["worldsize"] / 2
        unsearched_regions = []
        for (x, y) in center_points:
            # loop over all of the region within the grid
            converged_cells = 0
            for i in range(x - search_grid_squares/2, x + search_grid_squares/2):
                for j in range(y - search_grid_squares/2, y + search_grid_squares/2):
                    probability = self.get_occupancy_grid_at_point(i - grid_to_world_offset, j - grid_to_world_offset)
                    if probability < 0.2 or probability > 0.8:
                        converged_cells += 1
            # mark the region as unsearched if less than 95% of the cells have converged
            if converged_cells < (search_grid_squares * search_grid_squares * 0.65):
                unsearched_regions.append((x, y))
        return unsearched_regions

    def get_nearest_unsearched_region_to_point(self, x, y):
        centers = self.get_search_grid_unsearched_regions()
        if self.iterations == 5:
            print "Getting nearest unsearched region to (%d, %d)" % (x, y)
        if centers:
            distance = float("INF")
            closest = centers[0]
            for center in centers:
                center = (center[0] - 400, center[1] - 400)
                distance_to_center = math.sqrt((x - center[0])**2 + (y - center[1])**2)
                if distance_to_center < distance:
                    distance = distance_to_center
                    closest = center
                    if self.iterations == 5:
                        print "New closest center point is (%d, %d) with distance %f" % (closest[0], closest[1], distance)
            return (closest[0] + 400, closest[1] + 400), distance
        else:
            # if there are no unsearched regions, just let the tanks drive in circles.
            return (x, y), 0

    def get_random_unsearched_region(self):
        regions = self.get_search_grid_unsearched_regions()
        return regions[random.randrange(len(regions))]

    def get_random_region(self):
        regions = self.get_search_grid_region_center_points()
        return regions[random.randrange(len(regions))]

    def done_exploring(self):
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                if self.grid[i][j] > 0.2 and self.grid[i][j] < 0.8:
                    return False
        return True

    def figure_out_where_to_go(self, x, y):
        if self.iterations % 100 == 0: # let it move toward the new point for two full iterations
            print "Selecting using random region."
            target_point = self.get_random_region()
        elif self.iterations % 50 == 0:
            print "Selecting using random unsearched region."
            target_point = self.get_random_unsearched_region()
        else :
            target_point, distance = self.get_nearest_unsearched_region_to_point(x, y)


        target_point = (target_point[0] - 400, target_point[1] - 400)
        print "Moving toward point (%d, %d)" % target_point
        return target_point

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []
        self.iterations += 1

        print "Iteration %d" % self.iterations

        for tank in mytanks:
            # get the occgrid for that tank
            occgrid = self.bzrc.get_occgrid(tank.index)
            starting_point = occgrid[0]
            occgrid = occgrid[1]
            # so now we have a grid that starts at starting_point and goes up and right. The first point is the starting point, and then each point moves up the world
            for i in range(len(occgrid)):
                for j in range(len(occgrid[i])):
                    observation = occgrid[i][j]
                    # so we have our observation. Let's update the probabilities
                    prior_probability = self.get_occupancy_grid_at_point(starting_point[0] + i, starting_point[1] + j)
                    # p(observation = occupied) = p(observation = occupied | state = occupied) * p(state = occupied) + p(observation = occupied | state = unoccupied) * p(state = unnocupied)
                    # so the observation probability is just the true positive rate times the prior plus the false positive rate times 1 - the prior
                    if observation:
                        observation_probability = self.constants["truepositive"] * prior_probability + self.constants["falsepositive"] * (1 - prior_probability)
                    else:
                        observation_probability = self.constants["truenegative"] * (1 - prior_probability) + self.constants["falsenegative"] * prior_probability

                    # the likelihood just depends on what the observation actually was
                    # if the observation is occupied, we want p(observation = occupied | state = occupied), or the true positive rate
                    # if the observation is unoccupied, we want p(observation = unoccupied | state = occupied), or the false negative rate
                    likelihood = self.constants["truepositive"] if observation else self.constants["falsenegative"]

                    # p(state = occupied | observation = occupied) = p(observation = occupied | state = occupied) * p(state = occupied) / p(observation = occupied)
                    # p(state = occupied | observation = unnoccupied) = p(observation = unoccupied | state = occupied) * p(satet = occupied) / p(observation = unoccupied)
                    new_probability = likelihood * prior_probability / observation_probability

                    # and finally, update the probability at that grid location
                    self.update_occupancy_grid_at_point(starting_point[0] + i, starting_point[1] + j, new_probability)
        if self.iterations % 5 == 0:
            grid_filter_gl.update_grid(numpy.array(self.grid))
            grid_filter_gl.draw_grid()
        if self.done_exploring():
            sys.exit()

        # user potential fields to explore
        self.potential_fields = {}


        def attractive_fields_func(x, y, res):
            # determine how far from point to flags
            # calculate attractive fields to closest flag
            alpha = 10
            closest_flag = None
            distance = float("inf")
            # find closest flag
            self.flags = []
            # found_unexplored_point = False
            # for i in range(len(self.grid)):
            #     for j in range(len(self.grid[i])):
            #         if self.grid[i][j] > 0.2 and self.grid[i][j] < 0.8:
            #             found_unexplored_point = True
            #             closest_flag = (i - 400, j - 400)
            #             distance = math.sqrt((i - 400 - x)**2 + (j - 400 - y)**2)
            #             print "Moving to point (%d, %d)" % closest_flag
            #             break
            #     if found_unexplored_point:
            #         break

            # for i in range(len(self.grid)):
            #     self.flags = self.flags + [(i - 400 + (i * 20), j - 400 + (j * 20)) for j in range(len(self.grid[i])) if self.grid[i][j] > 0.2 and self.grid[i][j] < 0.8]
            # found_unexplored_point = False
            # for flag in self.flags:
            #     distance_to_flag = math.sqrt((flag[0] - x)**2 + (flag[1] - y)**2)
            #     if distance_to_flag < distance:
            #         distance = distance_to_flag
            #         closest_flag = flag

            if self.iterations % 100 == 0: # let it move toward the new point for two full iterations
                print "Selecting using random region."
                closest_flag = self.get_random_region()
                distance = math.sqrt((x - closest_flag[0])**2 + (y - closest_flag[1])**2)
            elif self.iterations % 50 == 0:
                print "Selecting using random unsearched region."
                closest_flag = self.get_random_unsearched_region()
                distance = math.sqrt((x - closest_flag[0])**2 + (y - closest_flag[1])**2)
            else :
                closest_flag, distance = self.get_nearest_unsearched_region_to_point(x, y)


            closest_flag = (closest_flag[0] - 400, closest_flag[1] - 400)
            print "Moving toward point (%d, %d)" % closest_flag


            # calculate angle between closest_flag and tank
            angle = math.atan2(closest_flag[1] - y, closest_flag[0] - x)
            # calculate dx and dy based off of distance and angle
            if distance < FLAGRADIUS:
                return 0,0
            elif distance < FLAGRADIUS + FLAGSPREAD:
                return alpha * (distance - FLAGRADIUS) * math.cos(angle), alpha * (distance - FLAGRADIUS) * math.sin(angle)
            else:
                return alpha * FLAGSPREAD * math.cos(angle), alpha * FLAGSPREAD * math.sin(angle)

        def repulsive_fields_func(x, y, res):
            alpha = 10
            INFINITY = 1000000

            potential_fields = []

            for i in range(len(self.grid)): #row
                for j in range(len(self.grid[i])): #point
                    if self.grid[i][j] > .8:
                        xavg = i - 400
                        yavg = j - 400
                        # for corner in obstacle:
                        #     xavg += corner[0]
                        #     yavg += corner[1]
                        # xavg = xavg / len(obstacle)
                        # yavg = yavg / len(obstacle)
                        radius = 10 #todo frob this later
                        distance = math.sqrt((xavg - x)**2 + (yavg - y)**2) #tank distance from center
                        angle = math.atan2(yavg - y, xavg - x)
                        if distance < radius + OBSTACLESPREAD:
                            potential_fields.append((-alpha * (OBSTACLESPREAD + radius - distance) * math.cos(angle), -alpha * (OBSTACLESPREAD + radius - distance) * math.sin(angle)))

            # merge potential fields
            return self.merge_potential_fields(potential_fields)

        def tangential_fields_func(x, y, res):
            alpha = 1

            potential_fields = []

            for i in range(len(self.grid)):
                for j in range(len(self.grid[i])):
                    if i > 795 or j > 795:
                        continue
                    if (self.grid[i][j] > 0.8 and self.grid[i+1][j] > 0.8 and self.grid[i+2][j] > 0.8 and self.grid[i+3][j] > 0.8) or (self.grid[i][j] > 0.8 and self.grid[i][j+1] > 0.8 and self.grid[i][j+2] > 0.8 and self.grid[i][j+3] > 0.8):
                        xavg = i - 400
                        yavg = j - 400
                        radius = 10 # magic number - frob
                        distance = math.sqrt((xavg - x)**2 + (yavg - y)**2) #tank distance from center
                        angle = math.atan2(yavg - y, xavg - x) + math.pi/2
                        if distance < radius + OBSTACLESPREAD:
                            potential_fields.append((-alpha * (OBSTACLESPREAD + radius - distance) * math.cos(angle), -alpha * (OBSTACLESPREAD + radius - distance) * math.sin(angle)))

            # for obstacle in self.obstacles:
            #     xavg = 0
            #     yavg = 0
            #     for corner in obstacle:
            #         xavg += corner[0]
            #         yavg += corner[1]
            #     xavg = xavg / len(obstacle)
            #     yavg = yavg / len(obstacle)
            #     radius = math.sqrt((obstacle[0][0] - xavg)**2 + (obstacle[0][1] - yavg)**2)
            #     distance = math.sqrt((xavg - x)**2 + (yavg - y)**2) #tank distance from center
            #     angle = math.atan2(yavg - y, xavg - x) + math.pi/2
            #     if distance < radius + OBSTACLESPREAD:
            #         potential_fields.append((-alpha * (OBSTACLESPREAD + radius - distance) * math.cos(angle), -alpha * (OBSTACLESPREAD + radius - distance) * math.sin(angle)))

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

        if self.iterations % 5 == 0 and (self.iterations % 50 != 5 and self.iterations % 50 != 10):
            for tank in mytanks:
                # self.potential_fields[tank.index] = self.calculate_attractive_fields(tank)
                # potential_fields = self.calculate_attractive_fields(tank)
                target_point = self.figure_out_where_to_go(tank.x, tank.y)
                self.move_to_position(tank, target_point[0], target_point[1])
                # self.potential_fields[tank.index] = self.potential_fields[tank.index] + self.calculate_repulsive_fields(tank, self.obstacles, mytanks + othertanks)
                # self.potential_fields[tank.index] = self.potential_fields[tank.index] + self.calculate_tangential_fields(tank)

            # actually move the tanks
            # for key in self.potential_fields.keys():
                # reduce potential fields to one
                # move in direction based off of dx and dy
                # self.potential_fields[key] = self.merge_potential_fields(self.potential_fields[key])

            # for tank in mytanks:
                # self.move_to_position(tank, tank.x + self.potential_fields[tank.index][0], tank.y + self.potential_fields[tank.index][1])
        results = self.bzrc.do_commands(self.commands)

    def calculate_attractive_fields(self, tank):
        dx, dy = self.attractive_fields_func(tank.x, tank.y, 20)
        return [(dx, dy)]

    def calculate_repulsive_fields(self, tank, obstacles, tanks):
        dx, dy = self.repulsive_fields_func(tank.x, tank.y, 20)
        return [(dx, dy)]

    def calculate_tangential_fields(self, tank):
        dx, dy = self.tangential_fields_func(tank.x, tank.y, 20)
        return [(dx, dy)]

    def attack_enemies(self, tank):
        """Find the closest enemy and chase it, shooting as you go."""
        best_enemy = None
        best_dist = 2 * float(self.constants['worldsize'])
        for enemy in self.enemies:
            if enemy.status != 'alive':
                continue
            dist = math.sqrt((enemy.x - tank.x)**2 + (enemy.y - tank.y)**2)
            if dist < best_dist:
                best_dist = dist
                best_enemy = enemy
        if best_enemy is None:
            command = Command(tank.index, 0, 0, False)
            self.commands.append(command)
        else:
            self.move_to_position(tank, best_enemy.x, best_enemy.y)

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
        dx = 0
        dy = 0
        for field in fields:
            dx += field[0]
            dy += field[1]
        return (dx, dy)

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
