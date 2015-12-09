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

import numpy

from bzrc import BZRC, Command
from os import system
from constants import SHOTSPEED

TIME_BETWEEN_TICKS = 0.1
C_VALUE = 0

SHOT_SPEED = 100

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []
        self.time_diff = 0
        self.F = numpy.matrix([[1, TIME_BETWEEN_TICKS, TIME_BETWEEN_TICKS**2, 0, 0, 0],
                               [0, 1, TIME_BETWEEN_TICKS, 0, 0, 0],
                               [0, -C_VALUE, 1, 0, 0, 0],
                               [0, 0, 0, 1, TIME_BETWEEN_TICKS, TIME_BETWEEN_TICKS**2],
                               [0, 0, 0, 0, 1, TIME_BETWEEN_TICKS],
                               [0, 0, 0, 0, -C_VALUE, 1]])
        self.F_Transpose = self.F.transpose()
        self.H = numpy.matrix([[1, 0, 0, 0, 0, 0],
                               [0, 0, 0, 1, 0, 0]])
        self.H_Transpose = self.H.transpose()
        self.sigma_x = numpy.matrix([[0.1, 0, 0, 0, 0, 0],
                                     [0, 0.1, 0, 0, 0, 0],
                                     [0, 0, 100, 0, 0, 0],
                                     [0, 0, 0, 0.1, 0, 0],
                                     [0, 0, 0, 0, 0.1, 0],
                                     [0, 0, 0, 0, 0, 100]])
        self.sigma_z = numpy.matrix([[25, 0],
                                     [0, 25]])
        self.kalman_init()
        self.target_angle = math.pi
        self.iterations = 0

    def kalman_init(self):
        self.mu = numpy.matrix([[0,0,0,0,0,0]]).transpose()
        self.sigma_t = numpy.matrix([[100, 0, 0, 0, 0, 0],
                                     [0, 0.1, 0, 0, 0, 0],
                                     [0, 0, 0.1, 0, 0, 0],
                                     [0, 0, 0, 100, 0, 0],
                                     [0, 0, 0, 0, 0.1, 0],
                                     [0, 0, 0, 0, 0, 0.1]])
        self.target_angle = math.pi

    def kalman_update(self, z_tplus1):
        K_tplus1 = (self.F * self.sigma_t * self.F_Transpose + self.sigma_x) * self.H_Transpose * (self.H * (self.F * self.sigma_t * self.F_Transpose + self.sigma_x) * self.H_Transpose + self.sigma_z).I
        mu_tplus1 = self.F * self.mu + K_tplus1 * (z_tplus1 - self.H * self.F * self.mu)
        sigma_tplus1 = (numpy.identity(6) - K_tplus1 * self.H) * (self.F * self.sigma_t * self.F_Transpose + self.sigma_x)
        self.mu = mu_tplus1
        self.sigma_t = sigma_tplus1

    def kalman_predict(self):
        # this one just returns our best guess about the mu value
        return self.F * self.mu

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
        self.time_diff += time_diff

        print "Iteration %d" % self.iterations

        if self.enemies and not self.enemies[0].x < -900000:
            if self.time_diff > TIME_BETWEEN_TICKS:
                self.time_diff = 0
                for tank in othertanks:
                    print "Enemy tank position: (%d, %d)" % (tank.x, tank.y)
                    print "Updating kalman values"
                    self.kalman_update(numpy.matrix([[tank.x],[tank.y]]))
                    print "New estimates: %r" % self.mu

            for tank in mytanks:
                # first figure out where the enemy tank is
                mu_to_aim_for = self.kalman_predict()
                print "Predicted mu: %r" % mu_to_aim_for
                print "Aiming for coordinates (%f, %f)" % (mu_to_aim_for[0,0], mu_to_aim_for[3,0])
                # figure out how long it will take the shot to actually reach the enemy tank
                distance_from_tank_to_target = math.sqrt((mu_to_aim_for[0,0] - tank.x)**2 + (mu_to_aim_for[3,0] - tank.y)**2)
                time_for_shot_to_reach_enemy_tank = distance_from_tank_to_target / SHOT_SPEED
                print "Time for shot to reach enemy tank: %f" % time_for_shot_to_reach_enemy_tank
                # and now shoot at the current position, plus however far it can get in the amount of time it will take to reach it.
                self.target_angle = math.atan2(mu_to_aim_for[0,0] + mu_to_aim_for[1,0] * time_for_shot_to_reach_enemy_tank - tank.y, mu_to_aim_for[3,0] + mu_to_aim_for[4,0] * time_for_shot_to_reach_enemy_tank - tank.x) * -1 + math.pi/2
                relative_angle = self.normalize_angle(self.target_angle - tank.angle)
                self.bzrc.angvel(tank.index, 2 * relative_angle)
                if abs(relative_angle) < 0.03:
                    self.bzrc.shoot(tank.index)
                # self.commands.append(Command(tank.index, 0, 2 * relative_angle, True))
                print "Current tank position is (%d, %d) with angle %f" % (tank.x, tank.y, tank.angle)
                print "\tGoing for angle %f" % self.target_angle
                print "Relative offset is %f, so angular velocity is %f" % (relative_angle, 2 * relative_angle)

        else:
            self.kalman_init()

        self.iterations += 1

        # print time_diff

        # for tank in mytanks:
            # self.attack_enemies(tank)

        results = self.bzrc.do_commands(self.commands)
        self.iterations

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

    def gnuplotThatShiz(self):
        f = open('tmp.gp', 'w')

        f.write( "set xrange [-400.0: 400.0] \n\
        set yrange [-400.0: 400.0] \n\
        set pm3d \n\
        set view map \n\
        unset key \n\
        set size square \n\
        unset arrow \n\
        set arrow from 0, 0 to -150, 0 nohead front lt 3 \n\
        set arrow from -150, 0 to -150, -50 nohead front lt 3 \n\
        set arrow from -150, -50 to 0, -50 nohead front lt 3 \n\
        set arrow from 0, -50 to 0, 0 nohead front lt 3 \n\
        set arrow from 200, 100 to 200, 330 nohead front lt 3 \n\
        set arrow from 200, 330 to 300, 330 nohead front lt 3 \n\
        set arrow from 300, 330 to 300, 100 nohead front lt 3 \n\
        set arrow from 300, 100 to 200, 100 nohead front lt 3 \n\
        set palette model RGB functions 1-gray, 1-gray, 1-gray \n\
        set isosamples 100 \n\
        sigma_x = 70 \n\
        sigma_y = 100 \n\
        rho = 0.3 \n\
        splot 1.0/(2.0 * pi * sigma_x * sigma_y * sqrt(1 - rho**2) )\
        * exp(-1.0/2.0 * (x**2 / sigma_x**2 + y**2 / sigma_y**2\
        - 2.0*rho*x*y/(sigma_x*sigma_y) ) ) with pm3d\n\
        set term png\n\
        set output \"blah.png\"\n\
        replot")

        f.close()
        system('gnuplot tmp.gp')


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
            prev_time = time.time()
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
