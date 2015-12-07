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

from bzrc import BZRC, Command

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
        self.iterations = 1
        self.commands = []

        for tank in mytanks:
            if abs(tank.angle) > math.pi/4:
                if self.iterations == 0:
                    self.commands.append(Command(tank.index, 0, random.randrange(-5, 5) * 10, 0)) # turn so we're moving towards the other agent
            else:
                speed = random.uniform(-0.5, 1.3)
                angval = self.turn() if self.iterations % 500 == 0 else 0
                print speed
                self.commands.append(Command(tank.index, speed, angval, 0))  # just move in a straignt line
            # if self.iterations % 500 == 0:
            #     turn = True if random.randrange(0, 100) > 50 else False
            #     if turn:
            #         self.turn_sixty_degrees(tank)

        results = self.bzrc.do_commands(self.commands)
        self.iterations += 1

    def turn_sixty_degrees(self, tank):
        amountToTurn = 1 if random.randrange(0, 100) > 50 else -1
        return (math.pi / 3) * amountToTurn

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
