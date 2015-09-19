#!/usr/bin/python -tt

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
        self.next_event_time = 0
        self.next_event = "move"
        self.shoot_time = 0

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        if time_diff > self.next_event_time:
            self.commands = []
            if self.next_event == "move":
                for tank in mytanks:
                    self.drive_forward(tank)
                self.next_event = "turn"
                self.next_event_time = time_diff + random.uniform(3, 8)
            elif self.next_event == "turn":
                for tank in mytanks:
                    self.turn_sixty_degrees(tank)
                self.next_event = "move"
                self.next_event_time = time_diff + 1.5

        # also shoot every two seconds or so
        if time_diff > self.shoot_time:
            for tank in mytanks:
                self.shoot(tank)
            self.shoot_time = time_diff + 2
        results = self.bzrc.do_commands(self.commands)


    def turn_sixty_degrees(self, tank):
        command = Command(tank.index, 0, math.pi / 3, False)
        self.commands.append(command)

    def drive_forward(self, tank):
        command = Command(tank.index, 1, 0, False)
        self.commands.append(command)

    def shoot(self, tank):
        self.bzrc.shoot(tank.index)

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
