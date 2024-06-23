#! /usr/bin/python3

import rospy
import random
from turtlesim.srv import Spawn


class TurtleSpawner:
    def __init__(self, turtles_count):
        rospy.init_node('spawner_node')

        self.turtles_count = turtles_count
        self.spawn_turtles()

    def spawn_turtles(self):
        rospy.wait_for_service('/spawn')
        for i in range(2, self.turtles_count + 2):
            spawn_func = rospy.ServiceProxy('/spawn', Spawn)
            res = spawn_func(random.uniform(0.0, 10.0),
                             random.uniform(0.0, 10.0),
                             0.0,
                             'turtle' + str(i))


if __name__ == "__main__":
    turtles_count = rospy.get_param("/turtles_count")
    TurtleSpawner(turtles_count)
