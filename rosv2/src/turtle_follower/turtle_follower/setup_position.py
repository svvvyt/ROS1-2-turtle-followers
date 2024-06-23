#!/usr/bin/env python3

import rclpy
from turtlesim.srv import Spawn
from rclpy.node import Node
import random


class SpawnerNode(Node):
    def __init__(self):
        super().__init__('spawner_node')
        self.declare_parameter('turtles_count', 1)
        self.turtles_count = self.get_parameter(
            'turtles_count').get_parameter_value().integer_value
        self.turtles_spawned = False
        self.spawner = self.create_client(Spawn, 'spawn')
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        if self.turtles_spawned:
            return
        elif self.spawner.service_is_ready():
            for i in range(2, self.turtles_count + 2):
                request = Spawn.Request()
                request.name = f'turtle{i}'
                request.x = random.uniform(0, 10)
                request.y = random.uniform(0, 10)
                request.theta = 0.0
                self.spawner.call_async(request)
            self.turtles_spawned = True


def main():
    rclpy.init()
    node = SpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
