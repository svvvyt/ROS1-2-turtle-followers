#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import numpy as np


class PositionCalculator:
    def __init__(self, speed):
        self.speed = speed
        self.current_turtle_position = np.zeros(3)

    def calculateSpeed(self, followed_position: list):
        target, norm = self.calc_target_vector(followed_position)
        current = self.calc_current_direction()
        x = 0.0 if norm < 0.25 else self.speed
        angular = ((np.arctan2(current[0] * target[1] - target[0] * current[1],
                               current[0] * target[0] + current[1] * target[1])) % (2 * np.pi)) - np.pi
        return x, angular * 4.0

    def updatePosition(self, current_turtle_position: list):
        self.current_turtle_position = current_turtle_position

    def calc_current_direction(self):
        angle = self.current_turtle_position[2]
        rotation_matrix = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        v = np.dot(rotation_matrix, np.array([-1.0, 0.0]))
        return v / np.linalg.norm(v)

    def calc_target_vector(self, followed_position):
        v = np.array(followed_position) - self.current_turtle_position[:2]
        norm = np.linalg.norm(v)
        return v / norm if norm != 0 else v, norm


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.speed = self.declare_parameter(
            'speed', 1.0).get_parameter_value().double_value
        self.index = self.declare_parameter(
            'index', 1).get_parameter_value().integer_value
        self.cmdvel_publisher = self.create_publisher(
            Twist, f'/turtle{self.index + 1}/cmd_vel', 10)
        self.turtle_1_pos_subscriber = self.create_subscription(Pose, f'/turtle{self.index}/pose',
                                                                self.handle_followed_turtle_position, 10)
        self.turtle_2_pos_subscriber = self.create_subscription(Pose, f'/turtle{self.index + 1}/pose',
                                                                self.update_current_turtle_position, 10)
        self.calculator = PositionCalculator(self.speed)

    def handle_followed_turtle_position(self, pos):
        x, angle = self.calculator.calculateSpeed([pos.x, pos.y])
        message = Twist()
        message.linear.x = x
        message.angular.z = angle
        self.cmdvel_publisher.publish(message)

    def update_current_turtle_position(self, pos):
        self.calculator.updatePosition([pos.x, pos.y, pos.theta])


def main():
    rclpy.init()
    subscriber = FollowerNode()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
