#! /usr/bin/python3

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3


class TurtleFollowerNode:
    def __init__(self):
        rospy.init_node('follower_node')

        this_turtle_id = rospy.get_param("~this_turtle_id")
        follow_turtle_id = rospy.get_param("~follow_turtle_id")

        rospy.Subscriber(f'/turtle{follow_turtle_id}/pose',
                         Pose, self.turtle_follow_pose_callback)
        rospy.Subscriber(f'/turtle{this_turtle_id}/pose',
                         Pose, self.turtle_this_pose_callback)

        self.this_turtle_vel_pub = rospy.Publisher(
            f'/turtle{this_turtle_id}/cmd_vel', Twist, queue_size=10)

        self.follow_pose = Pose()
        self.this_pose = Pose()

    def signed_angle_between_vectors(self, vector1, vector2):
        dot_product = sum(a * b for a, b in zip(vector1, vector2))
        magnitude1 = math.hypot(*vector1)
        magnitude2 = math.hypot(*vector2)
        cos_theta = dot_product / (magnitude1 * magnitude2)
        # Clamp cos_theta within [-1, 1]
        cos_theta = max(min(cos_theta, 1), -1)
        angle_rad = math.acos(cos_theta)
        cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
        angle_rad = -angle_rad if cross_product < 0 else angle_rad
        return math.degrees(angle_rad)

    def turtle_follow_pose_callback(self, msg):
        self.follow_pose = msg
        self.translate_turtle()

    def turtle_this_pose_callback(self, msg):
        self.this_pose = msg
        self.translate_turtle()

    def translate_turtle(self):
        transform = Twist()
        transform.linear.x = self.follow_pose.x - self.this_pose.x
        transform.linear.y = self.follow_pose.y - self.this_pose.y

        if transform.linear.x**2 + transform.linear.y**2 < 1:
            cmd_vel = Twist()
        else:
            cmd_vel = Twist(linear=Vector3(1, 0, 0))

        follow_magnitude = math.hypot(transform.linear.x, transform.linear.y)
        follow_dir = (transform.linear.x / follow_magnitude,
                      transform.linear.y / follow_magnitude)

        turtle_dir = (-1 * math.sin(self.this_pose.theta - math.pi / 2),
                      math.cos(self.this_pose.theta - math.pi / 2))

        target_angle = self.signed_angle_between_vectors(
            turtle_dir, follow_dir)
        cmd_vel.angular.z = 1.5 * target_angle

        self.this_turtle_vel_pub.publish(cmd_vel)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    TurtleFollowerNode().run()
