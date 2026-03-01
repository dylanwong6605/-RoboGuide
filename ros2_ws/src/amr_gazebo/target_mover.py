#!/usr/bin/env python3
"""
target_mover.py
---------------
Drives two person models back and forth, each on a different axis,
mirroring the original person_1 and person_2 actor paths.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PersonMover(Node):
    def __init__(self):
        super().__init__('target_mover')

        # Publisher for person_1 (moves along Y: linear.y)
        self.pub_p1 = self.create_publisher(Twist, '/person_1/cmd_vel', 10)

        # Publisher for person_2 (moves along X: linear.x)
        self.pub_p2 = self.create_publisher(Twist, '/person_2/cmd_vel', 10)

        self.speed = 1.0          # m/s for both
        self.travel_dist = 6.0    # 3 m each way

        # Independent distance trackers and directions
        self.p1_dist = 0.0
        self.p1_dir = 1    # +Y first

        self.p2_dist = 0.0
        self.p2_dir = 1    # +X first

        # person_2 starts offset so they're not in sync
        self.p2_dist = 3.0  # start person_2 halfway through its leg

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            'Target mover started.\n'
            '  person_1: back and forth along Y axis at x=3\n'
            '  person_2: back and forth along X axis at y=1'
        )

    def step(self, dist, direction):
        """Advance one timestep, return (new_dist, new_direction)."""
        dist += self.speed * self.timer_period
        if dist >= self.travel_dist:
            direction *= -1
            dist = 0.0
        return dist, direction

    def timer_callback(self):
        # ── Person 1: Y-axis movement ──────────────────────────────────
        self.p1_dist, self.p1_dir = self.step(self.p1_dist, self.p1_dir)
        msg_p1 = Twist()
        msg_p1.linear.y = self.speed * self.p1_dir   # strafe along Y
        msg_p1.linear.x = 0.0
        msg_p1.angular.z = 0.0
        self.pub_p1.publish(msg_p1)

        # ── Person 2: X-axis movement ──────────────────────────────────
        self.p2_dist, self.p2_dir = self.step(self.p2_dist, self.p2_dir)
        msg_p2 = Twist()
        msg_p2.linear.x = self.speed * self.p2_dir   # forward/back along X
        msg_p2.linear.y = 0.0
        msg_p2.angular.z = 0.0
        self.pub_p2.publish(msg_p2)


def main(args=None):
    rclpy.init(args=args)
    node = PersonMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
