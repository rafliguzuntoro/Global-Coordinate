#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class GlobalCoordinate(Node):

    def __init__(self):
        super().__init__('global_coordinate')

        self.wheel_radius = 0.05
        self.wheel_separation = 0.5

        self.theta = 0.0

        self.phi_left = 0.0
        self.phi_right = 0.0

        self.create_subscription(
            Float64MultiArray,
            '/wheel_velocity_input',
            self.wheel_velocity_callback,
            10
        )

        self.left_wheel_pub = self.create_publisher(
            Float64,
            '/left_wheel/command',
            10
        )

        self.right_wheel_pub = self.create_publisher(
            Float64,
            '/right_wheel/command',
            10
        )

        self.global_velocity_pub = self.create_publisher(
            Twist,
            '/global_velocity',
            10
        )

        self.dt = 0.1

        self.timer = self.create_timer(
            self.dt,
            self.calculate_global_velocity
        )

        self.get_logger().info(
            'Global Coordinate aktif. '
            'Input: [kecepatan roda kiri, kecepatan roda kanan]'
        )

    def wheel_velocity_callback(self, msg):

        if len(msg.data) != 2:
            self.get_logger().warning(
                'Input harus berisi tepat 2 nilai: '
                '[roda_kiri, roda_kanan]'
            )
            return

        self.phi_left = float(msg.data[0])
        self.phi_right = float(msg.data[1])

        left_msg = Float64()
        left_msg.data = self.phi_left
        self.left_wheel_pub.publish(left_msg)

        right_msg = Float64()
        right_msg.data = self.phi_right
        self.right_wheel_pub.publish(right_msg)

        self.get_logger().info(
            f'Input roda kiri = {self.phi_left:.3f} rad/s | '
            f'Input roda kanan = {self.phi_right:.3f} rad/s'
        )

    def calculate_global_velocity(self):

        x_dot = (
            self.wheel_radius / 2.0
            * (self.phi_left + self.phi_right)
            * math.cos(self.theta)
        )

        y_dot = (
            self.wheel_radius / 2.0
            * (self.phi_left + self.phi_right)
            * math.sin(self.theta)
        )

        theta_dot = (
            self.wheel_radius / self.wheel_separation
            * (self.phi_right - self.phi_left)
        )

        self.theta += theta_dot * self.dt

        msg = Twist()
        msg.linear.x = x_dot
        msg.linear.y = y_dot
        msg.angular.z = theta_dot

        self.global_velocity_pub.publish(msg)

        self.get_logger().info(
            f'x_dot = {x_dot:.3f} m/s | '
            f'y_dot = {y_dot:.3f} m/s | '
            f'theta_dot = {theta_dot:.3f} rad/s | '
            f'theta = {self.theta:.3f} rad'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GlobalCoordinate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
