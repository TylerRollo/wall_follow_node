#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import rclpy.time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        # TODO: create subscribers and publishers
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.error_calculation,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )


        # Initialize variables
        self.kp = 5.0
        self.ki = 0.0
        self.kd = 0.2
        self.look_ahead_distance = 0.05
        self.steer_angle = 0.0
        self.last_time_reading = self.get_clock().now()
        self.current_time_reading = self.get_clock().now()
        self.error_dist = 0.0
        self.last_error_dist = 0.0
        self.integral = 0.0
        self.speed = 0.0
        self.initial_time = self.get_clock().now()

    def get_duration_in_seconds(self, time_now, time_then):
        return (time_now - time_then).nanoseconds * 1e-9

    def error_calculation(self, scan_msg):
        angle_increment = 0.005823
        theta_a = angle_increment * 195  # Angle for a_dist (65 degrees)
        theta_c = angle_increment * 165  # Angle for c_dist (55 degrees)

        b_dist = scan_msg.ranges[809]
        a_dist = scan_msg.ranges[614]
        c_dist = scan_msg.ranges[644]

        # Calculate the angles and distances
        a1_angle = np.arctan((a_dist * np.cos(theta_a) - b_dist) / (a_dist * np.sin(theta_a)))
        d_ta = b_dist * np.cos(a1_angle)

        a2_angle = np.arctan((c_dist * np.cos(theta_c) - b_dist) / (c_dist * np.sin(theta_c)))
        d_tc = b_dist * np.cos(a2_angle)

        d_t = 0.5 * (d_ta + d_tc)

        # Error distance calculation
        self.error_dist = 1.35 - d_t
        if self.get_duration_in_seconds(self.get_clock().now(), self.initial_time) < 5:
            self.error_dist *= 0.3

        self.pid_controller()

    def pid_controller(self):
        self.current_time_reading = self.get_clock().now()
        dt = self.get_duration_in_seconds(self.current_time_reading, self.last_time_reading)

        # PID calculations
        self.integral += self.error_dist * dt
        d_error = (self.error_dist - self.last_error_dist) / dt if dt > 0 else 0.0
        self.last_time_reading = self.current_time_reading
        self.last_error_dist = self.error_dist

        if self.get_duration_in_seconds(self.current_time_reading, self.initial_time) < 5:
            d_error = 0  # Cancel the initial volatile behavior of d_error

        self.steer_angle = -self.kp * self.error_dist + self.ki * self.integral + self.kd * d_error

        # Clamp the steering angle
        self.steer_angle = max(min(self.steer_angle, np.pi), -np.pi)

        # Speed control based on the steering angle
        if abs(self.steer_angle) < 0.1745:
            self.speed = 1.5
        elif 0.1745 <= abs(self.steer_angle) < 0.349:
            self.speed = 1.0
        else:
            self.speed = 0.3

        # Publish Ackermann message
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.speed = self.speed
        ackermann_msg.drive.steering_angle = self.steer_angle
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = "laser"
        self.drive_publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = WallFollow()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
