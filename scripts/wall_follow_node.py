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
            self.scan_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # TODO: set PID gains
        self.kp = .8
        self.ki = 0.06
        self.kd = 0.00005

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0 
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.desired_distance = 1
        self.prev_time = self.start_time = Node.get_clock(self).now()
        
    """
    Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

    Args:
        range_data: single range array from the LiDAR
        angle: between angle_min and angle_max of the LiDAR

    Returns:
        range: range measurement in meters at the given angle

    """
    def get_range(self, range_data, angle):
        #TODO: implement
        # Ensure the angle is within the bounds of the LiDAR's angle_min and angle_max
        if angle < range_data.angle_min or angle > range_data.angle_max:
            return float('nan')  # Return NaN if the angle is out of range

        # Calculate the index for the given angle
        index = int((angle - range_data.angle_min) / range_data.angle_increment)

        # Ensure the index is within valid bounds
        if index < 0 or index >= len(range_data.ranges):
            return float('nan')  # Return NaN if the index is out of bounds

        return range_data.ranges[index]

    """
    Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

    Args:
        range_data: single range array from the LiDAR
        dist: desired distance to the wall

    Returns:
        error: calculated error
    """

    def get_error(self, range_data, dist):
        # Measure distances to the left wall
        a = self.get_range(range_data, np.radians(45))  # Convert to radians
        b = self.get_range(range_data, np.radians(90))  # Convert to radians

        # Avoid NaN or inf values
        if np.isnan(a) or np.isnan(b) or np.isinf(a) or np.isinf(b):
            return 0.0  # Return no error if the measurements are invalid

        # Calculate alpha using the provided formula
        alpha = np.arctan2((a * np.cos(np.radians(45)) - b), (a * np.sin(np.radians(45))))

        # Calculate the current distance to the wall
        current_distance = b * np.cos(alpha)

        # Optional: Implement lookahead distance
        L = .75 # Example lookahead distance, adjust as needed
        lookahead_distance = current_distance + (L * np.sin(alpha))

        # Calculate the error
        error = self.desired_distance - lookahead_distance  # Use lookahead distance for error calculation

        self.get_logger().info(f'a: {a}, b: {b}, alpha: {alpha}')

        if 0.0 < abs(error) < .15:
            error = 0.0

        return error
    
    """
    Based on the calculated error, publish vehicle control

    Args:
        error: calculated error
        velocity: desired velocity

    Returns:
        None
    """
    # NEGATIVE error ==> RIGHT 
    # POSITIVE error ==> LEFT
    def pid_control(self, error, velocity):        
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * (error - self.prev_error)

        angle = p + i + d
    
        self.prev_error = error

        # Speed control based on steering angle
        if abs(angle) < np.radians(10):
            velocity = 1.5
        elif abs(angle) < np.radians(20):
            velocity = 1.0
        else:
            velocity = 0.5  
        
        self.get_logger().info(f'Error: {error}')

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        self.drive_publisher.publish(drive_msg)


    
    """
    Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

    Args:
        msg: Incoming LaserScan message

    Returns:
        None
    """
    def scan_callback(self, msg):
        error = self.get_error(msg, self.desired_distance)
        velocity = .5  # Start with a base velocity
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() # type: ignore