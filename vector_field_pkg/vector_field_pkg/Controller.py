#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class TriangleVectorField:
    def __init__(self, vertices, field_vectors):
        self.vertices = np.array(vertices)
        self.field_vectors = np.array(field_vectors)
    
    def barycentric_coordinates(self, point):
        v0 = self.vertices[1] - self.vertices[0]
        v1 = self.vertices[2] - self.vertices[0]
        v2 = point - self.vertices[0]
        d00 = np.dot(v0, v0)
        d01 = np.dot(v0, v1)
        d11 = np.dot(v1, v1)
        d20 = np.dot(v2, v0)
        d21 = np.dot(v2, v1)
        denom = d00 * d11 - d01 * d01
        if denom == 0:
            return np.array([1.0, 0.0, 0.0])
        lambda2 = (d11 * d20 - d01 * d21) / denom
        lambda3 = (d00 * d21 - d01 * d20) / denom
        lambda1 = 1.0 - lambda2 - lambda3
        return np.array([lambda1, lambda2, lambda3])
    
    def get_desired_velocity(self, point):
        lambdas = self.barycentric_coordinates(point)
        desired_vel = (lambdas[0] * self.field_vectors[0] +
                       lambdas[1] * self.field_vectors[1] +
                       lambdas[2] * self.field_vectors[2])
        return desired_vel

class VectorFieldController(Node):
    def __init__(self):
        super().__init__('vector_field_controller')  # <-- Correct way in ROS 2

        # Publisher for velocity commands:
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to odometry for current robot position:
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Triangle definition
        vertices = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
        field_vectors = [[0.5, 0.0], [0.5, 0.2], [0.5, -0.2]]
        self.triangle_field = TriangleVectorField(vertices, field_vectors)
        
        # Initial position
        self.current_position = np.array([0.0, 0.0])

        # Timer instead of manual sleep
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.control_loop_callback)

    def odom_callback(self, msg):
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
    
    def control_loop_callback(self):
        # Compute desired velocity
        desired_vel = self.triangle_field.get_desired_velocity(self.current_position)
        
        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = desired_vel[0]
        twist_msg.linear.y = desired_vel[1]
        twist_msg.angular.z = 0.0

        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = VectorFieldController()
    rclpy.spin(controller)  # ROS 2 spins the node
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()