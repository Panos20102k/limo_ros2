#!/usr/bin/env python3
import math
import rclpy
from functools import partial
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# PID Control for velocity regulation

class PIDNode(Node):
    def __init__(self):
        super().__init__("pid")

        self.current_vel = None

        # Target velocity 
        self.target_v_x = 0.5  
        self.target_omega_z = 0.0 

        # Control gains 
        self.Kp_linear = 10.0
        self.Ki_linear = 0.01
        self.Kd_linear = 0.5
        self.Kp_angular = 10.0
        self.Ki_angular = 0.01
        self.Kd_angular = 0.5

        # Initialize errors
        self.integral_err_linear = 0.0
        self.previous_err_linear = 0.0
        self.integral_err_angular = 0.0
        self.previous_err_angular = 0.0

        # Bound in integral windup
        self.integral_bound = 100

        # Control tolerance
        self.tolerance = 0.01

        self.cmd_vel_pub_ = self.create_publisher(
            Twist, "cmd_vel", 10)
        self.odom_sub_ = self.create_subscription(
            Odometry, "odometry", self.odom_sub_callback, 10)
        self.control_loop_timer_ = self.create_timer(0.001, self.control_loop)

    def odom_sub_callback(self, msg):
        # Get current velocity
        self.current_vel = msg.twist.twist
        
    def control_loop(self):
        # Check if odom is received
        if self.current_vel == None:
            return 
        
        # Get P errors
        err_v_x = self.target_v_x - self.current_vel.linear.x
        err_omega_z = self.target_omega_z - self.current_vel.angular.z

        # Get I errors
        self.integral_err_linear += err_v_x
        self.integral_err_angular += err_omega_z
        
        # Bound integral windup
        if self.integral_err_linear >= self.integral_bound:
            self.integral_err_linear = self.integral_bound
        elif self.integral_err_linear <= - self.integral_bound:
            self.integral_err_linear = -self.integral_bound

        if self.integral_err_angular >= self.integral_bound:
            self.integral_err_angular = self.integral_bound
        elif self.integral_err_angular <= - self.integral_bound:
            self.integral_err_angular = -self.integral_bound
        
        # Get D errors
        der_err_v_x = err_v_x - self.previous_err_linear
        der_err_omega_z = err_omega_z - self.previous_err_angular

        # PID Control 
        cmd_vel_msg = Twist()
        if err_v_x >= self.tolerance or err_v_x <= -self.tolerance:
            cmd_vel_msg.linear.x = self.Kp_linear*err_v_x + self.Ki_linear*self.integral_err_linear + self.Kd_linear*der_err_v_x
            self.previous_err_linear = err_v_x
        else:
            cmd_vel_msg.linear.x = 0.0
            self.previous_err_linear = err_v_x

        if err_omega_z >= self.tolerance or err_omega_z <= -self.tolerance:
            cmd_vel_msg.angular.z = self.Kp_angular*err_omega_z + self.Ki_angular*self.integral_err_angular + self.Kd_angular*der_err_omega_z
            self.previous_err_angular = err_omega_z
        else:
            cmd_vel_msg.angular.z = 0.0
            self.previous_err_linear = err_v_x

        # Publish
        self.cmd_vel_pub_.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()