#!/usr/bin/python3

import odrive
from odrive.enums import *
from odrive.enums import AXIS_STATE_UNDEFINED, AXIS_STATE_CLOSED_LOOP_CONTROL
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import String, Float32, Float64, UInt16,Int8
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from numpy import interp


# 565 -3250

class CarverOdriveManualSteeringNode(Node):
    def __init__(self):
        super().__init__('carver_odrive_manual_steering')
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.timeout_cmd_vel = 1.0

        self.torque_cons = 25.07
        self.Jerk = 200.0
        self.create_subscription(Twist, "/cmd_vel", self.inv_kin, 10)
        self.steering_pub = self.create_publisher(Float32, "/steering_angle", 10)
        self.wheel_velocity_pub = self.create_publisher(Float32, "feedback_wheelspeed", 10)
        
        self.create_timer(0.01, self.odrive_loop)
        self.previous_cmd_vel_callback = 0
        self.ser = None
        self.torque_raw = None
        self.carver_mode = None
        self.prev_mode = None
        self.curr_mode = None
        self.center_steer = 3950
        self.steering_cmd = self.center_steer
        self.steer_raw = 0
        self.accl_vel = 0
        self.accel_dir = 0
        self.initial_odrive()
        self.vx_speed = 0
        self.vx_cmd = 0
        self.w_cmd = 0
    
    def inv_kin(self, msg):
        data = Float32()
        
        L = 1.890
        self.vx_speed = msg.linear.x * 10
        if msg.linear.x != 0:
            data.data = -math.atan(L*msg.angular.z/msg.linear.x)
        self.steering_pub.publish(data)

    def initial_odrive(self):
        self.odrv = odrive.find_any()
        
        while self.odrv.axis0.current_state == AXIS_STATE_UNDEFINED:
            time.sleep(0.01)

        while self.odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            self.odrv.clear_errors()
            self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.01)
        
        time.sleep(1)
        self.Odrive_VelControl()
        

    def Odrive_VelControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv.axis0.controller.config.vel_ramp_rate = 60
        self.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
        print("Finished setup Odrive")
        time.sleep(1)
    
    def odrive_loop(self):

        try:
            # Grab the axis
            axis = self.odrv.axis0
            
            # Check if the error attributes exist and are non-zero
            if hasattr(axis, 'error') and (axis.error != 0 or 
                                        axis.motor.error != 0 or 
                                        axis.encoder.error != 0):
                self.get_logger().error("ODrive Axis Errors detected.")
                self.get_logger().error(f"Axis Error: {hex(axis.error)}")
                self.get_logger().error(f"Motor Error: {hex(axis.motor.error)}")
                self.get_logger().error(f"Encoder Error: {hex(axis.encoder.error)}")

                # Attempt to clear errors
                try:
                    self.odrv.clear_errors()
                    self.get_logger().info("ODrive errors cleared.")
                except Exception as clear_err:
                    self.get_logger().error(f"Failed to clear ODrive errors: {clear_err}")

                # Attempt to reconnect
                self.get_logger().info("Attempting to reconnect to ODrive due to detected errors...")
                try:
                    self.initial_odrive()  # Or your custom re-init method
                    self.get_logger().info("Successfully reconnected to ODrive.")
                except Exception as reconnection_error:
                    self.get_logger().error(f"Failed to reconnect to ODrive: {reconnection_error}")
                    # Depending on your application, you might want to raise or handle further

            # If no errors or after clearing, set the control mode and velocity
            axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            axis.controller.input_vel = self.vx_speed / (2.0 * math.pi * 1.041677 / 9.65)

            # Calculate wheel velocity from the ODrive feedback
            # Example: converting from revolutions per second (RPS) to rad/s, etc.
            wheel_velocity = (
                axis.pos_vel_mapper.vel * 2.0 * math.pi * 1.041677 / 9.65
            )  # Example: Convert RPS to rad/s (user-defined calculation)
            wheel_velocity_msg = Float32()
            wheel_velocity_msg.data = float(wheel_velocity * 0.16)  # Multiply by wheel radius
            self.wheel_velocity_pub.publish(wheel_velocity_msg)

            # self.get_logger().info(
            #     f"Set Speed = {self.accl_vel:.2f} rad/s ({self.vx_speed:.2f} rev/s)"
            # )

        except Exception as e:
            # Catch any other exceptions (including lost connection)
            self.get_logger().error(f"An error occurred: {e}")
            self.get_logger().info("Attempting to reconnect to ODrive...")
            try:
                # Attempt reconnection logic here
                self.initial_odrive()
                self.get_logger().info("Reconnected to ODrive.")
            except Exception as reconnection_error:
                self.get_logger().error(f"Failed to reconnect to ODrive: {reconnection_error}")
        

def main(args=None):
    rclpy.init(args=args)
    node = CarverOdriveManualSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
