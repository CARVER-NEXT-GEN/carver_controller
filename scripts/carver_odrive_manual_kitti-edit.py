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
        self.create_subscription(Float32, "/pedal", self.pedal_callback, best_effort_qos)
        self.create_subscription(Int8, "/accel_direction", self.accel_dir_callback, 10)
        # self.create_subscription(Twist, "cmd_vel_confidence", self.cmd_vel_callback, 10)
        # self.ackaman_feedback_pub = self.create_publisher(AckermannFeedback, "feedback", 10)
        self.wheel_velocity_pub = self.create_publisher(Float32, "feedback_wheelspeed", 10)
        # self.diag_pub = self.create_publisher(String, "motor_diag", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.create_timer(0.01, self.cmd_loop)
        self.create_timer(0.01, self.odrive_loop)
        self.cmd_vel = [0.0, 3950.0]
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
        
    def accel_dir_callback(self, msg: Int8):
        self.accel_dir = msg.data
        #self.get_logger().info(f"Acceleration Direction: {self.accel_dir}")

    def Odrive_VelControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv.axis0.controller.config.vel_ramp_rate = 60
        self.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    
    def Odrive_TorqueControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        # odrv.axis0.controller.config.torque_ramp_rate = 40
        self.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        
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
        # Removed the call to self.Odrive_TorqueControl() if you don't need torque control
        # self.Odrive_TorqueControl()

        print("Finished setup Odrive")
        time.sleep(1)

        
    def odrive_loop(self):
        """
        Periodically called method to command velocity on ODrive and check for errors.
        If any ODrive errors are detected (axis, motor, or encoder), then attempt to reconnect.
        """
        self.vx_speed = self.accl_vel / (2.0 * math.pi)

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
            axis.controller.input_vel = self.vx_speed

            # Calculate wheel velocity from the ODrive feedback
            # Example: converting from revolutions per second (RPS) to rad/s, etc.
            wheel_velocity = (
                axis.pos_vel_mapper.vel * 2.0 * math.pi * 1.041677 / 9.65
            )  # Example: Convert RPS to rad/s (user-defined calculation)
            wheel_velocity_msg = Float32()
            wheel_velocity_msg.data = float(wheel_velocity * 0.16)  # Multiply by wheel radius
            self.wheel_velocity_pub.publish(wheel_velocity_msg)

            # Optionally retrieve and log velocity from the encoder
            self.odrive_get_vel()
            self.get_logger().info(
                f"Set Speed = {self.accl_vel:.2f} rad/s ({self.vx_speed:.2f} rev/s)"
            )

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


    
    def odrive_get_vel(self):
        vel = self.odrv.axis0.vel_estimate
        #print(f"current velocity from odrive is {vel}")
    
    def cmd_loop(self):
        pass

    def cmd_vel_callback(self, msg):
        # print(msg)
        self.cmd_vel[0] = int(msg.linear.x)
        self.cmd_vel[1] = int(msg.angular.z)
        self.flag = msg.linear.z
        self.previous_cmd_vel_callback = time.time()
        pass
    
    def pedal_callback(self, msg):
        raw_data = msg.data
        if raw_data > 0.1 and raw_data < -0.1 :
            raw_data = 0
        linear_vel = raw_data
        
        data = Twist()
        data.linear.x = linear_vel * 60
        self.accl_vel = linear_vel * 60
        self.cmd_vel_pub.publish(data)
        
    def accl_callback(self, msg: UInt16):
        raw_data = msg.data
        if raw_data < 620 :
            raw_data = 0
        

        linear_vel = self.accel_dir*float(interp(float(raw_data),[620,3200],[0,235.5])) # 235.5 radps = 37.5 rev/s
        self.accl_vel = linear_vel
        # self.get_logger().info(f"Set Speed = {self.accl_vel} radps")
        msg = Twist()
        msg.linear.x = linear_vel
        
        # self.get_logger().info(f"{msg}")
        self.cmd_vel_pub.publish(msg)

    def accel_dir_callback(self, msg: Int8):
        self.accel_dir = float(msg.data)

        
        

def main(args=None):
    rclpy.init(args=args)
    node = CarverOdriveManualSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
