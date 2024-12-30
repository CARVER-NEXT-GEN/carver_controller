#!/usr/bin/python3

import odrive
from odrive.enums import *
from odrive.enums import AXIS_STATE_UNDEFINED, AXIS_STATE_CLOSED_LOOP_CONTROL
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import String, Float32, Float64, UInt16
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

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
        self.create_subscription(UInt16, "/accl_publisher", self.accl_callback, best_effort_qos)

        # self.create_subscription(Twist, "cmd_vel_confidence", self.cmd_vel_callback, 10)
        # self.ackaman_feedback_pub = self.create_publisher(AckermannFeedback, "feedback", 10)
        # self.wheel_velocity_pub = self.create_publisher(Float32, "feedback_wheelspeed", 10)
        self.diag_pub = self.create_publisher(String, "motor_diag", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Create longitudinal speed publisher that get velocity from odrive. 
        self.longitudinal_pub = self.create_publisher(Float32, "longitudinal", 10)
        self.position_pub = self.create_publisher(Float32,"position", 10)
        
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
        self.initial_odrive()

    def Odrive_VelControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv.axis0.controller.config.vel_ramp_rate = 20
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
        self.vx_speed = self.accl_vel / (2.0 * math.pi)

        try:
            axis = self.odrv.axis0

            # Check if 'error' attribute exists
            if hasattr(axis, 'error') and (axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0):
                self.get_logger().error("ODrive Axis Errors detected.")
                self.get_logger().error(f"Axis Error: {hex(axis.error)}")
                self.get_logger().error(f"Motor Error: {hex(axis.motor.error)}")
                self.odrv.clear_errors()
                self.get_logger().info("Errors cleared.")
                
            self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            self.odrv.axis0.controller.input_vel = self.vx_speed
            # Get odrive speed from encoder.
            self.odrive_get_vel()
            self.odrive_get_pos()
            self.get_logger().info(f"Set Speed = {self.accl_vel} rad/s {self.vx_speed} rev/s")

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.get_logger().info("Attempting to reconnect to ODrive...")
            try:
                
                self.get_logger().info("Reconnected to ODrive.")
                # Re-initialize ODrive settings
                self.initial_odrive()
            except Exception as reconnection_error:
                self.get_logger().error(f"Failed to reconnect to ODrive: {reconnection_error}")


            # print(self.odrv.axis0.controller.config.control_mode, self.steering_cmd, self.vx_speed)
    
    def odrive_get_vel(self):
        vel = self.odrv.axis0.vel_estimate*2.0*math.pi*0.175
        msg = Float32()
        msg.data = float(vel)
        self.longitudinal_pub.publish(msg)
        print(f"current velocity from odrive is {vel}")
    
    def odrive_get_pos(self):
        pos = self.odrv.axis0.pos_estimate*2.0*math.pi*0.175
        msg = Float32()
        msg.data = float(pos)
        self.position_pub.publish(msg)
    
    def cmd_loop(self):
        pass

    def cmd_vel_callback(self, msg):
        # print(msg)
        self.cmd_vel[0] = int(msg.linear.x)
        self.cmd_vel[1] = int(msg.angular.z)
        self.flag = msg.linear.z
        self.previous_cmd_vel_callback = time.time()
        pass
    
    def accl_callback(self, msg: UInt16):
        raw_data = msg.data
        if raw_data < 15000 :
            raw_data = 0
        
        linear_vel = float(raw_data)*((3.0*60.0*2.0*math.pi*0.175)/50000.0)
        self.accl_vel = linear_vel
        # self.get_logger().info(f"Set Speed = {self.accl_vel} radps")
        msg = Twist()
        msg.linear.x = linear_vel
        
        # self.get_logger().info(f"{msg}")
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarverOdriveManualSteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
