#!/usr/bin/python3

import odrive
from odrive.utils import start_liveplotter, dump_errors
from odrive.enums import *
from odrive.enums import AXIS_STATE_UNDEFINED, AXIS_STATE_CLOSED_LOOP_CONTROL
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import String, Float32, Float64, UInt16
from carver_interfaces.msg import PinMapped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


class MotorController(Node):

    def __init__(self):
        super().__init__('MotorController')
        best_effort_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
        self.timeout_cmd_vel = 1.0

        self.torque_cons = 25.07
        self.Jerk = 200.0

        self.create_subscription(Twist, "cmd_vel_confidence", self.cmd_vel_callback, 10)
        self.create_subscription(PinMapped, "/xbox_controller", self.xbox_callback, 10)
        # self.create_subscription(UInt16, "/accl_publisher", self.accl_callback, 10)
        self.create_subscription(UInt16, "/accl_publisher", self.accl_callback, best_effort_qos)
        
        self.wheel_velocity_pub = self.create_publisher(Float32, "feedback_wheelspeed", 10)
        self.odrive_vel_pub = self.create_publisher(Float64, "/odrive_cmd_vel", 10)
        self.diag_pub = self.create_publisher(String, "motor_diag", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.create_timer(0.01, self.cmd_loop)
        self.create_timer(0.001, self.cmd_device_loop)
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
        self.xbox_velocity = 0.0
        
        # Accelerator variables
        self.accl_raw = 0
        self.accl_vel = 0.0
        self.accl_acc = 0.0

        self.ster_position = 0.0
        
        self.gear = 1.0
        self.gear_state = 0.0
        self.gear_changing = False
        self.gear_release = True
        
        self.initial_odrive()

    

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
        self.Odrive_TorqueControl()

        print("Finished setup Odrive")
        time.sleep(1)


    def Odrive_VelControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv.axis0.controller.config.vel_ramp_rate = 3.0
        self.odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    
    def Odrive_TorqueControl(self):
        self.odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        # odrv.axis0.controller.config.torque_ramp_rate = 40
        self.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH

    def fetch_values(self):
        return [self.odrv.axis0.encoder.vel_estimate]
        
    def odrive_loop(self):

        self.odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv.axis0.controller.input_vel = self.xbox_velocity * -5.0 * self.gear 
        # self.odrv.axis0.controller.input_vel = self.better_hand_velocity * 10.0 
        self.odrv.axis0.controller.input_vel = float(self.accl_vel)

        msg = Float64()
        msg.data = self.xbox_velocity * -5.0 * self.gear 
        
        self.odrive_vel_pub.publish(msg)
        # print(odrive.utils.dump_errors)
        # odrive.utils.start_liveplotter(self.fetch_values)

            # print(self.odrv.axis0.controller.config.control_mode, self.steering_cmd, self.vx_speed)

    def xbox_callback(self, msg: PinMapped):
        self.xbox_velocity = msg.left.vertical
        self.ster_position = msg.left.horizontal
        self.gear_state = ((msg.rear.lt * -1) + msg.rear.rt) / 2
        
        if self.gear > 5.0: self.gear = 5.0
        if self.gear < 1.0: self.gear = 1.0
        
        if self.gear_state > 0.5:
            self.gear_state = 1.0
        elif self.gear_state < -0.5:
            self.gear_state = -1.0
        else:
            self.gear_state = 0.0 
            
        if self.gear_state == 1.0 or self.gear_state == -1.0:
            if self.gear_release == True:
                self.gear_changing = True
            else:
                self.gear_changing = False
            
        if self.gear_changing == True:
            self.gear += self.gear_state
        
        if self.gear_state == 0.0:
            self.gear_release = True
        else:
            self.gear_release = False
            
        print(self.gear, self.gear_state, self.gear_changing)

    def accl_callback(self, msg: UInt16):
        # hand_velocity_prev = self.hand_velocity
        # self.hand_velocity = msg.data   
        # if self.hand_velocity <= 1.0:
        #     self.hand_velocity = hand_velocity_prev
            
        # self.better_hand_velocity = self.hand_velocity - 1.0
        
        # if self.better_hand_velocity < 0.09:
        #     self.better_hand_velocity = 0.0
        # # self.hand_velocity = (self.hand_velocity - 1)
        # self.get_logger().info(f"hand: {self.better_hand_velocity}")
        raw_data = msg.data
        if raw_data < 15000 :
            raw_data = 0
        
        linear_vel = float(raw_data)*((10.0*2.0*math.pi*0.175)/50000.0)
        self.accl_vel = linear_vel
        self.get_logger().info(f"Set Speed = {self.accl_vel} rps")
        msg = Twist()
        msg.linear.x = linear_vel
        
        self.get_logger().info(f"{msg}")
        self.cmd_vel_pub.publish(msg)
    
    def cmd_loop(self):
        if time.time() - self.previous_cmd_vel_callback <= self.timeout_cmd_vel:
            
            pass
        else:
            pass
            # self.cmd_vel = [0.0, 3950.0]

    def cmd_vel_callback(self, msg):
        # print(msg)
        self.cmd_vel[0] = int(msg.linear.x)
        self.cmd_vel[1] = int(msg.angular.z)
        self.flag = msg.linear.z
        self.previous_cmd_vel_callback = time.time()
        pass

    def cmd_device_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()