#!/usr/bin/python3

from carver_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python import get_package_share_directory
import os
import yaml
import numpy as np


class CarverMessenger(Node):
    def __init__(self):
        super().__init__('carver_messenger_node')
        self.get_logger().info('carver messenger node has been start')

        # Declare parameters
        self.declare_parameter('file', 'imu_covariance.yaml')
        self.declare_parameter('mode', 'publish_data')  # Default mode is 'publish_data'

        pkg_name = 'carver_controller'

        imu_cov_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = imu_cov_pkg_share_path.split('install')

        file = self.get_parameter('file').value
        self.imu_cov_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)

        # Retrieve mode
        self.mode = self.get_parameter('mode').value
        if self.mode not in ['save_covariance', 'publish_data']:
            self.get_logger().error(f"Invalid mode: {self.mode}. Must be 'save_covariance' or 'publish_data'")
            rclpy.shutdown()
            return
        if self.mode == 'publish_data':   
            with open(self.imu_cov_path, 'r') as file:
                value = yaml.safe_load(file)
            self.euler_cov_086 = self.extract_covariance(value, 'euler covariance_BNO086')
            self.euler_cov_055 = self.extract_covariance(value, 'euler covariance_BNO055')
            self.acc_cov_086 = self.extract_covariance(value, 'acc covariance_BNO086')
            self.acc_cov_055 = self.extract_covariance(value, 'acc covariance_BNO055')
            self.gyro_cov_086 = self.extract_covariance(value, 'gyro covariance_BNO086')
            self.gyro_cov_055 = self.extract_covariance(value, 'gyro covariance_BNO055')
            self.mag_cov_086 = self.extract_covariance(value, 'mag covariance_BNO086')
            self.mag_cov_055 = self.extract_covariance(value, 'mag covariance_BNO055')

        #Pub
        self.imu_086_publisher = self.create_publisher(Imu, 'imu_086/data', 10)
        self.imu_055_publisher = self.create_publisher(Imu, 'imu_055/data', 10)
        self.mag_086_publisher = self.create_publisher(MagneticField, 'imu_086/mag', 10)
        self.mag_055_publisher = self.create_publisher(MagneticField, 'imu_055/mag', 10)
        #Sub
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.imu_floatarray = self.create_subscription(Float64MultiArray, '/cubemx_imu_data', self.imu_callback, qos_profile)

        self.n = 0
        self.n_max = 10000
        self.euler_list_086 = []
        self.quat_list_086 = []
        self.acc_list_086 = []
        self.gyro_list_086 = []
        self.mag_list_086 = []

        self.euler_list_055 = []
        self.quat_list_055 = []
        self.acc_list_055 = []
        self.gyro_list_055 = []
        self.mag_list_055 = []

    def save_covariance(self, cov_086, cov_055, name : str):

        with open(self.imu_cov_path, 'r') as file:
            value = yaml.safe_load(file) or {}
            # cov_list = cov.tolist()
            value[f'{name} covariance_BNO086'] = cov_086
            value[f'{name} covariance_BNO055'] = cov_055
        with open(self.imu_cov_path, 'w') as file:
            yaml.dump(value, file)

    def imu_callback(self, msg:Float64MultiArray):

        if self.mode == 'save_covariance':
            if self.n < self.n_max:
                self.n+=1

                self.euler_list_086.append([
                    msg.data[12],
                    msg.data[13],
                    msg.data[14]
                ])
                self.quat_list_086.append([
                    msg.data[30], 
                    msg.data[31], 
                    msg.data[32], 
                    msg.data[33]
                ])
                self.acc_list_086.append([
                    msg.data[3],
                    msg.data[4],
                    msg.data[5]
                ])
                self.gyro_list_086.append([
                    msg.data[6],
                    msg.data[7],
                    msg.data[8]
                ])
                self.mag_list_086.append([
                    msg.data[9],
                    msg.data[10],
                    msg.data[11]
                ])


                self.euler_list_055.append([
                    msg.data[27],
                    msg.data[28],
                    msg.data[29]
                ])
                self.quat_list_055.append([
                    msg.data[34], 
                    msg.data[35], 
                    msg.data[36], 
                    msg.data[37]
                ])
                self.acc_list_055.append([
                    msg.data[18],
                    msg.data[19],
                    msg.data[20]
                ])
                self.gyro_list_055.append([
                    msg.data[21],
                    msg.data[22],
                    msg.data[23]
                ])
                self.mag_list_055.append([
                    msg.data[24],
                    msg.data[25],
                    msg.data[26]
                ])

                self.get_logger().info("collect data: " + str(self.n))
                if self.n == self.n_max:
                    self.get_logger().info("Data collection complete. Saving covariance and shutting down.")
                    self.compute_and_save_covariance()
                    # rclpy.shutdown()
                    exit()
            return
        elif self.mode == 'publish_data':   

            # Parse IMU 086 data
            imu_086 = Imu()
            imu_086.header.stamp = self.get_clock().now().to_msg()
            imu_086.header.frame_id = 'imu_086_frame'

            imu_086.orientation.x = msg.data[30]
            imu_086.orientation.y = msg.data[31]
            imu_086.orientation.z = msg.data[32]
            imu_086.orientation.w = msg.data[33]

            imu_086.angular_velocity.x = msg.data[6]
            imu_086.angular_velocity.y = msg.data[7]
            imu_086.angular_velocity.z = msg.data[8]
            imu_086.linear_acceleration.x = msg.data[3]
            imu_086.linear_acceleration.y = msg.data[4]
            imu_086.linear_acceleration.z = msg.data[5]

            imu_086.orientation_covariance = self.euler_cov_086 # Not sure to use quat or euler
            imu_086.linear_acceleration_covariance = self.acc_cov_086
            imu_086.angular_velocity_covariance = self.gyro_cov_086

            # Parse Magnetometer data for IMU 086
            mag_086 = MagneticField()
            mag_086.header.stamp = imu_086.header.stamp
            mag_086.header.frame_id = 'imu_086_frame'
            mag_086.magnetic_field.x = msg.data[9]
            mag_086.magnetic_field.y = msg.data[10]
            mag_086.magnetic_field.z = msg.data[11]
            mag_086.magnetic_field_covariance = self.mag_cov_086

            # ////////////////////////////////////////////////////////

            # Parse IMU 055 data
            imu_055 = Imu()
            imu_055.header.stamp = imu_086.header.stamp
            imu_055.header.frame_id = 'imu_055_frame'

            imu_055.orientation.x = msg.data[34]
            imu_055.orientation.y = msg.data[35]
            imu_055.orientation.z = msg.data[36]
            imu_055.orientation.w = msg.data[37]

            imu_055.angular_velocity.x = msg.data[21]
            imu_055.angular_velocity.y = msg.data[22]
            imu_055.angular_velocity.z = msg.data[23]
            imu_055.linear_acceleration.x = msg.data[18]
            imu_055.linear_acceleration.y = msg.data[19]
            imu_055.linear_acceleration.z = msg.data[20]

            imu_055.orientation_covariance = self.euler_cov_055
            imu_055.linear_acceleration_covariance = self.acc_cov_055
            imu_055.angular_velocity_covariance = self.gyro_cov_055

            # Parse Magnetometer data for IMU 055
            mag_055 = MagneticField()
            mag_055.header.stamp = imu_055.header.stamp
            mag_055.header.frame_id = 'imu_055_frame'
            mag_055.magnetic_field.x = msg.data[24]
            mag_055.magnetic_field.y = msg.data[25]
            mag_055.magnetic_field.z = msg.data[26]
            mag_055.magnetic_field_covariance = self.mag_cov_055

            # Publish IMU 086 data and Magnetometer data
            self.imu_086_publisher.publish(imu_086)
            self.mag_086_publisher.publish(mag_086)
            # Publish IMU 055 dataand Magnetometer data
            self.imu_055_publisher.publish(imu_055)
            self.mag_055_publisher.publish(mag_055)

            # self.get_logger().info('Published IMU and MagneticField messages')

    def compute_and_save_covariance(self):
        # Compute covariance for BNO086
        euler_cov_086 = self.compute_covariance(self.euler_list_086, 9)
        # quat_cov_086 = self.compute_covariance(self.quat_list_086, 16)
        acc_cov_086 = self.compute_covariance(self.acc_list_086, 9)
        gyro_cov_086 = self.compute_covariance(self.gyro_list_086, 9)
        mag_cov_086 = self.compute_covariance(self.mag_list_086, 9)

        # Compute covariance for BNO055
        euler_cov_055 = self.compute_covariance(self.euler_list_055, 9)
        # quat_cov_055 = self.compute_covariance(self.quat_list_055, 16)
        acc_cov_055 = self.compute_covariance(self.acc_list_055, 9)
        gyro_cov_055 = self.compute_covariance(self.gyro_list_055, 9)
        mag_cov_055 = self.compute_covariance(self.mag_list_055, 9)

        self.save_covariance(euler_cov_086, euler_cov_055, 'euler')
        self.save_covariance(acc_cov_086, acc_cov_055, 'acc')
        self.save_covariance(gyro_cov_086, gyro_cov_055, 'gyro')
        self.save_covariance(mag_cov_086, mag_cov_055, 'mag')
            
    def compute_covariance(self, data_list, size):
        """
        Compute a covariance matrix and convert it to a 1D array of specified size.
        If data is insufficient, return a default array of zeros.
        """
        if len(data_list) > 1:
            array = np.array(data_list)
            cov_matrix = np.cov(array.T)  # Compute covariance matrix

            return np.resize(cov_matrix.flatten(), size).tolist()  # Ensure 1D array with `size` elements
        else:
            return [0.0] * size  # Default covariance

    def extract_covariance(self, value, key):
        return value.get(key, [])

   

def main(args=None):
    rclpy.init(args=args)
    node = CarverMessenger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()