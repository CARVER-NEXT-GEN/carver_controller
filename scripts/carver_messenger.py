#!/usr/bin/python3

from carver_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy

import numpy as np


class CarverMessenger(Node):
    def __init__(self):
        super().__init__('carver_messenger_node')

        self.get_logger().info('carver messenger node has been start')

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
        self.n_max = 100
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

    def imu_callback(self, msg:Float64MultiArray):

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
                self.get_logger().info("collect data finish, start to publish data from imu")
        else:
            # Find covariance in BNO086
            def compute_covariance(data_list, size=9):
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
            # Compute covariance for BNO086
            euler_cov_086 = compute_covariance(self.euler_list_086, size=9)
            quat_cov_086 = compute_covariance(self.quat_list_086, size=9)
            acc_cov_086 = compute_covariance(self.acc_list_086, size=9)
            gyro_cov_086 = compute_covariance(self.gyro_list_086, size=9)
            mag_cov_086 = compute_covariance(self.mag_list_086, size=9)

            # Compute covariance for BNO055
            euler_cov_055 = compute_covariance(self.euler_list_055, size=9)
            quat_cov_055 = compute_covariance(self.quat_list_055, size=9)
            acc_cov_055 = compute_covariance(self.acc_list_055, size=9)
            gyro_cov_055 = compute_covariance(self.gyro_list_055, size=9)
            mag_cov_055 = compute_covariance(self.mag_list_055, size=9)

            # ***Old solution***
            # orien_array_086 = np.array(self.quat_list_086)
            # orein_cov_086 = np.absolute(np.cov(orien_array_086.T))
            # # Find covariance in BNO055
            # orien_array_055 = np.array(self.quat_list_055)
            # orein_cov_055 = np.absolute(np.cov(orien_array_055.T))


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

            imu_086.orientation_covariance = euler_cov_086 # Not sure to use quat or euler
            imu_086.linear_acceleration_covariance = acc_cov_086
            imu_086.angular_velocity_covariance = gyro_cov_086

            # Parse Magnetometer data for IMU 086
            mag_086 = MagneticField()
            mag_086.header.stamp = imu_086.header.stamp
            mag_086.header.frame_id = 'imu_086_frame'
            mag_086.magnetic_field.x = msg.data[9]
            mag_086.magnetic_field.y = msg.data[10]
            mag_086.magnetic_field.z = msg.data[11]
            mag_086.magnetic_field_covariance = mag_cov_086

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

            imu_055.orientation_covariance = euler_cov_055 # Not sure to use quat or euler
            imu_055.linear_acceleration_covariance = acc_cov_055
            imu_055.angular_velocity_covariance = gyro_cov_055

            # Parse Magnetometer data for IMU 055
            mag_055 = MagneticField()
            mag_055.header.stamp = imu_055.header.stamp
            mag_055.header.frame_id = 'imu_055_frame'
            mag_055.magnetic_field.x = msg.data[24]
            mag_055.magnetic_field.y = msg.data[25]
            mag_055.magnetic_field.z = msg.data[26]
            mag_055.magnetic_field_covariance = mag_cov_055

            # Publish IMU 086 data and Magnetometer data
            self.imu_086_publisher.publish(imu_086)
            self.mag_086_publisher.publish(mag_086)
            # Publish IMU 055 dataand Magnetometer data
            self.imu_055_publisher.publish(imu_055)
            self.mag_055_publisher.publish(mag_055)

            # self.get_logger().info('Published IMU and MagneticField messages')

   

def main(args=None):
    rclpy.init(args=args)
    node = CarverMessenger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()