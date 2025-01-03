#!/usr/bin/python3

from carver_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class FixImuFrame(Node):
    def __init__(self):
        super().__init__('fix_imu_frame_node')
        self.get_logger().info('fix imu frame node has been started')
        self.sub = self.create_subscription(Imu, 'rtabmap/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, 'imu_l515/data', 10)
        self.br = TransformBroadcaster(self)

        # Define individual rotations: Pitch (-90°) and Roll (-90°)
        self.roll_quaternion = quaternion_from_euler(np.pi/2, 0, 0)
        self.pitch_quaternion = quaternion_from_euler(0, -np.pi/2, 0)

    def imu_callback(self, msg:Imu):

        # Original quaternion from IMU
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # Apply rotations in order: First Pitch, Then Roll
        q_roll = quaternion_multiply(self.roll_quaternion, q)  # Rotate pitch
        q_rotated = quaternion_multiply(self.pitch_quaternion, q_roll)  # Then rotate roll

        # Create a new IMU message
        swapped_msg = Imu()
        swapped_msg.header = msg.header

        # Swap orientation axes
        swapped_msg.orientation.x = q_rotated[2]
        swapped_msg.orientation.y = -q_rotated[0]
        swapped_msg.orientation.z = -q_rotated[1]  # Negate Y-axis for consistency
        swapped_msg.orientation.w = q_rotated[3]

        # Swap angular velocity axes
        swapped_msg.angular_velocity.x = msg.angular_velocity.z
        swapped_msg.angular_velocity.y = -msg.angular_velocity.x
        swapped_msg.angular_velocity.z = -msg.angular_velocity.y

        # Swap linear acceleration axes
        swapped_msg.linear_acceleration.x = msg.linear_acceleration.z
        swapped_msg.linear_acceleration.y = -msg.linear_acceleration.x
        swapped_msg.linear_acceleration.z = msg.linear_acceleration.y

        # Create TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'  # Parent frame
        t.child_frame_id = 'camera_imu_optical_frame'  # Child frame
        
        # Translation (optional, set to zero if not needed)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Rotation (from combined quaternion)
        t.transform.rotation.x = q_rotated[2]
        t.transform.rotation.y = -q_rotated[0]
        t.transform.rotation.z = -q_rotated[1]
        t.transform.rotation.w = q_rotated[3]
        
        # Broadcast the transform
        self.br.sendTransform(t)

        # Publish the modified IMU data
        self.pub.publish(swapped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixImuFrame()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
