#!/usr/bin/python3

from carver_controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, MagneticField

'''
    // IMU_086 acceleration
    f64array_msg.data.data[0] = IMU_086_FRTOS.acceleration.x;
    f64array_msg.data.data[1] = IMU_086_FRTOS.acceleration.y;
    f64array_msg.data.data[2] = IMU_086_FRTOS.acceleration.z;

    // IMU_086 linear acceleration
    f64array_msg.data.data[3] = IMU_086_FRTOS.linear_acceleration.x;
    f64array_msg.data.data[4] = IMU_086_FRTOS.linear_acceleration.y;
    f64array_msg.data.data[5] = IMU_086_FRTOS.linear_acceleration.z;

    // IMU_086 angular velocity
    f64array_msg.data.data[6] = IMU_086_FRTOS.angular_velocity.x;
    f64array_msg.data.data[7] = IMU_086_FRTOS.angular_velocity.y;
    f64array_msg.data.data[8] = IMU_086_FRTOS.angular_velocity.z;

    // IMU_086 magnetometer
    f64array_msg.data.data[9]  = IMU_086_FRTOS.magnetometer.x;
    f64array_msg.data.data[10] = IMU_086_FRTOS.magnetometer.y;
    f64array_msg.data.data[11] = IMU_086_FRTOS.magnetometer.z;

    // IMU_086 euler angles
    f64array_msg.data.data[12] = IMU_086_FRTOS.euler_angle.roll;
    f64array_msg.data.data[13] = IMU_086_FRTOS.euler_angle.pitch;
    f64array_msg.data.data[14] = IMU_086_FRTOS.euler_angle.yaw;


//     IMU_055 acceleration
    f64array_msg.data.data[15] = IMU_055_FRTOS.accel.x;
    f64array_msg.data.data[16] = IMU_055_FRTOS.accel.y;
    f64array_msg.data.data[17] = IMU_055_FRTOS.accel.z;

    // IMU_055 linear acceleration
    f64array_msg.data.data[18] = IMU_055_FRTOS.lin_acc.x;
    f64array_msg.data.data[19] = IMU_055_FRTOS.lin_acc.y;
    f64array_msg.data.data[20] = IMU_055_FRTOS.lin_acc.z;

    // IMU_055 gyro (angular velocity)
    f64array_msg.data.data[21] = IMU_055_FRTOS.gyro.x;
    f64array_msg.data.data[22] = IMU_055_FRTOS.gyro.y;
    f64array_msg.data.data[23] = IMU_055_FRTOS.gyro.z;

    // IMU_055 magnetometer
    f64array_msg.data.data[24] = IMU_055_FRTOS.mag.x;
    f64array_msg.data.data[25] = IMU_055_FRTOS.mag.y;
    f64array_msg.data.data[26] = IMU_055_FRTOS.mag.z;

    // IMU_055 euler angles
    f64array_msg.data.data[27] = IMU_055_FRTOS.euler.roll;
    f64array_msg.data.data[28] = IMU_055_FRTOS.euler.pitch;
    f64array_msg.data.data[29] = IMU_055_FRTOS.euler.yaw;

    //
    f64array_msg.data.data[30] = IMU_086_FRTOS.quaternion.i;
    f64array_msg.data.data[31] = IMU_086_FRTOS.quaternion.j;
    f64array_msg.data.data[32] = IMU_086_FRTOS.quaternion.k;
    f64array_msg.data.data[33] = IMU_086_FRTOS.quaternion.w;
    //
    f64array_msg.data.data[34] = IMU_055_FRTOS.quat.x;
    f64array_msg.data.data[35] = IMU_055_FRTOS.quat.y;
    f64array_msg.data.data[36] = IMU_055_FRTOS.quat.z;
    f64array_msg.data.data[37] = IMU_055_FRTOS.quat.w;
'''

class CarverMessenger(Node):
    def __init__(self):
        super().__init__('carver_messenger_node')
        #Pub
        self.odom086_pub = self.create_publisher(Odometry, '/odom086', 10)
        self.odom055_pub = self.create_publisher(Odometry, '/odom055', 10)
        self.imu055_pub = self.create_publisher(Imu, '/imu086', 10)
        self.imu055_pub = self.create_publisher(Imu, '/imu055', 10)
        self.mag086_pub = self.create_publisher(MagneticField, '/mag086', 10)
        self.mag055_pub = self.create_publisher(MagneticField, '/mag055', 10)
        #Sub
        self.imu_floatarray = self.create_subscription(Float64MultiArray, '/cubemx_imu_data', self.imu_floatarray_callback, 10)



    def imu_floatarray_callback(self, msg:Float64MultiArray):
        self.imu086_msg = Imu()

        # orientation BNO086
        self.imu086_msg.orientation.x = msg.data[30]
        self.imu086_msg.orientation.y = msg.data[31]
        self.imu086_msg.orientation.z = msg.data[32]
        self.imu086_msg.orientation.w = msg.data[33]

        # float64[9]
        self.imu086_msg.orientation_covariance

        # angular_velocity BNO086
        self.imu086_msg.angular_velocity.x = msg.data[6]
        self.imu086_msg.angular_velocity.y = msg.data[7]
        self.imu086_msg.angular_velocity.z = msg.data[8]

        # float64[9]
        self.imu086_msg.angular_velocity_covariance

        # linear_acceleration BNO086
        self.imu086_msg.linear_acceleration.x = msg.data[3]
        self.imu086_msg.linear_acceleration.y = msg.data[4]
        self.imu086_msg.linear_acceleration.z = msg.data[5]

        # float64[9]
        self.imu086_msg.linear_acceleration_covariance

# ////////////////////////////////////////////////////////////////////////////////

        self.imu055_msg = Imu()

        # orientation BNO055
        self.imu055_msg.orientation.x = msg.data[34]
        self.imu055_msg.orientation.y = msg.data[35]
        self.imu055_msg.orientation.z = msg.data[36]
        # self.imu055_msg.orientation.w = msg.data[37]

        # float64[9]
        self.imu055_msg.orientation_covariance

        # angular_velocity BNO055
        self.imu055_msg.angular_velocity.x = msg.data[21]
        self.imu055_msg.angular_velocity.y = msg.data[22]
        self.imu055_msg.angular_velocity.z = msg.data[23]

        # float64[9]
        self.imu055_msg.angular_velocity_covariance

        # linear_acceleration BNO086
        self.imu055_msg.linear_acceleration.x = msg.data[18]
        self.imu055_msg.linear_acceleration.y = msg.data[19]
        self.imu055_msg.linear_acceleration.z = msg.data[20]

        # float64[9]
        self.imu055_msg.linear_acceleration_covariance

# ////////////////////////////////////////////////////////////////////////////////

        self.mag055_msg = MagneticField()

        self.mag055_msg.magnetic_field.x
        self.mag055_msg.magnetic_field.y
        self.mag055_msg.magnetic_field.z

        


        self.acceleration = msg.data
    

    

def main(args=None):
    rclpy.init(args=args)
    node = CarverMessenger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()