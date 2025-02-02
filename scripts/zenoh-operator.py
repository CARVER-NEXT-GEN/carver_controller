import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Joy, Image
import zenoh
import json
import ast  # ✅ ใช้ ast เพื่อแปลงจาก Python Dict เป็น JSON
from geometry_msgs.msg import Twist, TransformStamped
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import pygame
# Configure Zenoh to connect through ngrok's URL and port
config = zenoh.Config()
config.insert_json5("connect/endpoints", json.dumps(["tcp/43.208.7.235:7447"]))  # Replace PORT_NUMBER with actual port

# Open Zenoh session
session = zenoh.open(config)

class ZenohLidarBridge(Node):
    def __init__(self):
        super().__init__('zenoh_lidar_bridge')

        self.mode = "JOY" #Select "JOY" or "STEER"
        # QoS ต้องตรงกับ Publisher (TRANSIENT_LOCAL)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1  # ใช้ depth 1 พอเพราะเป็นข้อมูล static
        )

        self.bridge = CvBridge()
        config = zenoh.Config()
        config.insert_json5("connect/endpoints", json.dumps(["tcp/43.208.7.235:7447"]))  # Replace PORT_NUMBER with actual port
        # ตั้งค่า ROS2 Publisher
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/received_image', 10)
        # Publisher สำหรับส่ง ROS tf message (tf2_msgs/TFMessage)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', 10)
        
        if self.mode == "JOY":
            self.joy_subscription = self.create_subscription(
                Joy,
                'joy',        # topic ที่รับข้อมูลจาก joystick
                self.joy_callback,
                10
            )
        if self.mode == "STEER":
            # Initialize pygame and joystick
            pygame.init()
            pygame.joystick.init()
            # Check if any joystick is connected
            if pygame.joystick.get_count() == 0:
                self.get_logger().error("No joystick connected.")
                exit()
            # Get the first joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            self.get_logger().info(f"Connected to joystick: {self.joystick.get_name()}")
            # Timer to publish messages at a regular interval
            self.timer = self.create_timer(0.01, self.steer_loop)

            self.is_backward = 1

        self.robot_description_sub = self.create_subscription(String,
                                                              '/robot_description',
                                                              self.robot_description_callback,
                                                              qos_profile)
        # เชื่อมต่อกับ Zenoh และ Subscribe Topic "laser/scan"
        self.zenoh_session = zenoh.open(config)
        self.zenoh_session.declare_subscriber("laser/scan", self.lidar_callback)
        self.dynamic_sub = self.zenoh_session.declare_subscriber("robot/tf", self.dynamic_callback)
        self.static_sub  = self.zenoh_session.declare_subscriber("robot/tf_static", self.static_callback)

        # Subscribe to the image topic
        self.sub_color_image = self.zenoh_session.declare_subscriber("demo/image", self.color_image_callback)
        self.sub_depth_image = self.zenoh_session.declare_subscriber("demo/depth", self.depth_image_callback)
        self.zenoh_pub_cmd_vel = self.zenoh_session.declare_publisher("cmd_vel")
        # กำหนด mapping ของแกนจาก Xbox controller
        # ค่าเริ่มต้นอาจแตกต่างกันไปตาม driver/OS ที่ใช้
        self.axis_linear = 1    # แกนแนวตั้งของ left stick (อาจเป็นแกนที่ 1)
        self.axis_angular_1 = 4   # แกนแนวนอนของ right stick (อาจเป็นแกนที่ 3)
        self.axis_angular_2 = 5   # แกนแนวนอนของ right stick (อาจเป็นแกนที่ 3)
        
        # ปรับค่าความแรง (scale) ของการเคลื่อนที่
        self.linear_scale = 1.0
        self.angular_scale = 0.5

    def steer_loop(self):
        pygame.event.pump()  # Update the state of the device

        # Prepare Joy message
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = []
        joy_msg.buttons = []

        # Axes (e.g., steering, throttle, brake)
        for i in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(i)
            joy_msg.axes.append(axis_value)

        # Buttons (e.g., paddle shifters, face buttons)
        for i in range(self.joystick.get_numbuttons()):
            button_value = self.joystick.get_button(i)
            joy_msg.buttons.append(button_value)

        # Publish the Joy message
        self.get_logger().info(f"Published Joy message: {joy_msg}")
        if self.joystick.get_button(23) == 1:
            self.is_backward = -1
        else:
            self.is_backward = 1
        # Generate and publish cmd_vel message based on joystick input
        # self.publish_cmd_vel(joy_msg)

        twist_msg = Twist()

        # Map steering (Axis 0) to angular velocity
        steering_axis = joy_msg.axes[0]
        twist_msg.angular.z = -self.angular_scale * steering_axis * 1.0  # Adjust multiplier as needed for turning speed

        # Map throttle (Axis 2) and brake (Axis 3) to linear velocity
        throttle_axis = (1-joy_msg.axes[2]) / 2.0 # Normalized to [0, 1]
        brake_axis = (1 + joy_msg.axes[3]) / 2.0     # Normalized to [0, 1]

        # Calculate linear velocity: throttle increases speed, brake decreases it
        twist_msg.linear.x = self.linear_scale * self.is_backward * throttle_axis# - brake_axis  # Adjust scale as needed for forward/reverse speed

        # แปลงข้อมูล Twist เป็น dictionary
        msg_dict = {
            "linear": {
                "x": twist_msg.linear.x,
                "y": twist_msg.linear.y,
                "z": twist_msg.linear.z
            },
            "angular": {
                "x": twist_msg.angular.x,
                "y": twist_msg.angular.y,
                "z": twist_msg.angular.z
            }
        }
        # แปลงเป็น JSON string
        payload = json.dumps(msg_dict)

        self.zenoh_pub_cmd_vel.put(payload.encode('utf-8'))
        # ส่งข้อมูล Twist ไปยัง topic cmd_vel
        # self.publisher.publish(twist)
        self.get_logger().info(f'Publishing: Linear X = {twist_msg.linear.x:.2f}, Angular Z = {twist_msg.angular.z:.2f}')
        # Publish the cmd_vel message
        # self.publisher_cmd_vel.publish(twist_msg)
        # self.get_logger().info(f"Published Twist message: {twist_msg}")

    def robot_description_callback(self, msg):
        print(msg)


    def dynamic_callback(self, sample):
        """
        Callback เมื่อได้รับ dynamic tf จาก Zenoh (key: robot/tf)
        แปลง JSON payload เป็น tf2_msgs/TFMessage และเผยแพร่ไปที่ /tf
        พร้อมกับอัปเดตเวลาใน header ของแต่ละ transform ด้วยเวลาปัจจุบัน
        """
        try:
            # แปลง payload จาก ZBytes เป็น string แล้ว decode เป็น JSON
            payload = bytes(sample.payload).decode("utf-8")
            data = json.loads(payload)
            self.get_logger().info("Received dynamic tf data from Zenoh.")
            
            tf_msg = TFMessage()
            transforms_list = data.get("transforms", [])
            
            # ดึงเวลาปัจจุบันจาก ROS clock แล้วแปลงเป็น message format
            current_time = self.get_clock().now().to_msg()
            
            for tf_dict in transforms_list:
                ts = self.dict_to_transform(tf_dict)
                # อัปเดตเวลาใน header ให้เป็นเวลาปัจจุบัน
                ts.header.stamp = current_time
                tf_msg.transforms.append(ts)
            
            if tf_msg.transforms:
                self.tf_pub.publish(tf_msg)
                self.get_logger().info(f"Published dynamic TFMessage with {len(tf_msg.transforms)} transforms.")
        except Exception as e:
            self.get_logger().error(f"Error processing dynamic tf data: {e}")

    def static_callback(self, sample):
        """
        Callback เมื่อได้รับ static tf จาก Zenoh (key: robot/tf_static)
        แปลง JSON payload เป็น tf2_msgs/TFMessage และเผยแพร่ไปที่ /tf_static
        """
        try:
            payload = bytes(sample.payload).decode("utf-8")
            data = json.loads(payload)
            self.get_logger().info("Received static tf data from Zenoh.")
            
            tf_msg = TFMessage()
            transforms_list = data.get("transforms", [])
            for tf_dict in transforms_list:
                ts = self.dict_to_transform(tf_dict)
                tf_msg.transforms.append(ts)
            
            if tf_msg.transforms:
                self.tf_static_pub.publish(tf_msg)
                self.get_logger().info(f"Published static TFMessage with {len(tf_msg.transforms)} transforms.")
        except Exception as e:
            self.get_logger().error(f"Error processing static tf data: {e}")

    def dict_to_transform(self, tf_dict):
        """
        แปลง dictionary ที่ประกอบด้วยข้อมูล tf ให้เป็น TransformStamped
        โดยคาดว่า dictionary มี structure ดังนี้:
          {
            "header": {
              "stamp": <float: seconds>,
              "frame_id": <string>
            },
            "child_frame_id": <string>,
            "translation": {"x": float, "y": float, "z": float},
            "rotation": {"x": float, "y": float, "z": float, "w": float}
          }
        """
        ts = TransformStamped()
        
        header = tf_dict.get("header", {})
        stamp_float = header.get("stamp", 0.0)
        sec = int(stamp_float)
        nanosec = int((stamp_float - sec) * 1e9)
        ts.header.stamp.sec = sec
        ts.header.stamp.nanosec = nanosec
        ts.header.frame_id = header.get("frame_id", "")
        
        ts.child_frame_id = tf_dict.get("child_frame_id", "")
        
        translation = tf_dict.get("translation", {})
        ts.transform.translation.x = translation.get("x", 0.0)
        ts.transform.translation.y = translation.get("y", 0.0)
        ts.transform.translation.z = translation.get("z", 0.0)
        
        rotation = tf_dict.get("rotation", {})
        ts.transform.rotation.x = rotation.get("x", 0.0)
        ts.transform.rotation.y = rotation.get("y", 0.0)
        ts.transform.rotation.z = rotation.get("z", 0.0)
        ts.transform.rotation.w = rotation.get("w", 1.0)
        
        return ts

    def color_image_callback(self, sample):
        # Convert the byte array back to an image
        # image_data = np.frombuffer(bytes(sample.payload), dtype=np.uint8)
        # frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        
        # # Display the image
        # cv2.imshow('Received Image', frame)
        # cv2.waitKey(1)  # Refresh display

        try:
            # แปลง byte array จาก sample.payload ให้เป็น numpy array
            image_data = np.frombuffer(bytes(sample.payload), dtype=np.uint8)
            # Decode numpy array เป็นภาพ (cv2 image) โดยใช้ imdecode
            frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().error("Failed to decode image")
                return

            # แปลง cv2 image เป็น ROS Image message (encoding แบบ bgr8)
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            # Publish ROS Image message ไปยัง topic
            self.image_pub.publish(image_msg)
            self.get_logger().info("Published image to 'received_image' topic")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def depth_image_callback(self, sample):

        try:
            # แปลง byte array จาก sample.payload ให้เป็น numpy array
            image_data = np.frombuffer(bytes(sample.payload), dtype=np.uint8)
            # Decode numpy array เป็นภาพ (cv2 image) โดยใช้ imdecode
            frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().error("Failed to decode image")
                return

            # แปลง cv2 image เป็น ROS Image message (encoding แบบ bgr8)
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            # Publish ROS Image message ไปยัง topic
            self.image_pub.publish(image_msg)
            self.get_logger().info("Published image to 'received_image' topic")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def joy_callback(self, msg: Joy):
        """
        Callback ฟังก์ชันที่จะเรียกเมื่อมีข้อมูลจาก joystick เข้ามา
        แปลงค่าแกนที่ได้รับเป็นคำสั่งเคลื่อนที่ (Twist) และส่งไปยัง topic cmd_vel
        """
        twist = Twist()
        
        # ตรวจสอบให้แน่ใจว่ามีข้อมูลแกนเพียงพอ
        if len(msg.axes) > max(self.axis_linear, self.axis_angular_1, self.axis_angular_2):
            twist.linear.x = self.linear_scale * msg.axes[self.axis_linear]
            twist.angular.z = self.angular_scale * (msg.axes[self.axis_angular_1] - msg.axes[self.axis_angular_2])
            
            # แปลงข้อมูล Twist เป็น dictionary
            msg_dict = {
                "linear": {
                    "x": twist.linear.x,
                    "y": twist.linear.y,
                    "z": twist.linear.z
                },
                "angular": {
                    "x": twist.angular.x,
                    "y": twist.angular.y,
                    "z": twist.angular.z
                }
            }
            # แปลงเป็น JSON string
            payload = json.dumps(msg_dict)

            self.zenoh_pub_cmd_vel.put(payload.encode('utf-8'))
            # ส่งข้อมูล Twist ไปยัง topic cmd_vel
            # self.publisher.publish(twist)
            self.get_logger().info(f'Publishing: Linear X = {twist.linear.x:.2f}, Angular Z = {twist.angular.z:.2f}')
        else:
            self.get_logger().warn("จำนวนแกนใน Joy message ไม่เพียงพอ!")

    def lidar_callback(self, sample):
        try:
            raw_data = bytes(sample.payload).decode("utf-8")  # แปลงข้อมูลจาก Zenoh เป็น string
            # print(f"Raw Data from Zenoh: {raw_data}")  # ✅ Debug ข้อมูลที่ได้รับ

            # ✅ ใช้ eval() และกำหนดให้รู้จัก `inf`
            data = eval(raw_data, {"__builtins__": {}}, {"inf": float('inf')})

            # ✅ ตรวจสอบ key "ranges", "angle_min", "angle_max"
            if not all(k in data for k in ["ranges", "angle_min", "angle_max"]):
                self.get_logger().error("Error: Missing keys in received Lidar data")
                return

            # ✅ สร้างข้อความ LaserScan
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser"
            scan_msg.angle_min = float(data["angle_min"])
            scan_msg.angle_max = float(data["angle_max"])
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / len(data["ranges"])
            scan_msg.range_min = 0.1
            scan_msg.range_max = 30.0
            scan_msg.ranges = [float(r) if r != float('inf') else float('nan') for r in data["ranges"]]  # ✅ เปลี่ยน inf → NaN

            # ✅ Publish ไปยัง ROS2
            self.lidar_pub.publish(scan_msg)

            self.get_logger().info("Published Lidar Scan to /scan")
        
        except Exception as e:
            self.get_logger().error(f"Error processing Lidar data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ZenohLidarBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
