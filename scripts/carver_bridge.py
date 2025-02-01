import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import zenoh
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import zenoh
import json
import cv2
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32, String
import math

class LaserScanPublisher(Node):
    def __init__(self):
        super().__init__('laser_scan_publisher')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Image conversion with CvBridge
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Use 0 for default camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Example width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # Example height
        
        config = zenoh.Config()
        config.insert_json5("connect/endpoints", json.dumps(["tcp/43.208.7.235:7447"]))  # Replace PORT_NUMBER with actual port

        self.subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile)
        # self.sub_robot_description = self.create_subscription(String, 'robot_description', self.robot_description_callback, qos_profile)
        self.zenoh_session = zenoh.open(config)
        
        
        self.publisher = self.zenoh_session.declare_publisher("laser/scan")
        self.zenoh_pub_tf = self.zenoh_session.declare_publisher("robot/tf")
        self.zenoh_pub_tf_static = self.zenoh_session.declare_publisher("robot/tf_static")
        self.zenoh_pub_robot_description = self.zenoh_session.declare_publisher("robot/description")
        
        
        self.create_timer(1/30, self.publish_images)
        self.zenoh_sub_cmd_vel = self.zenoh_session.declare_subscriber("cmd_vel", self.zenoh_cmd_vel)
        
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'received_cmd_vel', 10)
        self.pub_steer_angle = self.create_publisher(Float32, 'steering_angle', 10)
        self.pub_pedal = self.create_publisher(Float32, 'pedal', 10)
        
        # Zenoh topics for color and depth images
        self.color_key_expr = 'demo/image'
        self.depth_key_expr = 'demo/depth'

        self.deadband_linear = 0.1
        self.deadband_angular = 0.1
        
        # กำหนด heartbeat timeout (วินาที) หากไม่มีข้อมูลเข้ามาให้ส่ง Twist ที่มีความเร็วเป็น 0
        self.heartbeat_timeout = 1.0  # หน่วยเป็นวินาที
        
        # กำหนดเวลาที่ได้รับข้อความล่าสุด (เริ่มต้นด้วยเวลาปัจจุบัน)
        self.last_msg_time = self.get_clock().now()
        # สร้าง timer สำหรับตรวจสอบ heartbeat ทุก ๆ 100 ms
        self.heartbeat_timer = self.create_timer(2, self.check_heartbeat)
        # ROS2 subscriptions for color and depth images
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_image_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )
        
        # Subscribe ข้อมูล tf จาก topic /tf (tf2_msgs/TFMessage)
        # Subscribe ROS topic /tf และ /tf_static
        
        
        
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, qos_profile)

        # Buffer สำหรับเก็บข้อความล่าสุด
        self.latest_dynamic_tf_msg = None
        self.latest_static_tf_msg = None

        # Dictionaries สำหรับเก็บ tf ที่ส่งออกไปครั้งล่าสุด (ใช้สำหรับเปรียบเทียบและส่ง keep‐alive)
        self.last_dynamic_transforms = {}
        self.last_static_transforms = {}

        # กำหนด threshold สำหรับ dynamic tf
        self.translation_threshold = 0.01  # 1 cm
        self.rotation_threshold = 0.01     # ค่าประมาณสำหรับ quaternion

        # Timer สำหรับ publish dynamic tf (ทุก 0.1 วินาที)
        self.timer_dynamic = self.create_timer(0.1, self.publish_dynamic_tf)
        # Timer สำหรับ publish static tf (ทุก 1 วินาที)
        self.timer_static = self.create_timer(1.0, self.publish_static_tf)
        
    
    def apply_deadband(self, value, threshold):
        """ ฟังก์ชันตรวจสอบ deadband: ถ้า |value| < threshold ให้ return 0.0 """
        return 0.0 if abs(value) < threshold else value
    
    def zenoh_cmd_vel(self, sample):
        try:
            # อัพเดตเวลาที่ได้รับข้อมูลล่าสุด
            self.last_msg_time = self.get_clock().now()
            # แปลงข้อมูลไบต์เป็นข้อความ (UTF-8) แล้วแปลงเป็น JSON
            payload = bytes(sample.payload).decode("utf-8")
            data = json.loads(payload)
            print("Received data:")
            print("  Linear:", data.get("linear"))
            print("  Angular:", data.get("angular"))
            # สร้าง Twist message
            twist = Twist()
            # ตรวจสอบและกำหนดค่า linear (ค่า default เป็น 0.0 หากไม่มีข้อมูล)
            if "linear" in data:
                twist.linear.x = data["linear"].get("x", 0.0)
                twist.linear.y = data["linear"].get("y", 0.0)
                twist.linear.z = data["linear"].get("z", 0.0)
            # ตรวจสอบและกำหนดค่า angular (ค่า default เป็น 0.0 หากไม่มีข้อมูล)
            if "angular" in data:
                twist.angular.x = data["angular"].get("x", 0.0)
                twist.angular.y = data["angular"].get("y", 0.0)
                twist.angular.z = data["angular"].get("z", 0.0)
            
            twist.linear.x = self.apply_deadband(twist.linear.x, self.deadband_linear)
            twist.linear.y = self.apply_deadband(twist.linear.y, self.deadband_linear)
            twist.linear.z = self.apply_deadband(twist.linear.z, self.deadband_linear)
            twist.angular.x = self.apply_deadband(twist.angular.x, self.deadband_angular)
            twist.angular.y = self.apply_deadband(twist.angular.y, self.deadband_angular)
            twist.angular.z = self.apply_deadband(twist.angular.z, self.deadband_angular)
            
            steer = Float32()
            steer.data = twist.angular.z * 0.5
            pedal = Float32()
            pedal.data = twist.linear.x
            self.pub_steer_angle.publish(steer)
            self.pub_pedal.publish(pedal)
            self.get_logger().info(f"Converted to Twist: Linear(x:{twist.linear.x:.2f}, y:{twist.linear.y:.2f}, z:{twist.linear.z:.2f}) | Angular(x:{twist.angular.x:.2f}, y:{twist.angular.y:.2f}, z:{twist.angular.z:.2f})")
            
            # เผยแพร่ Twist message ไปยัง ROS2 topic
            self.pub_cmd_vel.publish(twist)
        except Exception as e:
            print("Error processing received sample:", e)
    
    def check_heartbeat(self):
        """
        Timer callback ตรวจสอบ heartbeat:
        - ถ้าวินาทีที่ผ่านไปจากข้อความล่าสุดมากกว่า heartbeat_timeout
          ให้ส่ง Twist ที่มีความเร็วเป็น 0 (reset command)
        """
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_msg_time).nanoseconds * 1e-9  # แปลงเป็นวินาที
        if elapsed > self.heartbeat_timeout:
            # สร้าง Twist message ที่มีความเร็วเป็น 0 ทั้งหมด
            twist = Twist()
            # (ค่า default ของ Twist ทุกตัวคือ 0.0 อยู่แล้ว)
            self.pub_cmd_vel.publish(twist)
            self.get_logger().info(
                f"Heartbeat expired (elapsed: {elapsed:.2f}s). Publishing zero Twist."
            )
            # อัพเดต last_msg_time เพื่อป้องกันการส่งซ้ำติดต่อกัน (หรือปรับตามต้องการ)
            self.last_msg_time = current_time
    
    def publish_images(self):
        # Capture a frame from the camera
        # ret, frame = self.cap.read()
        # if not ret:
        #     self.get_logger().error('Failed to capture frame from camera')
        #     return
        
        # # Compress the color image to JPEG to reduce size
        # # _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        
        # _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        
        # # Publish the compressed color image to Zenoh
        # self.zenoh_session.put(self.color_key_expr, buffer.tobytes())
        # self.get_logger().info('Published color image to Zenoh')
        pass
        
    def color_image_callback(self, msg):
        # Convert ROS2 image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg)
        # frame = self.bridge.imgmsg_to_cv2(msg, 'bgr16')
        
        # Compress the color image to JPEG to reduce size
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 25])
        
        # Publish the compressed color image to Zenoh
        self.zenoh_session.put(self.color_key_expr, buffer.tobytes())
        self.get_logger().info('Published color image to Zenoh')

    def depth_image_callback(self, msg):
        # Convert ROS2 depth image to OpenCV format
        # depth_frame = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        
        depth_frame = self.bridge.imgmsg_to_cv2(msg)
        
        # Compress the depth image using PNG (better for depth images)
        _, buffer = cv2.imencode('.jpg', depth_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        
        # Publish the compressed depth image to Zenoh
        # self.zenoh_session.put(self.depth_key_expr, buffer.tobytes())
        self.get_logger().info('Published depth image to Zenoh')
        
    def laser_callback(self, msg):
        scan_data = {
            "ranges": list(msg.ranges),
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max
        }
        self.publisher.put(str(scan_data))

    def tf_callback(self, msg: TFMessage):
        """ รับข้อมูล dynamic tf จาก topic /tf """
        self.latest_dynamic_tf_msg = msg

    def tf_static_callback(self, msg: TFMessage):
        """ รับข้อมูล static tf จาก topic /tf_static """
        self.latest_static_tf_msg = msg

    def transform_to_dict(self, t: TransformStamped):
        """ แปลง TransformStamped เป็น dictionary """
        return {
            "header": {
                "stamp": t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                "frame_id": t.header.frame_id
            },
            "child_frame_id": t.child_frame_id,
            "translation": {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "z": t.transform.translation.z
            },
            "rotation": {
                "x": t.transform.rotation.x,
                "y": t.transform.rotation.y,
                "z": t.transform.rotation.z,
                "w": t.transform.rotation.w
            }
        }

    def is_significantly_different(self, new_tf, old_tf):
        """ เปรียบเทียบการเปลี่ยนแปลงระหว่าง tf ใหม่และ tf เก่า """
        t_new = new_tf["translation"]
        t_old = old_tf["translation"]
        diff_t = math.sqrt(
            (t_new["x"] - t_old["x"])**2 +
            (t_new["y"] - t_old["y"])**2 +
            (t_new["z"] - t_old["z"])**2
        )
        if diff_t > self.translation_threshold:
            return True

        r_new = new_tf["rotation"]
        r_old = old_tf["rotation"]
        diff_r = math.sqrt(
            (r_new["x"] - r_old["x"])**2 +
            (r_new["y"] - r_old["y"])**2 +
            (r_new["z"] - r_old["z"])**2 +
            (r_new["w"] - r_old["w"])**2
        )
        if diff_r > self.rotation_threshold:
            return True

        return False

    def publish_dynamic_tf(self):
        """
        Timer callback สำหรับ publish dynamic tf ผ่าน Zenoh
        - ถ้ามีข้อมูลใหม่จาก /tf จะตรวจสอบและส่งเฉพาะ tf ที่เปลี่ยนแปลง
        - ถ้าไม่มีข้อมูลใหม่ จะส่ง keep‐alive ด้วย tf ที่ส่งออกไปแล้ว (หรืออาร์เรย์ว่าง)
        """
        updated_transforms = []

        if self.latest_dynamic_tf_msg is not None:
            # มีข้อมูลใหม่เข้ามา
            for transform in self.latest_dynamic_tf_msg.transforms:
                tf_dict = self.transform_to_dict(transform)
                child_frame = tf_dict["child_frame_id"]
                if child_frame not in self.last_dynamic_transforms:
                    updated_transforms.append(tf_dict)
                    self.last_dynamic_transforms[child_frame] = tf_dict
                else:
                    old_tf = self.last_dynamic_transforms[child_frame]
                    if self.is_significantly_different(tf_dict, old_tf):
                        updated_transforms.append(tf_dict)
                        self.last_dynamic_transforms[child_frame] = tf_dict
                    else:
                        updated_transforms = list(self.last_dynamic_transforms.values())
            # ล้าง buffer หลังจากประมวลผล tf ใหม่แล้ว
            self.latest_dynamic_tf_msg = None
        else:
            # ไม่มีข้อมูลใหม่ => ส่ง keep‐alive ด้วย tf ที่ส่งออกไปแล้ว (ถ้ามี)
            if self.last_dynamic_transforms:
                updated_transforms = list(self.last_dynamic_transforms.values())
            else:
                updated_transforms = []  # กรณีไม่มี tf เลย

        # สร้าง payload แม้จะไม่มีรายการ tf ก็ตาม
        payload = {
            "transforms": updated_transforms,
            "timestamp": self.get_clock().now().nanoseconds * 1e-9  # timestamp ในรูปแบบวินาที (float)
        }
        payload_str = json.dumps(payload)

        if self.zenoh_session is not None:
            try:
                self.zenoh_pub_tf.put(payload_str.encode('utf-8'))
                self.get_logger().info(f"Published {len(updated_transforms)} dynamic tf transforms via Zenoh.")
            except Exception as e:
                self.get_logger().error(f"Failed to publish dynamic tf data: {e}")
        else:
            self.get_logger().warn("Zenoh session not available for dynamic tf.")

    def publish_static_tf(self):
        """
        Timer callback สำหรับ publish static tf ผ่าน Zenoh
        - เมื่อได้รับข้อมูลใหม่จาก /tf_static จะอัพเดทข้อมูล static transforms
        - ถ้าไม่มีข้อมูลใหม่ ก็จะส่ง keep‐alive ด้วย static tf ที่เคยได้รับ
        """
        updated_transforms = []

        if self.latest_static_tf_msg is not None:
            for transform in self.latest_static_tf_msg.transforms:
                tf_dict = self.transform_to_dict(transform)
                child_frame = tf_dict["child_frame_id"]
                # สำหรับ static tf เราอัพเดททุกครั้งที่ได้รับ
                self.last_static_transforms[child_frame] = tf_dict
            self.latest_static_tf_msg = None

        if self.last_static_transforms:
            updated_transforms = list(self.last_static_transforms.values())

        if updated_transforms:
            payload = {
                "transforms": updated_transforms,
                "timestamp": self.get_clock().now().nanoseconds * 1e-9
            }
            payload_str = json.dumps(payload)
            if self.zenoh_session is not None:
                try:
                    self.zenoh_pub_tf_static.put(payload_str.encode('utf-8'))
                    self.get_logger().info(f"Published {len(updated_transforms)} static tf transforms via Zenoh.")
                except Exception as e:
                    self.get_logger().error(f"Failed to publish static tf data: {e}")
            else:
                self.get_logger().warn("Zenoh session not available for static tf.")

    def transform_to_dict(self, t):
        """
        แปลง TransformStamped (t) เป็น dictionary
        """
        return {
            "header": {
                "stamp": t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                "frame_id": t.header.frame_id
            },
            "child_frame_id": t.child_frame_id,
            "translation": {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "z": t.transform.translation.z
            },
            "rotation": {
                "x": t.transform.rotation.x,
                "y": t.transform.rotation.y,
                "z": t.transform.rotation.z,
                "w": t.transform.rotation.w
            }
        }
    
    def is_significantly_different(self, new_tf, old_tf):
        """
        เปรียบเทียบ tf ใหม่กับ tf เก่า หากมีการเปลี่ยนแปลงเกิน threshold ให้คืนค่า True
        """
        # เปรียบเทียบการแปลตำแหน่ง (translation)
        t_new = new_tf["translation"]
        t_old = old_tf["translation"]
        diff_t = math.sqrt(
            (t_new["x"] - t_old["x"])**2 +
            (t_new["y"] - t_old["y"])**2 +
            (t_new["z"] - t_old["z"])**2
        )
        if diff_t > self.translation_threshold:
            return True
        
        # เปรียบเทียบการหมุน (rotation)
        r_new = new_tf["rotation"]
        r_old = old_tf["rotation"]
        diff_r = math.sqrt(
            (r_new["x"] - r_old["x"])**2 +
            (r_new["y"] - r_old["y"])**2 +
            (r_new["z"] - r_old["z"])**2 +
            (r_new["w"] - r_old["w"])**2
        )
        if diff_r > self.rotation_threshold:
            return True
        
        return False

    def publish_tf(self):
        """
        Timer callback สำหรับ publish tf ผ่าน Zenoh
        - หากมี tf ใหม่ที่เปลี่ยนแปลง จะส่งออกทันที
        - หากไม่มี tf ใหม่ จะส่งข้อมูล tf ล่าสุดออกไปเป็น keep-alive
        """
        updated_transforms = []
        
        if self.latest_tf_msg is not None:
            # มีข้อมูล tf ใหม่เข้ามา
            for transform in self.latest_tf_msg.transforms:
                tf_dict = self.transform_to_dict(transform)
                child_frame = tf_dict["child_frame_id"]
                
                # ถ้ายังไม่เคยส่ง transform นี้ หรือมีการเปลี่ยนแปลงเกิน threshold
                if child_frame not in self.last_transforms:
                    updated_transforms.append(tf_dict)
                    self.last_transforms[child_frame] = tf_dict
                else:
                    old_tf = self.last_transforms[child_frame]
                    if self.is_significantly_different(tf_dict, old_tf):
                        updated_transforms.append(tf_dict)
                        self.last_transforms[child_frame] = tf_dict
                    # ถ้าไม่เปลี่ยนแปลงมากพอ ไม่ต้องส่ง tf ตัวนี้ใหม่
            # ล้าง buffer หลังจากประมวลผล tf ใหม่แล้ว
            self.latest_tf_msg = None
        else:
            # ไม่มี tf ใหม่เข้ามา แต่ถ้ามีข้อมูล tf ที่เคยส่งออกไปแล้ว
            # ส่งข้อมูล tf เดิมออกไปเป็น keep-alive
            if self.last_transforms:
                updated_transforms = list(self.last_transforms.values())
        
        # ส่งออก payload หากมี transform ที่จะส่ง
        if updated_transforms:
            payload = {
                "transforms": updated_transforms,
                "timestamp": self.get_clock().now().nanoseconds * 1e-9  # timestamp ในรูปแบบ float วินาที
            }
            payload_str = json.dumps(payload)
            if self.zenoh_session is not None:
                try:
                    # ส่งข้อมูลออกไปในรูปแบบ byte
                    self.zenoh_pub_tf.put(payload_str.encode('utf-8'))
                    self.get_logger().info(f"Published {len(updated_transforms)} tf transforms via Zenoh.")
                except Exception as e:
                    self.get_logger().error(f"Failed to publish tf data: {e}")
            else:
                self.get_logger().warn("Zenoh session not available.")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
