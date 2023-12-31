import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from os import path
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results
from sensor_msgs.msg import CameraInfo
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from turret_control.turret_tools import create_marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from math import pi

class BoxPredictor(Node):
    def __init__(self):
        super().__init__('drone_box_predictor_node')
        self.camera_prefix = self.declare_parameter(
          'camera_prefix', '/front_camera').get_parameter_value().string_value
        self.camera_link = self.declare_parameter(
          'camera_link', 'camera_link_optical').get_parameter_value().string_value

        #Defualt 90 degrees FOV
        self.camera_hfov = self.declare_parameter(
          'camera_hfov', 1.57).get_parameter_value().double_value
        
        #self.publisher_ = self.create_publisher(Float32MultiArray, 'box_boundaries', 10)
        self.publisher_ = self.create_publisher(Image, f'{self.camera_prefix}/image_with_bounding_boxes', 10)
        
        self.package_share_directory = get_package_share_directory('turret_cv_models')
        self.model_path = path.join(self.package_share_directory, 'cv_models', 'drone-0.2.pt')
        self.yolo_model = YOLO(self.model_path)
        self.sub = self.create_subscription(Image, f'{self.camera_prefix}/image_raw', 
                                            self.callback, 10)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_processed_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        #Subscribe to the camera info topic
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'front_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=10)
        )
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectoryPoint,
                                               '/turret/joint_trajectory_points', qos_profile=qos)
        self.marker_publisher = self.create_publisher(Marker, 'marker', 10)
        self.latest_camera_info:CameraInfo = None
        self.camera_resolution = None

    def camera_info_callback(self, msg):
        # This method is called whenever a new camera info message is received
        # The camera info is in the msg parameter
        self.latest_camera_info = msg
        self.camera_resolution = (msg.width, msg.height)
        #self.degrees_per_pixel = self.camera_hfov / math.sqrt(msg.width**2 + msg.height**2)
        self.degrees_per_pixel = [self.camera_hfov / msg.width, self.camera_hfov / msg.height]
        #self.get_logger().info('Received camera info: %s' % msg)
        
    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img, center_x, center_y = self.predict_box(cv_image)
        augmented_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        augmented_msg.header = msg.header
        self.get_logger().info(f'Augmented Image:{augmented_msg.header}')
        self.publisher_.publish(augmented_msg)
        if center_x is None or center_y is None:
            return
        
        platform_joint_angle, tilt_joint_angle = self.calculate_angle((center_x, center_y))
        #if -0.001 < platform_joint_angle < 0.01 and -0.01 < tilt_joint_angle  < 0.01:
        #    return
        self.get_logger().info(f'Platform Joint Angle: {platform_joint_angle}, Tilt Joint Angle: {tilt_joint_angle}') 
        self.send_goal(platform_joint_angle, tilt_joint_angle)

    
    def get_camera_link_pose(self):
        try:
            # Get the transform from the world frame to the camera link
            transform = self.tf_buffer.lookup_transform('base_link', self.camera_link, rclpy.time.Time())
            
            # The position of the camera link is in the translation field of the transform
            rotation = transform.transform.rotation
            position = transform.transform.translation
            camera_point = Point(x=position.x, y=position.y, z=position.z)
            camera_pose = Pose( position=camera_point, orientation=rotation)
            return camera_pose
        except Exception as e:
            self.get_logger().error('Failed to get camera link position: %s' % e)
            return None
            
    def get_camera_link_position_angles(self, camera_pose:Pose):                        
        quaternion = (camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z, camera_pose.orientation.w)
        
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.get_logger().info(f'Camera Link Position: {math.degrees(roll)}, {math.degrees(pitch)}, {math.degrees(yaw)}')
        return [roll, pitch, yaw]
        
    def predict_box(self, input_data):
        results: Results  = self.yolo_model.track(input_data, persist=True)
        self.get_logger().info(f'Raw prediction : {len(results)}')
        result = results[0]
        #self.get_logger().info(f'Result : {result.tojson()}')        
        if result.boxes.conf.numel() <= 0:
            self.get_logger().info(f'No detection')
            return input_data, None, None

        conf = result.boxes.conf[0].item()   
        self.get_logger().info(f'Boxes Confidence: {conf}')
        if conf < 0.25:
            self.get_logger().info(f'Low confidence')
            return input_data, None, None
        class_id = int(result.boxes.cls[0].item())
        name = result.names[class_id]
        #self.get_logger().info(f'Boxes Class: {name}')
        x, y, w, h = result.boxes.xywh[0]
        #self.get_logger().info(f'Boxes: {x}, {y}, {w}, {h}')
        img = result.plot(conf=True, img=input_data)
        center = (int(x), int(y))
        radius = 3
        color = (0, 255, 0)
        thickness = -1
        cv2.circle(img, center, radius, color, thickness)
        angles = self.calculate_angle(center)
        
        text = f"({math.degrees(angles[0]):.2f},{math.degrees(angles[1]):.2f})"
        position = (int(x), int(y+10))
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        color = (0, 255, 0)  # Green color
        thickness = 1
        cv2.putText(img, text, position, font, fontScale, color, thickness, cv2.LINE_AA)        
        return img, int(x), int(y)  
    
    def calculate_angle(self, object_center):
        if self.camera_resolution is None:
            return 0.0, 0.0
        
        x_delta, y_delta = [self.camera_resolution[0]/2 - object_center[0], self.camera_resolution[1]/2 - object_center[1]]    
        angle_x, angle_y = (round(-x_delta*self.degrees_per_pixel[0], 2), round(-y_delta*self.degrees_per_pixel[1], 2))
        return angle_x, angle_y
        
    def send_goal(self, platform_joint_angle, tilt_joint_angle):   
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        #if current_time - self.last_processed_time < 60:
        #    return
        camera_pose = self.get_camera_link_pose()
        if camera_pose is None:
            return

        camera_orientation = self.get_camera_link_position_angles(camera_pose)
        roll, pitch, yaw = camera_orientation
        target_orientation = quaternion_from_euler(0, tilt_joint_angle, -platform_joint_angle, axes='sxyz')
        quaternion = [camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z, camera_pose.orientation.w]
        new_orientation = quaternion_multiply(quaternion, target_orientation)
        new_quaterion = Quaternion(x=new_orientation[0], y=new_orientation[1], z=new_orientation[2], w=new_orientation[3])
        newcamera_pose = Pose(position=camera_pose.position, orientation=new_quaterion)
        marker = create_marker(self, "base_link", 
                                    pose=newcamera_pose,
                                    scale=(1.0, 0.01, 0.01),
                                                 color=(1.0, 0.0, 0.0, 1.0))
        self.marker_publisher.publish(marker)
        roll, pitch, yaw = euler_from_quaternion(new_orientation)
        joint_points = JointTrajectoryPoint()
        joint_points.positions = [pi+yaw, pitch]
        joint_points.time_from_start = Duration(seconds=0, nanoseconds=500).to_msg()        
        self.joint_trajectory_publisher.publish(joint_points)   
        self.last_sent_goal_time = current_time     

def main(args=None):
    rclpy.init(args=args)
    box_predictor = BoxPredictor()
    rclpy.spin(box_predictor)
    box_predictor.destroy_node()
    rclpy.shutdown()
    
#ros2 run turret_control drone_box_predictor_node
if __name__ == '__main__':
    main()