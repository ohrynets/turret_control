import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from os import path
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results

class BoxPredictor(Node):
    def __init__(self):
        super().__init__('drone_box_predictor_node')
        self.camera_prefix = self.declare_parameter(
          'camera_prefix', '/front_camera').get_parameter_value().string_value
        
        #self.publisher_ = self.create_publisher(Float32MultiArray, 'box_boundaries', 10)
        self.publisher_ = self.create_publisher(Image, f'{self.camera_prefix}/image_with_bounding_boxes', 10)
        
        self.package_share_directory = get_package_share_directory('turret_cv_models')
        self.model_path = path.join(self.package_share_directory, 'cv_models', 'drone-0.2.pt')
        self.yolo_model = YOLO(self.model_path)
        self.sub = self.create_subscription(Image, f'{self.camera_prefix}/image_raw', 
                                            self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = self.predict_box(cv_image)
        result, compressed_image = cv2.imencode('.jpg', cv_image)

        # Compress the image
        # Create a CompressedImage message
        # compressed_msg = CompressedImage()
        # compressed_msg.header = msg.header
        # compressed_msg.header.stamp = self.get_clock().now().to_msg()
        # compressed_msg.format = "jpeg"
        # compressed_msg.data = compressed_image.tobytes()
        compressed_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        compressed_msg.header = msg.header
        self.get_logger().info(f'Image is compressed:{result}')
        self.get_logger().info(f'Compressed Image:{compressed_msg.header}')
        self.publisher_.publish(compressed_msg)
        
    def predict_box(self, input_data):
        results: Results  = self.yolo_model.track(input_data, persist=True)
        self.get_logger().info(f'Raw prediction : {len(results)}')
        result = results[0]
        self.get_logger().info(f'Result : {result.tojson()}')        
        if result.boxes.conf.numel() <= 0:
            self.get_logger().info(f'No detection')
            return input_data        
        self.get_logger().info(f'Boxes Confidence: {result.boxes.conf[0]}')
        conf = result.boxes.conf[0].item()   
        self.get_logger().info(f'Boxes Confidence: {conf}')
        if conf < 0.25:
            self.get_logger().info(f'Low confidence')
            return input_data
        class_id = int(result.boxes.cls[0].item())
        name = result.names[class_id]
        self.get_logger().info(f'Boxes Class: {name}')
        x, y, w, h = result.boxes.xywh[0]
        self.get_logger().info(f'Boxes Shape: {result.boxes.xywh.shape}')
        self.get_logger().info(f'Boxes: {x}, {y}, {w}, {h}')
        img = result.plot(conf=True, img=input_data)
        center = (int(x), int(y))
        radius = 10
        color = (0, 255, 0)
        thickness = -1
        cv2.circle(img, center, radius, color, thickness)
        return img
        # Postprocess the raw_result to get your box boundaries
        #box_boundaries = result.boxes
        ##self.get_logger().info(f'Raw box boundries shape: {box_boundaries.shape}')
        #self.get_logger().info(f'Raw box boundries: {box_boundaries[0][0]}')
        # Create a ROS2 message and publish the box boundaries
        #msg = Float32MultiArray()
        #msg.data = box_boundaries
        #self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    box_predictor = BoxPredictor()
    rclpy.spin(box_predictor)
    box_predictor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()