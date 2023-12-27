import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TargetOnImageNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/front_camera/image_with_target', 10)
        self.subscription = self.create_subscription(
            Image,
            '/front_camera/image_with_bounding_boxes',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg: Image):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Draw a horizontal and vertical line on the image
        height, width, _ = cv_image.shape
        mid_x, mid_y = width // 2, height // 2
        start_x, end_x = width // 3, (2 * width) // 3
        start_y, end_y = height // 3, (2 * height) // 3
        cv2.line(cv_image, (start_x, mid_y), (end_x, mid_y), (0, 255, 0), 5)  # Horizontal line
        cv2.line(cv_image, (mid_x, start_y), (mid_x, end_y), (0, 255, 0), 5)  # Vertical line

        # Convert the CV Image back to a ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        img_msg.header = msg.header
        # Publish the processed image
        self.publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetOnImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()