import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration

class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor_node')
        self.camera_prefix = self.declare_parameter(
          'camera_prefix', '/front_camera').get_parameter_value().string_value
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(CompressedImage, f'{self.camera_prefix}/compressed', 10)
        self.sub = self.create_subscription(Image, f'{self.camera_prefix}/image_raw', 
                                            self.callback, 10)
        self.bridge = CvBridge()

    def callback(self, data):
        # Convert the raw image to OpenCV format
        self.get_logger().info(f'Received image :{data.header}')
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Compress the image
        result, compressed_image = cv2.imencode('.jpg', cv_image)

        # Compress the image
        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header = data.header
        msg.format = "jpeg"
        msg.data = compressed_image.tobytes()

        self.get_logger().info(f'Image is compressed:{result}')

        # Publish the compressed image
        self.pub.publish(msg)
        cv2.imwrite(f'/tmp/capture/image_{data.header.stamp.sec}.jpg', cv_image)

def main(args=None):
    rclpy.init(args=args)    
    icn = ImageCompressorNode()
    icn.get_logger().info(f'Image Compressor Node Started')
    rclpy.spin(icn)
    icn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()