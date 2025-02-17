from turret_control.camera_utils import draw_optical_flow_countours, calc_optical_flow, calculate_contours
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from message_filters import Subscriber, Cache
from sensor_msgs.msg import Image
from rclpy.time import Time, Duration

class CalculateOpticalFlowNode(Node):
    def __init__(self):
        super().__init__('calculate_optical_flow_node')
        
        # Define parameter descriptors
        camera_topic_descriptor = ParameterDescriptor(description='The topic to subscribe to for camera images')
        optical_flow_topic_descriptor = ParameterDescriptor(description='The topic to publish the optical flow images')
        optical_flow_boxes_topic_descriptor = ParameterDescriptor(description='The topic to publish the optical flow with bounding boxes')
        cache_size_descriptor = ParameterDescriptor(description='The size of the cache to store frames')
        camera_frame_id_descriptor = ParameterDescriptor(description='The frame id of the camera')
        frame_frequency_descriptor = ParameterDescriptor(description='The frequency at which to process frames (Hz)')
        
        # Declare parameters with descriptions
        self.declare_parameter('camera_topic', 'camera/image_raw', camera_topic_descriptor)
        self.declare_parameter('optical_flow_topic', 'camera/optical_flow', optical_flow_topic_descriptor)
        self.declare_parameter('optical_flow_boxes_topic', 'camera/optical_flow_boxes', optical_flow_boxes_topic_descriptor)
        self.declare_parameter('cache_size', 100, cache_size_descriptor)
        self.declare_parameter('camera_frame_id', 'camera_frame', camera_frame_id_descriptor)
        self.declare_parameter('frame_frequency', 2, frame_frequency_descriptor)
        
        # Retrieve parameter values
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.optical_flow_topic = self.get_parameter('optical_flow_topic').get_parameter_value().string_value
        self.optical_flow_boxes_topic = self.get_parameter('optical_flow_boxes_topic').get_parameter_value().string_value
        self.cache_size = self.get_parameter('cache_size').get_parameter_value().integer_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.frame_frequency = self.get_parameter('frame_frequency').get_parameter_value().integer_value
        
        self.frame_delay = 1.0 / self.frame_frequency
        self.sub = Subscriber(self, Image, self.camera_topic)
        
        self.cache = Cache(self.sub, self.cache_size)
        self.cache.registerCallback(self.cache_callback)
        
        self.timer = self.create_timer(1.0, self.image_callback)            
        self.publisher_flow = self.create_publisher(Image, self.optical_flow_topic, 10)
        self.publisher_flow_boxes = self.create_publisher(Image, self.optical_flow_boxes_topic, 10)
        
        self.bridge = CvBridge()
    
    def cache_callback(self, msg):
        self.get_logger().info(f'Received image with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        
    def image_callback(self):
        latest_time = self.cache.getLastestTime()
        if latest_time is not None:
            start_time = latest_time - Duration(seconds=self.frame_delay)
            # Get messages from the last 5 seconds
            messages = self.cache.getInterval(start_time, latest_time)
            if len(messages) > 1:
                first_msg = messages[0]
                last_msg = messages[-1]
                self.get_logger().info(f'First cached image timestamp: {first_msg.header.stamp.sec}.{first_msg.header.stamp.nanosec}')
                self.get_logger().info(f'Last cached image timestamp: {last_msg.header.stamp.sec}.{last_msg.header.stamp.nanosec}')
                if first_msg is not None and last_msg is not None:
                    first_frame = self.bridge.imgmsg_to_cv2(first_msg, 'bgr8')
                    last_frame = self.bridge.imgmsg_to_cv2(last_msg, 'bgr8')
                    if first_frame is not None and last_frame is not None:
                        optical_flow = calc_optical_flow(first_frame, last_frame)
                        optical_flow_msg = self.generate_optical_flow_message(optical_flow)
                        self.publisher_flow.publish(optical_flow_msg)
                        optical_flow_boxes_msg = self.generate_bounding_boxes_message(last_frame, optical_flow)
                        self.publisher_flow_boxes.publish(optical_flow_boxes_msg)

    def generate_optical_flow_message(self, optical_flow):
        optical_flow_msg = self.bridge.cv2_to_imgmsg(optical_flow, 'bgr8')
        optical_flow_msg.header.stamp = self.get_clock().now().to_msg()
        optical_flow_msg.header.frame_id = self.camera_frame_id
        return optical_flow_msg
    
    def generate_bounding_boxes_message(self, image, optical_flow):
        frame_with_flow_countour, boxes = draw_optical_flow_countours(image, optical_flow, draw_boxes=True)

        frame_with_flow_countour_msg = self.bridge.cv2_to_imgmsg(frame_with_flow_countour, 'bgr8')
        frame_with_flow_countour_msg.header.stamp = self.get_clock().now().to_msg()
        frame_with_flow_countour_msg.header.frame_id = self.camera_frame_id
        return frame_with_flow_countour_msg

def main(args=None):
    rclpy.init(args=args)
    optical_flow_node = CalculateOpticalFlowNode()
    rclpy.spin(optical_flow_node)
    optical_flow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()