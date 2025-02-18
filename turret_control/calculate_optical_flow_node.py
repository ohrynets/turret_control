from turret_control.camera_utils import draw_optical_flow_countours, calc_optical_flow, calculate_contours
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from message_filters import Subscriber, Cache
from sensor_msgs.msg import Image
from rclpy.time import Time, Duration
from vision_msgs.msg import Detection2DArray, Detection2D, Pose2D, Point2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseWithCovariance

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
        contour_topic_descriptor = ParameterDescriptor(description='This the topic for publishing identified contours')
        
        # Declare parameters with descriptions
        self.declare_parameter('camera_topic', 'camera/image_raw', camera_topic_descriptor)
        self.declare_parameter('optical_flow_topic', 'camera/optical_flow', optical_flow_topic_descriptor)
        self.declare_parameter('optical_flow_boxes_topic', 'camera/optical_flow_boxes', optical_flow_boxes_topic_descriptor)
        self.declare_parameter('cache_size', 100, cache_size_descriptor)
        self.declare_parameter('camera_frame_id', 'camera_frame', camera_frame_id_descriptor)
        self.declare_parameter('frame_frequency', 2, frame_frequency_descriptor)
        self.declare_parameter('contour_topic', 'camera/contours', contour_topic_descriptor)
        
        # Retrieve parameter values
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.optical_flow_topic = self.get_parameter('optical_flow_topic').get_parameter_value().string_value
        self.optical_flow_boxes_topic = self.get_parameter('optical_flow_boxes_topic').get_parameter_value().string_value
        self.cache_size = self.get_parameter('cache_size').get_parameter_value().integer_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.contour_topic = self.get_parameter('contour_topic').get_parameter_value().string_value
        self.frame_frequency = self.get_parameter('frame_frequency').get_parameter_value().integer_value
        
        self.frame_delay = 1.0 / self.frame_frequency
        self.duration = Duration(seconds=self.frame_delay)
        self.sub = Subscriber(self, Image, self.camera_topic)
        
        self.cache = Cache(self.sub, self.cache_size)
        self.cache.registerCallback(self.cache_callback)
        
        self.timer = self.create_timer(1.0, self.image_callback)            
        self.publisher_flow = self.create_publisher(Image, self.optical_flow_topic, 10)
        self.publisher_flow_boxes = self.create_publisher(Image, self.optical_flow_boxes_topic, 10)
        self.publisher_contour = self.create_publisher(Detection2DArray, self.contour_topic, 10)
        
        self.bridge = CvBridge()
    
    def cache_callback(self, msg):
        self.get_logger().info(f'Received image with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        
    def image_callback(self):
        latest_time = self.cache.getLastestTime()
        if latest_time is not None:
            one_sec = Time(seconds=self.frame_delay, clock_type=latest_time.clock_type)
            self.get_logger().info(f'Latest cached image timestamp: {latest_time}')
            if self.duration.nanoseconds > latest_time.nanoseconds:
                return
            start_time = Time(nanoseconds=latest_time.nanoseconds - self.duration.nanoseconds, clock_type=latest_time.clock_type)
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
                        optical_flow_boxes_msg, detection_msgs = self.generate_bounding_boxes_message(last_frame, optical_flow)
                        self.publisher_flow_boxes.publish(optical_flow_boxes_msg)
                        if detection_msgs.detections is not None:
                            self.get_logger().info(f'Publishing detection with timestamp: {detection_msgs}')
                            self.publisher_contour.publish(detection_msgs)

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
        
        detections = self.create_detection2d_from_boxes(boxes)
        return frame_with_flow_countour_msg, detections

    def create_detection2d_from_boxes(self, boxes):
                
        detections_array = Detection2DArray()
        detections_array.header.stamp = self.get_clock().now().to_msg()
        detections_array.header.frame_id = self.camera_frame_id
        detections = []
        if len(boxes) == 0:
            return detections_array

        for box in boxes:
            x, y, w, h = box
            detection = Detection2D()
            detection.header.stamp = self.get_clock().now().to_msg()
            detection.header.frame_id = self.camera_frame_id
        
            # Set the bounding box
            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.position = Point2D()
            bbox.center.position.x = float(x + w/2)
            bbox.center.position.y = float(y + h/2)
            bbox.center.theta = 0.0
            bbox.size_x = float(w)
            bbox.size_y = float(h)
            detection.bbox = bbox
            
            #Add a default hypothesis (modify as needed)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "motion"
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose = PoseWithCovariance()
            detection.results = [hypothesis]
            
            detections.append(detection)
        detections_array.detections = detections
        return detections_array
    
def main(args=None):
    rclpy.init(args=args)
    optical_flow_node = CalculateOpticalFlowNode()
    rclpy.spin(optical_flow_node)
    optical_flow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()