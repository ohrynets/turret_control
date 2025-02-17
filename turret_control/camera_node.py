from turret_control.camera_utils import init_camera, read_camera
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(LifecycleNode):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('refresh_rate', 10.0)
        self.declare_parameter('camera_width', 1600)
        self.declare_parameter('camera_height', 1200)
        
        self.bridge = CvBridge()
        self.camera = None
        self.timer = None
        self.publisher_ = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().double_value
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value

        self.timer_period = 1.0 / self.refresh_rate
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        self.camera = init_camera(self.camera_id, self.camera_width, self.camera_height)
        self.get_logger().info(f'Activating camera... {self.camera} with id {self.camera_id}')
        self.publisher_ = self.create_publisher(Image, self.camera_topic, 10)
        #self.publisher_.on_activate()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        if self.timer is not None:
            self.timer.cancel()
        self.publisher_.on_deactivate()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        if self.camera is not None:
            self.camera.release()
            self.camera = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        if self.camera is not None:
            self.camera.release()
            self.camera = None
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Error...')
        if self.camera is not None:
            self.camera.release()
            self.camera = None
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        
        if self.camera is not None:
            frame = read_camera(self.camera)
            if frame is not None:
                self.get_logger().info(f'Reading from camera... {self.camera} with id {self.camera_id}')
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)    
    camera_node.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
    main()