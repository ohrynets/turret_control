import rclpy
from rclpy.node import Node
import rclpy.waitable
from std_msgs.msg import String
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError
import threading
import time
from datetime import datetime, timedelta
from turret_interfaces.msg import SlackMessage
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
import cv2
import urllib.request
import numpy as np
from cv_bridge import CvBridge

class SlackNode(Node):
    def __init__(self):
        super().__init__('slack_node')
        self.bridge = CvBridge()
        # Declare parameters with default values
        self.declare_parameter('slack_token', 'xoxb-1234567890')
        self.declare_parameter('slack_channel', 'C064YL34N8G')
        self.declare_parameter('bot_member_id', 'U07BA0R2UQG')
        self.declare_parameter('slack_input_topic', 'slack_in')
        self.declare_parameter('slack_output_topic', 'slack_out')
        
        # Retrieve parameters
        self.slack_token = self.get_parameter('slack_token').get_parameter_value().string_value
        slack_channel = self.get_parameter('slack_channel').get_parameter_value().string_value
        bot_member_id = self.get_parameter('bot_member_id').get_parameter_value().string_value
        self.slack_input_topic = self.get_parameter('slack_input_topic').get_parameter_value().string_value
        self.slack_output_topic = self.get_parameter('slack_output_topic').get_parameter_value().string_value
        
        self.slack_client = WebClient(token=self.slack_token)        
        self.slack_channel = slack_channel
        self.bot_member_id = bot_member_id
        # Tests to see if the token is valid
        auth_test = self.slack_client.auth_test()
        self.bot_member_id = auth_test["user_id"]        
        self.get_logger().info(f"Authentication is successful, app's bot user ID: {self.bot_member_id}")

        custom_qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=1)
        )
        
        self.publisher_ = self.create_publisher(SlackMessage, self.slack_input_topic, 10)
        self.subscription = self.create_subscription(
            SlackMessage,
            self.slack_output_topic,
            self.slack_out_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Start Slack listener in a separate thread
        threading.Thread(target=self.listen_to_slack, daemon=True).start()    

    def slack_out_callback(self, msg):
        try:
            self.slack_client.chat_postMessage(channel=self.slack_channel, text=msg.text)
        except SlackApiError as e:
            self.get_logger().error(f"Failed to send message to Slack: {e.response['error']}")

    def resize_image_with_padding(self, cv_image, target_width=1050, target_height=1610, padding_color=(0, 0, 0)):
        # Calculate the aspect ratio of the original image
        h, w = cv_image.shape[:2]
        aspect_ratio = w / h

        # Calculate the new width and height to maintain the aspect ratio
        if w / h > target_width / target_height:
            new_width = target_width
            new_height = round(new_width / aspect_ratio)
        else:
            new_height = target_height
            new_width = round(new_height * aspect_ratio)

        # Resize the image
        resized_image = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)

        # Calculate padding
        top = (target_height - new_height) // 2
        bottom = target_height - new_height - top
        left = (target_width - new_width) // 2
        right = target_width - new_width - left

        # Add padding to the resized image
        padded_image = cv2.copyMakeBorder(resized_image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=padding_color)

        return padded_image

    def download_image_with_opencv(self, url):
        # Use urllib to download the image
        # Create a request object
        req = urllib.request.Request(url)
        # Add the Authorization header with the Bearer token
        req.add_header('Authorization', f'Bearer {self.slack_token}')
        
        resp = urllib.request.urlopen(req)
        self.get_logger().info(f"Response Code:{resp.getcode()} for URL:{resp.geturl()}")
        image = np.asarray(bytearray(resp.read()), dtype="uint8")        
        # Decode the image as a matrix
        cv_image = cv2.imdecode(image, cv2.IMREAD_COLOR)                                
        return cv_image
        
    def retreive_image(self, file):
        cv_image = self.download_image_with_opencv(file['url_private'])
        # Resize the image with padding 1344x336
        cv_image_resized = self.resize_image_with_padding(cv_image, 336, 1344)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image_resized, encoding='bgr8')        
        return ros_image
                                    
    def listen_to_slack(self):
        self.get_logger().info("Listening to Slack messages")
        latest_timestamp = datetime.now() - timedelta(seconds=1)
        latest_timestamp = latest_timestamp.timestamp()
        while rclpy.ok():            
            result = self.slack_client.conversations_history(channel=self.slack_channel, limit=100, 
                                                    inclusive=False, oldest=str(latest_timestamp))
            messages = result['messages']            
            #self.get_logger().info(f"Received messages:{result}")
            for msg in messages:
                if msg['type'] == 'message' and 'text' in msg:  
                    if msg['user'] == self.bot_member_id:
                        continue
                    self.get_logger().info(f" Message:{msg['text']}")
                    slack_msg = SlackMessage()
                    slack_msg.text = msg['text']                    
                    latest_timestamp = msg['ts']
                    slack_msg.is_screenshot = False                    
                    if 'files' in msg:
                        if msg['user'] == self.bot_member_id:
                            continue
                        file_info = msg["files"][0]                        
                        slack_msg.screenshot = self.retreive_image(file_info)   
                        slack_msg.is_screenshot = True
                    self.get_logger().info(f"Publishing message {slack_msg.text} to {self.slack_input_topic}")
                    self.publisher_.publish(slack_msg)
                    
                    
            time.sleep(2)            
                    

def main(args=None):
    rclpy.init(args=args)
    slack_ros_node = SlackNode()
    rclpy.spin(slack_ros_node)
    slack_ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()