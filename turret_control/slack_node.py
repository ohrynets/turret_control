import rclpy
from rclpy.node import Node
import rclpy.waitable
from std_msgs.msg import String
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError
import threading
import time

class SlackROSNode(Node):
    def __init__(self):
        super().__init__('slack_ros_node')
        # Declare parameters with default values
        self.declare_parameter('slack_token', 'xoxb-1234567890')
        self.declare_parameter('slack_channel', 'C064YL34N8G')
        self.declare_parameter('bot_member_id', 'U07BA0R2UQG')
        
        
        # Retrieve parameters
        slack_token = self.get_parameter('slack_token').get_parameter_value().string_value
        slack_channel = self.get_parameter('slack_channel').get_parameter_value().string_value
        bot_member_id = self.get_parameter('bot_member_id').get_parameter_value().string_value
        
        self.slack_client = WebClient(token=slack_token)        
        self.slack_channel = slack_channel
        self.bot_member_id = bot_member_id
        # Tests to see if the token is valid
        auth_test = self.slack_client.auth_test()
        self.bot_member_id = auth_test["user_id"]        
        self.get_logger().info(f"Authentication is sucessfull, app's bot user ID: {self.bot_member_id}")

        self.publisher_ = self.create_publisher(String, 'slack_in', 10)
        self.subscription = self.create_subscription(
            String,
            'slack_out',
            self.slack_out_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Start Slack listener in a separate thread
        threading.Thread(target=self.listen_to_slack, daemon=True).start()    

    def slack_out_callback(self, msg):
        try:
            self.slack_client.chat_postMessage(channel=self.slack_channel, text=msg.data)
        except SlackApiError as e:
            self.get_logger().error(f"Failed to send message to Slack: {e.response['error']}")

    def listen_to_slack(self):
        latest_timestamp = time.time()
        while rclpy.ok():            
            result = self.slack_client.conversations_history(channel=self.slack_channel, limit=100, 
                                                    inclusive=False, oldest=latest_timestamp)
            messages = result['messages']            
            for msg in messages:
                if msg['type'] == 'message' and 'text' in msg:  
                    if msg['user'] == self.bot_member_id:
                        continue
                    self.get_logger().info(f" Message:{msg['text']}")
                    self.publisher_.publish(String(data=msg['text']))
                    latest_timestamp = msg['ts']
            time.sleep(2)            
                    

def main(args=None):
    rclpy.init(args=args)
    slack_ros_node = SlackROSNode()
    rclpy.spin(slack_ros_node)
    slack_ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()