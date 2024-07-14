#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import time
import rclpy
from rclpy.node import Node
from llama_ros.llama_client_node import LlamaClientNode
from llama_msgs.action import GenerateResponse
from turret_interfaces.msg import SlackMessage
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge

class LlamaNode(Node):
    
    def prompt_received(self, msg:SlackMessage):
        self.get_logger().info(f"Prompt received ...{msg.text}")       
        self.send_prompt(msg)
    
    def __init__(self) -> None:
        super().__init__("llama_node")
        self.bridge = CvBridge()
        
        self.declare_parameter('slack_input_topic', 'slack_in')
        self.declare_parameter('slack_output_topic', 'slack_out')
        self.declare_parameter('llm_name', 'llava')        

        self.slack_input_topic = self.get_parameter('slack_input_topic').get_parameter_value().string_value
        self.slack_output_topic = self.get_parameter('slack_output_topic').get_parameter_value().string_value
        self.llm_name = self.get_parameter('llm_name').get_parameter_value().string_value

        custom_qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=1)
        )
        
        self.subscription = self.create_subscription(
            SlackMessage,
            self.slack_input_topic,
            self.prompt_received,
            10)
        
        self.publisher_ = self.create_publisher(SlackMessage, self.slack_output_topic, 10)

        self.tokens = 0
        self.initial_time = -1
        self.eval_time = -1

        self._llama_client = LlamaClientNode.get_instance(self.llm_name)                
        self.get_logger().info(f"Node started. Reading messages from: {self.slack_input_topic} ...")

    def text_cb(self, feedback) -> None:

        if self.eval_time < 0:
            self.eval_time = time.time()

        self.tokens += 1
        print(feedback.feedback.partial_response.text, end="", flush=True)

    def send_prompt(self, prompt_msg:SlackMessage) -> None:

        goal = GenerateResponse.Goal()
        goal.prompt = prompt_msg.text
        goal.sampling_config.temp = 0.2
        goal.sampling_config.penalty_last_n = 8
        goal.reset = True

        self.initial_time = time.time()
        if prompt_msg.is_screenshot:
            goal.image = prompt_msg.screenshot
        self.get_logger().info(f"Received prompt: {prompt_msg.text} image:{prompt_msg.is_screenshot}")
        result, status = self._llama_client.generate_response(goal, self.text_cb)
        if status != GoalStatus.STATUS_SUCCEEDED:
            return ""
        
        slack_msg = SlackMessage()
        slack_msg.text = result.response.text
        self.publisher_.publish(slack_msg)        

        self.get_logger().info("END")
        end_time = time.time()
        self.get_logger().info(
            f"Time to eval: {self.eval_time - self.initial_time} s")
        self.get_logger().info(
            f"Prediction speed: {self.tokens / (end_time - self.eval_time)} t/s")


    
def main():
    rclpy.init()
    llama_node = LlamaNode()
    rclpy.spin(llama_node)
    llama_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
