#!/usr/bin/env python

import rclpy 

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import math 
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration

class JoyToTurretNode(Node):
    is_moving:bool = False
    
    def __init__(self):
        super().__init__("joy_to_turret_node")
        self.subscription = self.create_subscription(
             Joy,
            '/joy',
            self.listener_callback,
            1)
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=10)
        )

        self.publisher = self.create_publisher(JointTrajectoryPoint,
                                               '/turret/joint_trajectory_points', qos_profile=qos)
        self.subscription
        
    
    def listener_callback(self, msg : Joy):
        range0 = msg.axes[0]
        range1 = msg.axes[1]
        if range0 == 0.0 and range1 == 0.0:
            return
        
        angle0 = calc_angle(range0)
        angle1 = calc_angle(range1, math.pi/2, math.pi/2)
        
        self.get_logger().debug(f'I heard: "{angle0} and {angle1}"')
        self.send_goal(angle0, angle1)


    def send_goal(self, angle0, angle1):
                       
        joint_points = JointTrajectoryPoint()
        joint_points.positions = [angle0, angle1]
        joint_points.time_from_start = Duration(seconds=0, nanoseconds=500).to_msg()        
        
        self.publisher.publish(joint_points)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
def calc_angle(range, min = 2*math.pi, max=2*math.pi):
    if range > 0:
        return round(range*max, 4)
    else:
        return round(range*min, 4)

def main(args=None):
    rclpy.init()
    
    mover = JoyToTurretNode()    
    rclpy.spin(mover)
    
if __name__ == '__main__':
    main()