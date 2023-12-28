#!/usr/bin/env python

import rclpy 

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import math 
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, QoSLivelinessPolicy
from rclpy.duration import Duration

class TurretMoverNode(Node):
    is_moving:bool = False
    
    def __init__(self):
        super().__init__("move_turret_node")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/turret_controller/follow_joint_trajectory')

        # Create a QoS profile
        qos = QoSProfile(
            depth=10,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=10)
        )

        self.subscription = self.create_subscription(
            JointTrajectoryPoint,
            '/turret/joint_trajectory_points',
            self.listener_callback,
            qos)
        
        self.subscription
    
    def listener_callback(self, msg : JointTrajectoryPoint):
        platform_joint_angle = msg.positions[0]
        tilt_joint_angle = msg.positions[1]
                
        self.get_logger().debug(f'I heard: "{platform_joint_angle} and {tilt_joint_angle}"')
        self.send_goal(platform_joint_angle, tilt_joint_angle)


    def send_goal(self, platform_joint_angle, tilt_joint_angle):
        if self.is_moving:
            self.get_logger().info("Still Moving, the goal skipped ...")            
            return
        
        self.get_logger().debug(f'Turning turret to: {platform_joint_angle} and {tilt_joint_angle}"')
        goal_msg = FollowJointTrajectory.Goal()
        joint_names = ['revolute_platform_joint', 'revolute_tilt_joint']
        
        points = []
                
        joint_points = JointTrajectoryPoint()
        joint_points.positions = [platform_joint_angle, tilt_joint_angle]
        joint_points.time_from_start = Duration(seconds=0, nanoseconds=500).to_msg()
        points.append(joint_points)
        
        goal_msg.goal_time_tolerance = Duration(seconds=2, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        
        self._action_client.wait_for_server()        
        self.send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected ...")            
            return                
        self.is_moving = True
        self.get_logger().info("Goal accepted ...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {str(result)}")
        self.is_moving = False
        #rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):        
        feedback = feedback_msg.feedback
        #self.get_logger().info(f"Feedback: {str(feedback)}")
        
def calc_angle(range, min = 2*math.pi, max=2*math.pi):
    if range > 0:
        return round(range*max, 4)
    else:
        return round(range*min, 4)

def main(args=None):
    rclpy.init()
    
    mover = TurretMoverNode()
    angle1 = 2*3.14
    angle2 = -0.1
    
    #future = mover.send_goal(angle1, angle2)
    
    rclpy.spin(mover)
    
if __name__ == '__main__':
    main()