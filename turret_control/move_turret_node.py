#!/usr/bin/env python

import rclpy 

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
import math 

class TurretMover(Node):
    is_moving:bool = False
    
    def __init__(self):
        super().__init__("move_turret_node")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/turret_controller/follow_joint_trajectory')
        self.subscription = self.create_subscription(
             Joy,
            '/joy',
            self.listener_callback,
            1)
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
        if self.is_moving:
            return
        
        self.get_logger().debug(f'Turning turret to: {angle0} and {angle1}"')
        goal_msg = FollowJointTrajectory.Goal()
        joint_names = ['revolute_platform_joint', 'revolute_tilt_joint']
        
        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]
        points.append(point1)
        
        point2 = JointTrajectoryPoint()
        point2.positions = [angle0, angle1]
        point2.time_from_start = Duration(seconds=0, nanoseconds=500).to_msg()
        points.append(point2)
        
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        self.is_moving = True
        self._action_client.wait_for_server()        
        self.send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected ...")
            return        
        
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
        
def calc_angle(range, min = 2*math.pi, max=2*math.pi):
    if range > 0:
        return round(range*max, 4)
    else:
        return round(range*min, 4)

def main(args=None):
    rclpy.init()
    
    mover = TurretMover()
    angle1 = 2*3.14
    angle2 = -0.1
    
    #future = mover.send_goal(angle1, angle2)
    
    rclpy.spin(mover)
    
if __name__ == '__main__':
    main()