#!/usr/bin/env python

import rclpy 

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TurretMover(Node):
    def __init__(self):
        super().__init__("move_turret_node")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/turret_controller/follow_joint_trajectory')

    def send_goal(self, angle1, angle2):
        goal_msg = FollowJointTrajectory.Goal()
        joint_names = ['revolute_platform_joint', 'revolute_tilt_joint']
        
        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]
        points.append(point1)
        
        point2 = JointTrajectoryPoint()
        point2.positions = [angle1, angle2]
        point2.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()
        points.append(point2)
        
        goal_msg.goal_time_tolerance = Duration(seconds=5, nanoseconds=0).to_msg()
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
        
        self.get_logger().info("Goal accepted ...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {str(result)}")
        rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
def main(args=None):
    rclpy.init()
    
    mover = TurretMover()
    angle1 = 2*3.14
    angle2 = -0.1
    
    future = mover.send_goal(angle1, angle2)
    
    rclpy.spin(mover)
    
if __name__ == '__main__':
    main()