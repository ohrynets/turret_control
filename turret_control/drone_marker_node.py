import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Vector3
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi

class DroneMarkerPublisher(Node):

    def __init__(self):
        super().__init__('drone_marker_node')
        # Declare and acquire `turtlename` parameter
        self.drone_name = self.declare_parameter(
          'drone_name', 'X4').get_parameter_value().string_value

        self.drone_x_offset = self.declare_parameter(
          'drone_x_offset', -5.0).get_parameter_value().double_value
        
        self.subscription = self.create_subscription(
            Odometry,
            f'/model/{self.drone_name}/odometry',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Marker, 'marker', 10)
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
        
        
        self.tf_broadcaster = TransformBroadcaster(self)
        #Transform Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg:Odometry):
        marker = self.create_marker("base_link", 
                                    pose=msg.pose.pose,
                                    type=Marker.ARROW,                                    
                                                 color=(0.0, 1.0, 0.0, 1.0))
        self.publisher_.publish(marker)
        self.broadcast_frame(msg)
        #self.get_logger().info(f'Publishing: "{marker}"')
        self.calculate_angle(msg)
    
    def calculate_angle(self, msg:Odometry):
        target_frame_camera = "base_link"
        target_frame_drone = self.get_drone_name()
        source_frame = "camera_link"
        
        try:            
            t_camera = self.tf_buffer.lookup_transform(
                target_frame = target_frame_camera,
                source_frame = source_frame,
                time = rclpy.time.Time())
            self.get_logger().info(f'Found Camera transform: {t_camera.transform}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {source_frame} to {target_frame_camera}: {ex}')
            return
        
        camera_position = t_camera.transform.translation
        drone_position = msg.pose.pose.position

        # Convert the quaternion rotations to Euler angles.
        start_position=Point(x=camera_position.x, y=camera_position.y, z=camera_position.z)
        end_position=Point(x=drone_position.x+self.drone_x_offset, y=drone_position.y, z=drone_position.z)        
        # Convert the points to Vector3
        start_point = np.array([start_position.x, start_position.y, start_position.z])
        end_point  = np.array([end_position.x, end_position.y, end_position.z])        
        direction =  end_point - start_point
        direction /= np.linalg.norm(direction)
        # Calculate the Euler angles that rotate the z-axis to the direction vector
        yaw = np.arctan2(direction[1], direction[0])        
        pitch = np.arctan2(direction[2], np.sqrt(direction[0]**2 + direction[1]**2))
        roll = 0.0
        euler_angles = [roll, -pitch, yaw]
        # Convert the Euler angles to a quaternion
        r = R.from_euler('xyz', euler_angles, degrees=False)
        self.get_logger().info(f'Angle difference: {euler_angles}')
        quaternion = r.as_quat()
        orientation=Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        camera_pose = Pose(position=start_position, orientation=orientation)
        marker = self.create_marker("base_link", 
                                    pose=camera_pose,
                                    scale=(1.0, 0.01, 0.01),
                                                 color=(1.0, 0.0, 0.0, 1.0))
        self.publisher_.publish(marker)
        self.send_goal(pi+euler_angles[2], euler_angles[1])
        
    def send_goal(self, platform_joint_angle, tilt_joint_angle):
                       
        joint_points = JointTrajectoryPoint()
        joint_points.positions = [platform_joint_angle, tilt_joint_angle]
        joint_points.time_from_start = Duration(seconds=0, nanoseconds=500).to_msg()        
        
        self.publisher.publish(joint_points)
        
    def create_marker(self, frame_id, points=None, pose: Pose = None, 
                      scale = (0.1, 0.1, 0.1),
                      type=Marker.ARROW, color=(0.0, 1.0, 0.0, 1.0)):
        # Convert the Euler angles to a quaternion
                
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        #marker.color = color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        
        marker.action = Marker.ADD                
        marker.type = type
        
        if pose is not None:
            marker.pose = pose
        
        if points is not None:
            marker.points.extend(points)
        
        return marker

    def broadcast_frame(self, msg:Odometry):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'base_link'
       t.child_frame_id = self.get_drone_name()
       
       t.transform.translation.x = msg.pose.pose.position.x
       t.transform.translation.y = msg.pose.pose.position.y
       t.transform.translation.z = msg.pose.pose.position.z
       t.transform.rotation.x = msg.pose.pose.orientation.x
       t.transform.rotation.y = msg.pose.pose.orientation.y
       t.transform.rotation.z = msg.pose.pose.orientation.z
       t.transform.rotation.w = msg.pose.pose.orientation.w

       self.tf_broadcaster.sendTransform(t)
       
    def get_drone_name(self):
        return f'drone_{self.drone_name}'


def main(args=None):
    rclpy.init(args=args)

    drone_marker_node = DroneMarkerPublisher()
    drone_marker_node.get_logger().info(f'Drone Marker Node Started')
    rclpy.spin(drone_marker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_marker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()