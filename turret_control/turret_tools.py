from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

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
