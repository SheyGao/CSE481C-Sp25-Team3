import numpy as np
import rclpy
import json
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener, TransformBroadcaster
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped

AVAILABLE_OBJECTS = ['bowl', 'spoon', 'knife', 'fork']
POSES_FILENAME = "object_poses.json"

class ObjectDetectionListener(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')
        
        # Setup tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Setup tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.marker_array_pose_sub = self.create_subscription(
            MarkerArray,
            'objects/marker_array',
            self.marker_array_callback,
            10)
        self.marker_array_pose_sub
        self.markers = []
        self.query_pose_sub = self.create_subscription(
            String,
            'team3objectposequery',
            self.query_pose_callback,
            10)
        self.query_pose_sub

    def marker_array_callback(self, msg):
        self.markers = msg.markers
        
        # Broadcast each marker as a TF frame
        for marker in self.markers:
            if marker.text in AVAILABLE_OBJECTS:
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = marker.header.frame_id
                transform.child_frame_id = f"{marker.text}_object"
                
                # Copy pose from marker
                transform.transform.translation.x = marker.pose.position.x
                transform.transform.translation.y = marker.pose.position.y
                transform.transform.translation.z = marker.pose.position.z
                transform.transform.rotation.x = marker.pose.orientation.x
                transform.transform.rotation.y = marker.pose.orientation.y
                transform.transform.rotation.z = marker.pose.orientation.z
                transform.transform.rotation.w = marker.pose.orientation.w
                
                # Broadcast the transform
                self.tf_broadcaster.sendTransform(transform)

    def query_pose_callback(self, msg):
        # ros2 topic pub team3objectposesave std_msgs/msg/String "{data: 'home'}" -1
        object_frame = f"{msg.data}_object"
        
        try:
            # Use lookup_transform directly to get the pose in base_link
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                object_frame,
                rclpy.time.Time())
            
            pose_data = {
                'name': object_frame,
                'translation': {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z
                },
                'rotation': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w
                }
            }
            
            with open(POSES_FILENAME, 'w') as f:
                json.dump(pose_data, f, indent=2)
                f.write("\n")
                
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform for {object_frame}: {str(e)}")
        
        return

def main():
    rclpy.init()
    node = ObjectDetectionListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()