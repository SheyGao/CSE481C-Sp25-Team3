import numpy as np
import rclpy
import json
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from visualization_msgs.msg import MarkerArray

AVAILABLE_OBJECTS = ['bowl', 'spoon', 'knife', 'fork']
POSES_FILENAME = "object_poses.json"

class ObjectDetectionListener(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')
        
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


    def query_pose_callback(self, msg):
        # ros2 topic pub team3objectposesave std_msgs/msg/String "{data: 'home'}" -1
        for marker in self.markers:
            if marker.text == msg.data:
                tf_buffer = Buffer()
                now = Time()
                trans = tf_buffer.lookup_transform(
                    'base_link',
                    'unknown',
                    now)

                relative = np.array(
                    [
                        [marker.pose.position.x],
                        [marker.pose.position.y],
                        [marker.pose.position.z],
                        [1],
                    ]
                )
                parent = np.array(
                    [
                        [trans.transform.translation.x],
                        [trans.transform.translation.y],
                        [trans.tranform.translation.z],
                        [1],
                    ]
                )
                P = np.matmul(parent, relative)

                global pose_data
                pose_data = {
                    'name': marker.text + '_object',
                    'translation': {
                        'x': P[0],
                        'y': P[1],
                        'z': P[2]
                    },
                    'rotation': {
                        'x': marker.pose.orientation.x,
                        'y': marker.pose.orientation.y,
                        'z': marker.pose.orientation.z,
                        'w': marker.pose.orientation.w
                    }
                }

                with open(POSES_FILENAME, 'w') as f:
                    json.dump(pose_data, f, indent=2)
                    f.write("\n")   

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
