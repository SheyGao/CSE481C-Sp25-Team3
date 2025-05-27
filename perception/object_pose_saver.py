import rclpy
import json
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

POSES_FILENAME = "object_poses.json"

# ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/team3_map.yaml
class FrameListener(Node):

    def __init__(self):
        super().__init__('stretch_tf_listener')
        
        self.marker_array_pose_sub = self.create_subscription(
            MarkerArray,
            'objects/marker_array',
            self.marker_array_callback,
            10)
        self.marker_array_pose_sub  # prevent unused variable warning

        self.markers = []

        self.save_pose_sub = self.create_subscription(
            String,
            'team3objectposesave',
            self.save_pose_callback,
            10)
        self.save_pose_sub

    def marker_array_callback(self, msg):
       self.markers = msg.markers


    def save_pose_callback(self, msg):
        # ros2 topic pub team3objectposesave std_msgs/msg/String "{data: 'home'}" -1
        for marker in self.markers:
            if marker.text == 'bowl' or marker.text == 'spoon' or marker.text == 'fork':
                pose_data = {
                    'name': marker.text + '_object',
                    'translation': {
                        'x': marker.pose.position.x,
                        'y': marker.pose.position.y,
                        'z': marker.pose.position.z
                    },
                    'rotation': {
                        'x': marker.pose.orientation.x,
                        'y': marker.pose.orientation.y,
                        'z': marker.pose.orientation.z,
                        'w': marker.pose.orientation.w
                    }
                }
            
                with open(POSES_FILENAME, 'w+') as f:
                    json.dump(pose_data, f, indent=2)
                    f.write("\n")    

   
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
