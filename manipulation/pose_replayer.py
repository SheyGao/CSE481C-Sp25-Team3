import rclpy
import time
import json
import hello_helpers.hello_misc as hm
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String
from tf2_geometry_msgs import PoseStamped

def load_and_replay(filename):
    multiplier, adjustment, joint_to_move = 1, 0, None
    rotate, q = None, None

    movement_node = hm.HelloNode.quick_create('group3node')
    saved_pose = None

    with open(filename) as f:
        saved_pose = json.load(f)

        x = saved_pose["translation"]["x"]
        y = saved_pose["translation"]["y"]
        z = saved_pose["translation"]["z"]

        movement_node.move_to_pose({'joint_arm': 0.0})
        movement_node.move_to_pose({'joint_lift': 1.5})
        movement_node.move_to_pose({'joint_gripper_finger_left': 1.2})

        movement_node.move_to_pose({'translate_mobile_base': x - 0.03})
        time.sleep(2)
        movement_node.move_to_pose({'joint_arm': -y - 0.0990})
        time.sleep(4)
        movement_node.move_to_pose({'joint_lift': z + 0.21})
        time.sleep(4)
        movement_node.move_to_pose({'joint_gripper_finger_left': 0.1})  # 0.2 for bowl
        time.sleep(2)

        movement_node.move_to_pose({'joint_lift': 1.5})
        movement_node.move_to_pose({'joint_arm': 0.0})

        # movement_node.move_to_pose({'joint_arm': 0.25})
        # movement_node.move_to_pose({'joint_lift': 1.0})

if __name__ == '__main__':
    load_and_replay('object_poses.json')