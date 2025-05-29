import rclpy
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

multiplier, adjustment, joint_to_move = 1, 0, None
rotate, q = None, None

movement_node = hm.HelloNode.quick_create('group3node')

def load_and_replay(filename):
    saved_pose = None

    with open(filename) as f:
        saved_pose = json.load(f)

        x = saved_pose["translation"]["x"]
        y = saved_pose["translation"]["y"]
        z = saved_pose["translation"]["z"]

        movement_node.move_to_pose({'joint_lift': y})
        movement_node.move_to_pose({'joint_arm': z})
        movement_node.move_to_pose({'translate_mobile_base': x})
        movement_node.move_to_pose({'joint_gripper_finger_left': -1.2})

        movement_node.move_to_pose({'joint_lift': 1.0})
        movement_node.move_to_pose({'joint_arm': 0.25})