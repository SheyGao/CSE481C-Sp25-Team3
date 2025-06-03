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

def drop():
    movement_node = hm.HelloNode.quick_create('group3node')

    movement_node.move_to_pose({'joint_arm': 0.5})
    movement_node.move_to_pose({'joint_lift': 0.75})
    time.sleep(4)
    movement_node.move_to_pose({'joint_gripper_finger_left': 1.2})  # 0.2 for bowl

if __name__ == '__main__':
    drop()