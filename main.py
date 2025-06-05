#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import perception.align_to_aruco as align_to_aruco
from perception.object_pose_server import AVAILABLE_OBJECTS
from perception.object_pose_server import POSES_FILENAME
from manipulation.pose_replayer import load_and_replay

DETECTION_QUERY_INTERVAL = 3
TIMEOUT_SECONDS = 10

def show_menu():
    print('\n--- Main Menu ---')
    print('1. Move to start position')
    print('2. Detect an object')
    print('3. Grab the detected object')
    print('4. Voice Command Mode')
    print('5. Exit')
    return input('Choose an option (1-5): ')

def align_to_start_pos():
    """ 
    This function should command the robot to move to the table using Navigation module.
    And then the robot should align itself to the 'table' marker automatically.
    """
    align_to_aruco.main()

def detect_object():
    """
    This function should command the robot to detect if a specified object exists using Perception module.
    """
    object = input('Enter which object to detect: ')

    if object in AVAILABLE_OBJECTS:
        rclpy.init()
        node = rclpy.create_node('detect_object_publisher')
        pub = node.create_publisher(String, 'team3objectposesave', 10)
        msg = String()
        msg.data = object
        pub.publish(msg)
        rclpy.shutdown()
    else:
        print(f'Error: invalid object name {object}.')
        print(f'Available options are: {AVAILABLE_OBJECTS}.')

def grab_detected_object():
    """
    This function should command the robot to grab the detected object using the relative positions between the object and robot.
    """
    filename = './' + POSES_FILENAME
    if os.path.exists(filename) and os.path.getsize(filename) > 0:
        load_and_replay(filename)
    else:
        print(f"Error: {filename} doesn't exist or is empty.")

class VoiceCommandListener(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')
        self.voice_command = self.create_subscription(
            String,
            'team3voicecommand',
            self.voice_command_callback,
            10)
        self.voice_command

    def voice_command_callback(self, msg):
        print(msg.data)
        # align_to_aruco.main()

        # start_time = time.time()
        # last_time = start_time

        # filename = './' + POSES_FILENAME
        # with open(filename, 'w') as f:
        #     # File is automatically emptied/created
        #     pass

        # while not(os.path.exists(filename) and os.path.getsize(filename) > 0):
        #     if time.time() - last_time > DETECTION_QUERY_INTERVAL:
        #         rclpy.init()
        #         node = rclpy.create_node('detect_object_publisher')
        #         pub = node.create_publisher(String, 'team3objectposesave', 10)
        #         msg = String()
        #         msg.data = msg.data
        #         pub.publish(msg)
        #         rclpy.shutdown()

        #         last_time = time.time()

        #     if time.time() - start_time > TIMEOUT_SECONDS:
        #         print("Timeout reached!")
        #         break
        
        # if os.path.exists(filename) and os.path.getsize(filename) > 0:
        #     load_and_replay(filename)
        # else:
        #     print(f"Error!")


def voice_command_publish():
    rclpy.init()
    node = VoiceCommandListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

def main():
    while True:
        choice = show_menu()
        
        if choice == '1':
            align_to_start_pos()
        elif choice == '2':
            detect_object()
        elif choice == '3':
            grab_detected_object()
        elif choice == '4':
            voice_command_publish()
        elif choice == '5':
            print('Goodbye!')
            break
        else:
            print('Invalid choice. Please try again.')
        
        input('\nPress Enter to continue...')

if __name__ == '__main__':
    main()