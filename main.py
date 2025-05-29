#!/usr/bin/env python3

import os

import rclpy
from std_msgs.msg import String

from perception.align_to_aruco import main as align_to_aruco
from perception.object_pose_server import AVAILABLE_OBJECTS

def show_menu():
    print('\n--- Main Menu ---')
    print('1. Move to start position')
    print('2. Detect an object')
    print('3. Grab the detected object')
    print('4. Exit')
    return input('Choose an option (1-4): ')

def align_to_start_pos():
    """ 
    This function should command the robot to move to the table using Navigation module.
    And then the robot should align itself to the 'table' marker automatically.
    """
    align_to_aruco()

def detect_object():
    """
    This function should command the robot to detect if a specified object exists using Perception module.
    """
    object = input('Enter which object to detect: ')

    if object in AVAILABLE_OBJECTS:
        pass
    else:
        print(f'Error: invalid object name {object}')
        print(f'Available options are: {AVAILABLE_OBJECTS}')
    
    rclpy.init()
    node = rclpy.create_node('detect_object_publisher')
    pub = node.create_publisher(String, 'team3objectposesave', 10)
    msg = String()
    msg.data = object
    pub.publish(msg)
    rclpy.shutdown()

def grab_detected_object():
    """
    This function should command the robot to grab the detected object using the relative positions between the object and robot.
    """
    

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
            print('Goodbye!')
            break
        else:
            print('Invalid choice. Please try again.')
        
        input('\nPress Enter to continue...')

if __name__ == '__main__':
    main()