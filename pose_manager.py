#!/usr/bin/env python3

import sys
import json
import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import time
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

POSES_FILE = "saved_poses.json"

class PoseManager(Node):
    def __init__(self):
        super().__init__('pose_manager')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.poses = {}
        
        # Reset file on start
        self.reset_file()
        
    def reset_file(self):
        """Reset the poses file"""
        with open(POSES_FILE, 'w') as f:
            json.dump({}, f)
        self.poses = {}
        
    def save_pose(self, name, source_frame, target_frame, timeout=10.0):
        """Save a single pose to file with continuous polling until success or timeout"""
        start_time = time.time()
        attempt_count = 0
        
        print(f"Attempting to get transform from '{source_frame}' to '{target_frame}'...")
        print(f"Timeout: {timeout} seconds")
        
        while time.time() - start_time < timeout:
            attempt_count += 1
            
            try:
                # Spin once to process any incoming TF data
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # Try to get the transform
                now = Time()
                trans = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.5))
                
                # If we get here, we successfully got the transform
                # Convert to serializable format
                pose_data = {
                    'source_frame': source_frame,
                    'target_frame': target_frame,
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
                
                # Save to file
                self.poses[name] = pose_data
                with open(POSES_FILE, 'w') as f:
                    json.dump(self.poses, f, indent=2)
                
                print(f"\n✓ Pose '{name}' saved successfully after {attempt_count} attempts!")
                print(f"  Source: {source_frame}")
                print(f"  Target: {target_frame}")
                return True
                
            except TransformException as ex:
                # Show progress indicator
                elapsed = time.time() - start_time
                remaining = timeout - elapsed
                print(f"\rAttempt {attempt_count}: Waiting for transform... ({remaining:.1f}s remaining)", end='', flush=True)
                
                # Small delay before next attempt
                time.sleep(0.1)
                
            except Exception as e:
                print(f"\n✗ Error saving pose: {e}")
                return False
        
        # If we get here, we've timed out
        print(f"\n✗ Timeout: Could not get transform from '{source_frame}' to '{target_frame}' after {timeout} seconds")
        return False
    
    def list_poses(self):
        """List all saved poses"""
        try:
            with open(POSES_FILE, 'r') as f:
                self.poses = json.load(f)
            
            if not self.poses:
                print("No poses saved yet.")
                return
            
            print("\nSaved Poses:")
            print("-" * 40)
            for i, (name, data) in enumerate(self.poses.items(), 1):
                print(f"{i}. {name}")
                print(f"   Source: {data['source_frame']}")
                print(f"   Target: {data['target_frame']}")
                print()
        except Exception as e:
            print(f"Error listing poses: {e}")


class PoseBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_broadcaster')
        self.br = TransformBroadcaster(self)
        self.transforms = []
        self.load_poses()
        
        # Set up timer for broadcasting
        time_period = 0.1  # seconds
        self.timer = self.create_timer(time_period, self.broadcast_timer_callback)
        
    def load_poses(self):
        """Load poses from file and prepare transforms"""
        try:
            with open(POSES_FILE, 'r') as f:
                poses = json.load(f)
            
            for name, pose_data in poses.items():
                transform = TransformStamped()
                transform.header.frame_id = pose_data['source_frame']
                transform.child_frame_id = f"saved_{name}"  # Add prefix to avoid conflicts
                
                # Set translation
                transform.transform.translation.x = pose_data['translation']['x']
                transform.transform.translation.y = pose_data['translation']['y']
                transform.transform.translation.z = pose_data['translation']['z']
                
                # Set rotation
                transform.transform.rotation.x = pose_data['rotation']['x']
                transform.transform.rotation.y = pose_data['rotation']['y']
                transform.transform.rotation.z = pose_data['rotation']['z']
                transform.transform.rotation.w = pose_data['rotation']['w']
                
                self.transforms.append(transform)
            
            self.get_logger().info(f"Loaded {len(self.transforms)} poses for broadcasting")
            
        except Exception as e:
            self.get_logger().error(f"Error loading poses: {e}")
    
    def broadcast_timer_callback(self):
        """Broadcast all transforms"""
        current_time = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = current_time
            self.br.sendTransform(transform)


class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')
        
        # Create an action client for FollowJointTrajectory
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')
    
    def move_lift_to_position(self, position):
        """Move the lift to a specific position using FollowJointTrajectory action"""
        # Create a trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_lift']
        
        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        trajectory.points = [point]
        
        # Create the goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # Send the goal
        self.get_logger().info(f'Sending goal to move lift to {position:.3f} meters...')
        future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for the goal to be accepted
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted!')
        
        # Wait for the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        if result:
            self.get_logger().info('✓ Lift movement completed!')
            return True
        else:
            self.get_logger().error('✗ Lift movement failed!')
            return False


def main_menu():
    """Display main menu and get user choice"""
    print("\n=== Pose Manager ===")
    print("1. Save a new pose")
    print("2. List all poses")
    print("3. Broadcast all poses")
    print("4. Move lift to pose")
    print("5. Exit")
    print()
    
    while True:
        choice = input("Enter your choice (1-5): ").strip()
        if choice in ['1', '2', '3', '4', '5']:
            return choice
        print("Invalid choice. Please enter 1-5.")


def save_pose_interactive(pose_manager):
    """Interactive pose saving"""
    print("\n=== Save New Pose ===")
    source_frame = input("Enter source frame: ").strip()
    target_frame = input("Enter target frame: ").strip()
    name = input("Enter name for this pose: ").strip()
    
    if not all([source_frame, target_frame, name]):
        print("All fields are required!")
        return
    
    # Optional: Ask for custom timeout
    timeout_input = input("Enter timeout in seconds (default: 10): ").strip()
    if timeout_input:
        try:
            timeout = float(timeout_input)
        except ValueError:
            print("Invalid timeout value. Using default: 10 seconds")
            timeout = 10.0
    else:
        timeout = 10.0
    
    pose_manager.save_pose(name, source_frame, target_frame, timeout)


def move_lift_to_pose():
    """Move the robot's lift to a saved pose's Y translation value"""
    try:
        # Load poses from file
        with open(POSES_FILE, 'r') as f:
            poses = json.load(f)
        
        if not poses:
            print("\nNo poses saved yet.")
            return
        
        # Display poses with numbers
        print("\n=== Move Lift to Pose ===")
        print("Select a pose:")
        print("-" * 40)
        pose_list = list(poses.items())
        for i, (name, data) in enumerate(pose_list, 1):
            y_value = data['translation']['y']
            print(f"{i}. {name} (Y: {y_value:.3f} meters)")
        
        # Get user selection
        while True:
            choice = input("\nEnter pose number (or 'c' to cancel): ").strip()
            if choice.lower() == 'c':
                print("Cancelled.")
                return
            
            try:
                index = int(choice) - 1
                if 0 <= index < len(pose_list):
                    break
                else:
                    print("Invalid number. Please try again.")
            except ValueError:
                print("Please enter a number or 'c'.")
        
        # Get the selected pose
        selected_name, selected_data = pose_list[index]
        y_value = selected_data['translation']['y']
        
        print(f"\nMoving lift to Y position of '{selected_name}': {y_value:.3f} meters")
        
        # Create a temporary node to control the lift
        lift_controller = LiftController()
        lift_controller.move_lift_to_position(y_value)
        
        # Clean up
        lift_controller.destroy_node()
        
    except FileNotFoundError:
        print("\n✗ Error: No saved poses file found.")
    except json.JSONDecodeError:
        print("\n✗ Error: Saved poses file is corrupted.")
    except Exception as e:
        print(f"\n✗ Error moving lift: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    pose_manager = None
    broadcaster = None
    
    try:
        while True:
            choice = main_menu()
            
            if choice == '1':
                # Save a new pose
                if pose_manager is None:
                    pose_manager = PoseManager()
                save_pose_interactive(pose_manager)
                
            elif choice == '2':
                # List all poses
                if pose_manager is None:
                    pose_manager = PoseManager()
                pose_manager.list_poses()
                
            elif choice == '3':
                # Broadcast all poses
                print("\nBroadcasting all saved poses...")
                print("Press Ctrl+C to stop broadcasting and return to menu.")
                
                if broadcaster is None:
                    broadcaster = PoseBroadcaster()
                
                try:
                    rclpy.spin(broadcaster)
                except KeyboardInterrupt:
                    print("\nStopped broadcasting.")
                    broadcaster.destroy_node()
                    broadcaster = None
                
            elif choice == '4':
                # Move lift to pose
                move_lift_to_pose()
                
            elif choice == '5':
                # Exit
                print("Goodbye!")
                break
    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if pose_manager is not None:
            pose_manager.destroy_node()
        if broadcaster is not None:
            broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()