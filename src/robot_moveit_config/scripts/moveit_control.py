#!/usr/bin/env python3
"""
MoveIt2 Python API example for controlling the robot arm and gripper.
Uses moveit_py official Python bindings.

Usage:
    1. Launch Gazebo + MoveIt2:
       ros2 launch robot_moveit_config gazebo_moveit.launch.py

    2. Run this script:
       ros2 run robot_moveit_config moveit_control.py
       # Or directly:
       python3 moveit_control.py
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import time


def get_moveit_params():
    """Build MoveIt configuration parameters."""
    pkg_moveit_config = get_package_share_directory('robot_moveit_config')

    # Build MoveIt config using the same builder as the launch file
    moveit_config = MoveItConfigsBuilder(
        "rm75_gripper_cameras",
        package_name="robot_moveit_config"
    ).to_moveit_configs()

    # Combine all parameters into a single dictionary
    params = {}

    # Robot description (URDF) - get from topic since Gazebo publishes it
    params["robot_description"] = ""  # Will be fetched from /robot_description topic

    # Robot description semantic (SRDF)
    if moveit_config.robot_description_semantic:
        params.update(moveit_config.robot_description_semantic)

    # Kinematics
    if moveit_config.robot_description_kinematics:
        params.update(moveit_config.robot_description_kinematics)

    # Planning pipelines
    if moveit_config.planning_pipelines:
        params.update(moveit_config.planning_pipelines)

    # Joint limits
    if moveit_config.joint_limits:
        params.update(moveit_config.joint_limits)

    # Trajectory execution
    if moveit_config.trajectory_execution:
        params.update(moveit_config.trajectory_execution)

    # Use simulation time
    params["use_sim_time"] = True

    return params


class RobotArmController:
    """Controller for robot arm and gripper using MoveIt2 Python API."""

    def __init__(self):
        # Initialize ROS2
        rclpy.init()

        # # Get MoveIt configuration parameters
        # moveit_params = get_moveit_params()

        # # Initialize MoveItPy with the robot configuration
        # self.moveit = MoveItPy(
        #     node_name="moveit_py_controller",
        #     config_dict=moveit_params
        # )

        # Build MoveIt config using the same pattern as gazebo_moveit.launch.py
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="rm75_gripper_cameras",
                package_name="robot_moveit_config"
            )
            .robot_description()
            .robot_description_semantic()
            .robot_description_kinematics()
            .moveit_cpp(
                file_path="config/motion_planning_python_api_tutorial.yaml"
            )
            .to_moveit_configs()
        )
        config_dict = moveit_config.to_dict()
        config_dict['use_sim_time'] = True 
        # Ensure MoveItPy's internal node does not fail on invalid /clock QoS overrides.
        config_dict['qos_overrides./clock.subscription.durability'] = 'system_default'
        config_dict['qos_overrides./clock.subscription.history'] = 'keep_last'
        config_dict['qos_overrides./clock.subscription.depth'] = 10
        config_dict['qos_overrides./clock.subscription.reliability'] = 'reliable'

        # Initialize MoveItPy
        self.moveit = MoveItPy(
            node_name="moveit_py_node",
            config_dict=config_dict
        )

        # Get planning components for arm and gripper
        self.arm = self.moveit.get_planning_component("robot_arm")
        self.gripper = self.moveit.get_planning_component("robot_hand")

        # Get robot model and state
        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        print("MoveIt2 Python controller initialized successfully!")
        print(f"Robot model: {self.robot_model.name}")

    def get_current_pose(self) -> PoseStamped:
        """Get the current pose of the end effector."""
        with self.planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
            pose = robot_state.get_pose("Link7")

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pose
            return pose_stamped

    def move_to_pose(self, pose: PoseStamped, cartesian: bool = False) -> bool:
        """
        Move the arm to a target pose.

        Args:
            pose: Target pose for the end effector
            cartesian: If True, use Cartesian path planning

        Returns:
            True if motion was successful, False otherwise
        """
        # Set start state to current state
        self.arm.set_start_state_to_current_state()

        # Set goal state using pose target
        self.arm.set_goal_state(
            pose_stamped_msg=pose,
            pose_link="Link7"
        )

        # Plan the motion
        print("Planning motion to target pose...")
        plan_result = self.arm.plan()

        if plan_result:
            print("Plan found! Executing...")
            # Execute the planned trajectory
            robot_trajectory = plan_result.trajectory
            result = self.moveit.execute(robot_trajectory, controllers=[])
            print("Motion executed!" if result else "Execution failed!")
            return result
        else:
            print("Planning failed!")
            return False

    def move_to_position(self, x: float, y: float, z: float,
                         qx: float = 0.0, qy: float = 0.707,
                         qz: float = 0.0, qw: float = 0.707) -> bool:
        """
        Move the arm to a target position with orientation.

        Args:
            x, y, z: Target position in meters
            qx, qy, qz, qw: Target orientation as quaternion

        Returns:
            True if motion was successful, False otherwise
        """
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        print(f"Moving to position: ({x:.3f}, {y:.3f}, {z:.3f})")
        return self.move_to_pose(pose)

    def move_to_named_state(self, state_name: str) -> bool:
        """
        Move the arm to a named state (e.g., 'home').

        Args:
            state_name: Name of the predefined state

        Returns:
            True if motion was successful, False otherwise
        """
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=state_name)

        print(f"Planning motion to '{state_name}' state...")
        plan_result = self.arm.plan()

        if plan_result:
            print("Plan found! Executing...")
            result = self.moveit.execute(plan_result.trajectory, controllers=[])
            print("Motion executed!" if result else "Execution failed!")
            return result
        else:
            print("Planning failed!")
            return False

    def open_gripper(self) -> bool:
        """
        Open the gripper using the 'open' named state.

        Returns:
            True if motion was successful, False otherwise
        """
        print("Opening gripper...")
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="open")

        plan_result = self.gripper.plan()

        if plan_result:
            result = self.moveit.execute(plan_result.trajectory, controllers=[])
            print("Gripper opened!" if result else "Failed to open gripper!")
            return result
        else:
            print("Gripper planning failed!")
            return False

    def close_gripper(self) -> bool:
        """
        Close the gripper using the 'close' named state.

        Returns:
            True if motion was successful, False otherwise
        """
        print("Closing gripper...")
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="close")

        plan_result = self.gripper.plan()

        if plan_result:
            result = self.moveit.execute(plan_result.trajectory, controllers=[])
            print("Gripper closed!" if result else "Failed to close gripper!")
            return result
        else:
            print("Gripper planning failed!")
            return False

    def shutdown(self):
        """Shutdown the controller."""
        print("Shutting down...")
        rclpy.shutdown()


def main():
    """Main function demonstrating robot arm and gripper control."""

    # Create the controller
    controller = RobotArmController()

    try:
        # Wait for system to be ready
        time.sleep(1.0)

        # Print current pose
        current_pose = controller.get_current_pose()
        print(f"\nCurrent end effector pose:")
        print(f"  Position: ({current_pose.pose.position.x:.3f}, "
              f"{current_pose.pose.position.y:.3f}, "
              f"{current_pose.pose.position.z:.3f})")
        print(f"  Orientation: ({current_pose.pose.orientation.x:.3f}, "
              f"{current_pose.pose.orientation.y:.3f}, "
              f"{current_pose.pose.orientation.z:.3f}, "
              f"{current_pose.pose.orientation.w:.3f})")

        # === Example 1: Move to home position ===
        print("\n=== Moving to home position ===")
        controller.move_to_named_state("home")
        time.sleep(1.0)

        # === Example 2: Open gripper ===
        print("\n=== Opening gripper ===")
        controller.open_gripper()
        time.sleep(0.5)

        # === Example 3: Move to a target pose ===
        # Adjust these values based on your robot's workspace
        print("\n=== Moving to target pose ===")
        controller.move_to_position(
            x=0.3,    # Forward
            y=0.0,    # Center
            z=0.4,    # Up
            qx=0.0,   # Orientation quaternion
            qy=0.707, # Pointing down
            qz=0.0,
            qw=0.707
        )
        time.sleep(1.0)

        # === Example 4: Close gripper (grasp) ===
        print("\n=== Closing gripper ===")
        controller.close_gripper()
        time.sleep(0.5)

        # === Example 5: Move to another position ===
        print("\n=== Moving to second position ===")
        controller.move_to_position(
            x=0.2,
            y=0.2,
            z=0.5,
            qx=0.0,
            qy=0.707,
            qz=0.0,
            qw=0.707
        )
        time.sleep(1.0)

        # === Example 6: Open gripper (release) ===
        print("\n=== Opening gripper ===")
        controller.open_gripper()
        time.sleep(0.5)

        # === Example 7: Return to home ===
        print("\n=== Returning to home ===")
        controller.move_to_named_state("home")

        print("\n=== Demo complete! ===")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
