import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Package directories
    pkg_moveit_config = get_package_share_directory('robot_moveit_config')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_package_path = Path(pkg_moveit_config)

    # 关闭在线模型库
    set_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    
    # 设置系统 Gazebo 模型路径
    gazebo_models_path = '/usr/share/gazebo-11/models'
    home_gazebo_models = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    model_paths = [gazebo_models_path, home_gazebo_models]

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([p for p in model_paths if os.path.isdir(p)])
    )

    # Paths
    urdf_xacro = os.path.join(pkg_moveit_config, 'config', 'rm75_gripper_cameras.urdf.xacro')
    controllers_file = os.path.join(pkg_moveit_config, 'config', 'ros2_controllers.yaml')
    default_world = os.path.join(pkg_moveit_config, 'gazebo', 'sim_env.world')
    
    # Robot description for Gazebo
    xacro_command = (
        f'xacro {urdf_xacro} use_fake_hardware:=false use_gazebo:=true '
        f'gazebo_controllers_file:={controllers_file}'
    )
    robot_description_content = Command([
        f"bash -c \"{xacro_command} | tr -d '\\n'\""
    ])

    # Build MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "rm75_gripper_cameras", 
        package_name="robot_moveit_config"
    ).to_moveit_configs()

    octomap_params = {
        "octomap_frame": "base_link",
        "octomap_resolution": 0.05,
    }

    # Override robot description with Gazebo-configured version
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    ))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    ))

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    # Environment setup
    ld.add_action(set_model_database_uri)
    ld.add_action(set_gazebo_model_path)

    # Gazebo server
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false',
            'world': LaunchConfiguration('world'),
        }.items()
    ))

    # Gazebo client
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    ))

    # Robot State Publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    ))

    # Spawn robot in Gazebo (delayed)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rm75_robot',
            '-x', '0.3', '-y', '0.0', '-z', '1.03',
            '-timeout', '120',
        ],
        output='screen'
    )
    ld.add_action(TimerAction(period=10.0, actions=[spawn_robot]))

    # Controller spawners (after robot is spawned)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_arm_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_hand_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],
        output='screen',
    )
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, arm_controller_spawner, gripper_controller_spawner],
        )
    ))

    # MoveIt move_group node
    move_group_params = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.planning_scene_monitor,
        moveit_config.sensors_3d,
        # moveit_config.robot_description_planning,
        moveit_config.trajectory_execution,
        moveit_config.joint_limits,
        octomap_params,
        {'use_sim_time': True},
    ]
    ld.add_action(Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    ))

    # RViz with MoveIt config
    rviz_config = str(launch_package_path / 'config/moveit.rviz')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # moveit_config.robot_description_planning,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    ))

    return ld
