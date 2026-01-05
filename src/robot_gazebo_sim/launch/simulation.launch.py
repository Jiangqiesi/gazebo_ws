import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_robot_gazebo_sim = get_package_share_directory('robot_gazebo_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rm_75_description = get_package_share_directory('rm_75_6f_description')

    # 关闭在线模型库（防止 SSL 错误和网络超时）
    set_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    
    # 设置系统 Gazebo 模型路径（包含 ground_plane 和 sun）
    gazebo_models_path = '/usr/share/gazebo-11/models'
    home_gazebo_models = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    model_paths = [gazebo_models_path, home_gazebo_models]
    pkg_model_dirs = [
        os.path.join(pkg_robot_gazebo_sim, 'models'),
        os.path.join(pkg_rm_75_description, 'models'),
    ]
    for path in pkg_model_dirs:
        if os.path.isdir(path):
            model_paths.append(path)

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join(model_paths)
    )

    # Paths
    urdf_file = os.path.join(pkg_robot_gazebo_sim, 'urdf', 'robot_system.urdf.xacro')
    world_file = os.path.join(pkg_robot_gazebo_sim, 'worlds', 'table_scene.world')
    rviz_config = os.path.join(pkg_robot_gazebo_sim, 'config', 'display.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot description from xacro
    robot_description = Command(['xacro ', urdf_file])

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'false',
        }.items()
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # Joint State Publisher (for visualization without controllers)
    # Spawn robot in Gazebo - with increased timeout
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rm75_robot',
            '-x', '0.4',
            '-y', '0.0',
            '-z', '0.75',
            '-timeout', '120',  # Increase timeout to 120 seconds
        ],
        output='screen'
    )

    # Spawn controllers after the robot is inserted into Gazebo
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    # RViz (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=None  # Always launch for debugging
    )

    # Delay spawn_robot to give Gazebo time to fully initialize
    delayed_spawn = TimerAction(
        period=10.0,  # Wait 10 seconds before spawning
        actions=[spawn_robot]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        set_model_database_uri,
        set_gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        delayed_spawn,
        spawn_controllers,
        # rviz,  # Uncomment to launch RViz automatically
    ])
