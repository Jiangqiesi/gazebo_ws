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
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_moveit_config = get_package_share_directory('robot_moveit_config')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 关闭在线模型库（防止 SSL 错误和网络超时）
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

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # # Robot description from xacro - configured for Gazebo
    # xacro_command = (
    #     f'xacro {urdf_xacro} use_fake_hardware:=false use_gazebo:=true '
    #     f'gazebo_controllers_file:={controllers_file}'
    # )
    # robot_description = Command([
    #     f"bash -c \"{xacro_command} | tr -d '\\n'\""
    # ])
    # robot_description_param = ParameterValue(robot_description, value_type=str)

    # Robot description from xacro - configured for Gazebo
    # 修改：直接使用 Command 调用 xacro，不使用 bash 和 tr 去除换行符。
    # tr -d '\n' 产生的超长单行字符串会导致 gazebo_ros2_control 参数解析溢出。
    robot_description = Command([
        'xacro ', urdf_xacro,
        ' use_fake_hardware:=false use_gazebo:=true ',
        ' gazebo_controllers_file:=', controllers_file
    ])
    robot_description_param = ParameterValue(robot_description, value_type=None)

    # Gazebo server (empty world)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
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
            'robot_description': robot_description_param,
            'use_sim_time': use_sim_time,
        }]
    )

    # Spawn robot in Gazebo
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
            '-timeout', '120',
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
            'robot_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robot_hand_controller',
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

    # Delay spawn_robot to give Gazebo time to fully initialize
    delayed_spawn = TimerAction(
        period=10.0,
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
    ])
