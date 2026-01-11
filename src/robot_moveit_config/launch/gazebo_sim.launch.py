import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包路径
    moveit_config_pkg = get_package_share_directory('robot_moveit_config')
    
    # 使用包含 ros2_control 配置的 xacro 文件
    urdf_xacro = os.path.join(moveit_config_pkg, 'config', 'rm75_gripper_cameras.urdf.xacro')
    controllers_file = os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')

    # 关闭在线模型库（防止网络错误）
    set_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    
    # 设置 Gazebo 模型路径
    gazebo_models_path = '/usr/share/gazebo-11/models'
    home_gazebo_models = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join([p for p in [gazebo_models_path, home_gazebo_models] if os.path.isdir(p)])
    )

    # 解析 URDF (Xacro) - 启用 Gazebo ros2_control
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_xacro,
        ' use_fake_hardware:=false',
        ' use_gazebo:=true',
        ' gazebo_controllers_file:=', controllers_file
    ])
    robot_description = {"robot_description": robot_description_content}

    # 启动 Gazebo 服务器和客户端
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
    )

    # Robot State Publisher (发布 TF 和 robot_description)
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_robot",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.0",
            "-timeout", "120"
        ],
        output="screen",
    )

    # 加载控制器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_hand_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # 确保顺序：Gazebo 启动后 -> 生成机器人 -> 启动控制器
    return LaunchDescription([
        set_model_database_uri,
        set_gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller, load_gripper_controller],
            )
        ),
    ])