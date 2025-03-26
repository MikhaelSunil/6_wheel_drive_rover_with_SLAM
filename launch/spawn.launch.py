import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    try:
        # Get package share directory
        share_dir = get_package_share_directory('mine_rover_description')
        urdf_dir = os.path.join(share_dir, 'urdf')

        xacro_file = os.path.join(urdf_dir, 'mine_rover.xacro')
        ros2_control_file = os.path.join(urdf_dir, 'mine_rover.ros2control')
        
        # Process xacro file
        robot_description_config = xacro.process_file(xacro_file)
        robot_urdf = robot_description_config.toxml()
    except Exception as e:
        return LaunchDescription([Node(
            package='launch_ros',
            executable='log_info',
            arguments=[f"Failed to load URDF: {e}"]
        )])

    # RViz configuration file
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # Declare GUI launch argument
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # LiDAR node (Assuming topic is /scan)
    lidar_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lidar',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        lidar_node
    ])

# Gazebo Launch File

def generate_gazebo_launch_description():
    world_file_name = 'myworld.world'
    package_name = 'mine_rover_description'
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('mine_rover_description')
    
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'mine_rover.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Start Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start Gazebo Sim with Custom World
    gazebo = Node(
        package='ros_gz_sim',
        executable='gz_sim',
        arguments=["-r", "-v 4", world_path],
        output='screen',
    )

    # Spawn Robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "mine_rover",
            "-allow_renaming", "true",
            "-z", "0.32",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0"
        ],            
        output='screen',
    )

    # Bridge ROS topics and Gazebo messages
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen',
    )  
    
    # LiDAR Bridge (Assuming /scan is published by Gazebo)
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn,
        start_gazebo_ros_bridge_cmd,
        robot_state_publisher,
        lidar_bridge
    ])

