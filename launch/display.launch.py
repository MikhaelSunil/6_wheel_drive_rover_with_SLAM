import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
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
    

    return LaunchDescription([
        gui_arg,
        declare_use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        
    ])  
