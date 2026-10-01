from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    xacro_file = os.path.join(
        get_package_share_directory('the_8th_leg_description'),
        'urdf',
        'the_8th_leg.urdf.xacro'
    )
    urdf_string = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_string,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # Spawn the robot from the robot_description topic 
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'the_8th_leg'],
            output='screen'
        ),

        # Bridge Gazebo's clock into ROS as /clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

    ])