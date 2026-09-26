from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    urdf_file = os.path.join(
        get_package_share_directory('the_8th_leg_description'),
        'urdf',
        'the_8th_leg.urdf'
    )

    return LaunchDescription([
        
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the robot URDF into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_file, '-name', 'the_8th_leg'],
            output='screen'
        ),

    ])