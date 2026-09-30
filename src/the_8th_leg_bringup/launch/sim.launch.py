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

        # Spawn the robot into Gazebo from the expanded xacro
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', urdf_string, '-name', 'the_8th_leg'],
            output='screen'
        ),

    ])