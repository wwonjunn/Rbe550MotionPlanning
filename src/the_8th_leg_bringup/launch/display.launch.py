from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    urdf_file = os.path.join(
        get_package_share_directory('the_8th_leg_description'),
        'urdf',
        'the_8th_leg.urdf'
    )
    with open(urdf_file, 'r') as f:
        urdf_string = f.read()

    return LaunchDescription([
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_string}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen'
                )
            ]
        )

])