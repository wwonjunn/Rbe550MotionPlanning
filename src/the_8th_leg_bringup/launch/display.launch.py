from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    xacro_file = os.path.join(
        get_package_share_directory('the_8th_leg_description'),
        'urdf',
        'the_8th_leg.urdf.xacro'
    )
    urdf_string = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(
        get_package_share_directory('the_8th_leg_bringup'),
        'rviz',
        'display.rviz'
    )

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
                    arguments=['-d', rviz_config],
                    output='screen'
                )
            ]
        ),

    ])