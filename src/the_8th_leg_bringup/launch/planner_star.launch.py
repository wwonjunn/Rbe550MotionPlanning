from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('the_8th_leg_description'),
        'urdf',
        'the_8th_leg.urdf.xacro'
    )
    moveit_config = (
        MoveItConfigsBuilder('the_8th_leg', package_name='the_8th_leg_moveit_config')
        .robot_description(file_path=xacro_file)
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package='the_8th_leg_planner',
            executable='rrt_planner_star',
            parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
            output='screen'
        )
    ])