from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bringup_dir = get_package_share_directory('the_8th_leg_bringup')
    moveit_dir = get_package_share_directory('the_8th_leg_moveit_config')
    xacro_file = os.path.join(
        get_package_share_directory('the_8th_leg_description'),
        'urdf',
        'the_8th_leg.urdf.xacro'
    )

    # Collect every MoveIt config file (URDF, SRDF, kinematics, planners, controllers)
    moveit_config = (
        MoveItConfigsBuilder('the_8th_leg', package_name='the_8th_leg_moveit_config')
        .robot_description(file_path=xacro_file)
        .to_moveit_configs()
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'sim.launch.py'))
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(moveit_dir, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True},
        ],
        output='screen'
    )

    def start_rviz_when_ready(event):
        if b'You can start planning now' in event.text:
            return [rviz]
        
    return LaunchDescription([
    sim,
    move_group,
    RegisterEventHandler(
        OnProcessIO(
            target_action=move_group,
            on_stdout=start_rviz_when_ready,
            on_stderr=start_rviz_when_ready,
        )
    ),
])