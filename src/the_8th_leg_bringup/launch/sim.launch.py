from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    controllers_file = os.path.join(
        get_package_share_directory('the_8th_leg_bringup'),
        'config',
        'controllers.yaml'
    )
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Publish robot_description on sim time
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_string, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn the robot from the topic
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'the_8th_leg'],
        output='screen'
    )

    # Bridge the Gazebo clock into ROS
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Controller spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', controllers_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        clock_bridge,

        # Robot existing to start the joint state broadcaster
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
        ),
        # Broadcaster up and start the arm controller
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
        ),
    ])