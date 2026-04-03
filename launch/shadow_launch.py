from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    pkg_share = os.path.join(os.environ['HOME'], 'shadow_ws', 'src', 'shadow_drone')

    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'shadow_drone.urdf')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )

    spawn_drone = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'shadow_drone',
                    '-file', urdf_path,
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_drone
    ])
