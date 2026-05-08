import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws_root = os.environ.get('ROS2_WS', '/ros2_ws')

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value=os.path.join(ws_root, 'scenarios', 'simple_tank.scn'),
        description='Path to Stonefish scenario file',
    )
    rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value='1000.0',
        description='Physics simulation rate [Hz]',
    )

    sim_node = Node(
        package='stonefish_ros2',
        executable='parsed_simulator',
        name='stonefish_simulator',
        parameters=[{
            'simulation_rate': LaunchConfiguration('simulation_rate'),
            'scenario_description': LaunchConfiguration('scenario'),
        }],
        output='screen',
    )

    return LaunchDescription([
        scenario_arg,
        rate_arg,
        sim_node,
    ])
