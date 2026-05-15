import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


STONEFISH_DATA = '/usr/local/share/Stonefish'
WS_ROOT = os.environ.get('ROS2_WS', '/ros2_ws')


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
    # GPU version only: window size and rendering quality
    width_arg  = DeclareLaunchArgument('window_width',  default_value='800')
    height_arg = DeclareLaunchArgument('window_height', default_value='600')
    quality_arg = DeclareLaunchArgument(
        'quality',
        default_value='low',
        description='Rendering quality: low / medium / high',
    )

    # GPU: stonefish_simulator  <dataPath> <scenarioPath> <rate> <W> <H> <quality>
    sim_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[
            STONEFISH_DATA,
            LaunchConfiguration('scenario'),
            LaunchConfiguration('simulation_rate'),
            LaunchConfiguration('window_width'),
            LaunchConfiguration('window_height'),
            LaunchConfiguration('quality'),
        ],
        output='screen',
    )

    # Converts stonefish_ros2/DVL → nav_msgs/Odometry (/bluerov2/dvl) for AQUA-SLAM.
    # Runs here because stonefish_ros2 is not installed in the AQUA-SLAM container.
    dvl_converter = ExecuteProcess(
        cmd=['python3', os.path.join(WS_ROOT, 'scripts', 'sim_dvl_converter.py')],
        output='screen',
    )

    # Adds acoustic noise to the raw multibeam2d cloud from Stonefish:
    # dropout, angular jitter, Rayleigh intensity → /sonar_3d15/points (PointCloud2).
    sonar_converter = ExecuteProcess(
        cmd=['python3', os.path.join(WS_ROOT, 'scripts', 'sim_sonar_3d15_converter.py')],
        output='screen',
    )

    return LaunchDescription([
        scenario_arg,
        rate_arg,
        width_arg,
        height_arg,
        quality_arg,
        sim_node,
        dvl_converter,
        sonar_converter,
    ])
