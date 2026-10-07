from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('a2_demo'))
    arguments = [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(share/'rviz/demo.rviz')),
        DeclareLaunchArgument('motion_seconds', default_value='1.4'),
        DeclareLaunchArgument('process_seconds', default_value='5.0'),
        DeclareLaunchArgument('busy_seconds', default_value='0.0'),
        DeclareLaunchArgument('stall', default_value='false'),
        DeclareLaunchArgument('station_timeout', default_value='12.0'),
        DeclareLaunchArgument('process_timeout', default_value='12.0'),
        DeclareLaunchArgument('output_dir', default_value=str(Path.cwd() / 'results')),
    ]
    def value(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    return LaunchDescription(arguments + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share/'urdf/scara.urdf').read_text(encoding='utf-8')}]),
        Node(package='a2_demo', executable='arm_sim', output='screen'),
        Node(package='a2_demo', executable='machine_sim', output='screen', parameters=[{
            'process_seconds': value('process_seconds', float),
            'busy_seconds': value('busy_seconds', float), 'stall': value('stall', bool)}]),
        Node(package='a2_demo', executable='task_manager', output='screen', parameters=[{
            'motion_seconds': value('motion_seconds', float),
            'station_timeout': value('station_timeout', float),
            'process_timeout': value('process_timeout', float),
            'output_dir': value('output_dir', str)}]),
        Node(package='a2_demo', executable='scene_view', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
