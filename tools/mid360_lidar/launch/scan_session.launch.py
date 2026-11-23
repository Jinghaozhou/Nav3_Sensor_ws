"""One supervised scan. Each component is stopped in reverse order before bag close."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]


def start(context):
    command = ['/usr/bin/python3', str(ROOT / 'scripts/start_scan.py'),
               '--mode', LaunchConfiguration('mode').perform(context),
               '--region', LaunchConfiguration('region').perform(context),
               '--bag-root', LaunchConfiguration('bag_root').perform(context)]
    if LaunchConfiguration('rviz').perform(context).lower() == 'false':
        command.append('--no-rviz')
    if LaunchConfiguration('render').perform(context).lower() == 'false':
        command.append('--no-render')
    return [ExecuteProcess(cmd=command, output='screen', sigterm_timeout='600', sigkill_timeout='600')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='building5'),
        DeclareLaunchArgument('region', default_value='space_2'),
        DeclareLaunchArgument('bag_root', default_value=str(ROOT / 'bags')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('render', default_value='true'),
        OpaqueFunction(function=start),
    ])
