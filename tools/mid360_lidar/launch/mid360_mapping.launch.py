"""Mid-360 实时建图一键启动:驱动 + Fast-LIO2 + rviz2 + 静态 TF。

接口与《三维导航设计文档》对齐:
  - 话题: /lidar_points(驱动 CustomMsg)、/imu(驱动,200Hz)
  - frame: 点云 lidar、IMU imu_link
  - Fast-LIO2 输出: /fast_lio/odom、/fast_lio/local_cloud、/Laser_map,
    TF odom→base_link(里程计);本 launch 补 base_link→lidar、base_link→imu_link 静态 TF
  - rviz2 用项目配置(只看 SLAM 处理后的点云)

用法(代替 README 4.1 的双终端流程):
    ros2 launch mid360_lidar mid360_mapping.launch.py
    参数: use_sim_time:=true(bag 回放时)、rviz:=false、config_file:=xxx.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_use = LaunchConfiguration('rviz')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='bag 回放时置 true(配合 ros2 bag play --clock)')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='是否弹 rviz2')
    declare_config_file = DeclareLaunchArgument(
        'config_file', default_value='mid360_robot.yaml',
        description='Fast-LIO2 配置文件(相对项目根目录)')

    declare_window_frames = DeclareLaunchArgument('window_frames', default_value='1500')

    # ---- 驱动(参数与 livox_ros_driver2 的 msg_MID360_launch.py 一致)----
    driver_params = [
        {'xfer_format': 1},       # 0-PointCloud2, 1-CustomMsg
        {'multi_topic': 0},
        {'data_src': 0},          # 0-真实雷达
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'lidar'},        # 设计文档:点云 frame = lidar
        {'imu_frame_id': 'imu_link'},  # 设计文档:IMU frame = imu_link(驱动已打补丁支持)
        {'user_config_path': os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'config', 'MID360_config.json')},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]
    driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=driver_params,
        remappings=[('/livox/lidar', '/lidar_points'),
                    ('/livox/imu', '/imu')],
    )

    # ---- Fast-LIO2(配置直接读项目文件,不用再拷 install 空间)----
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([PROJECT_DIR, config_file]),
                    {'use_sim_time': use_sim_time}],
        remappings=[('/livox/lidar', '/lidar_points'),
                    ('/livox/imu', '/imu')],
        output='screen',
    )

    # ---- rviz2(项目干净视图:只显示 SLAM 处理后的点云)----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(PROJECT_DIR, 'rviz', 'fastlio_map.rviz')],
        output='own_log',
        condition=IfCondition(rviz_use),
    )

    # ---- 健康监控(FL-08):/fast_lio/health + 频率/资源日志 ----
    # 用系统 python3 跑(conda 的 python 没有 rclpy)
    monitor = ExecuteProcess(
        cmd=['/usr/bin/python3',
             os.path.join(PROJECT_DIR, 'scripts', 'livox_monitor.py')],
        output='screen',
    )

    # ---- 局部地图滑动窗口(组会 2026-09-29):/fast_lio/local_map ----
    # 按帧存储 FIFO 窗口,供 KISS-Matcher 重定位/SCAN 使用
    local_map_window = ExecuteProcess(
        cmd=['/usr/bin/python3',
             os.path.join(PROJECT_DIR, 'scripts', 'local_map_window.py'),
             '--window-time', LaunchConfiguration('initial_window'),
             '--tracking-window-time', LaunchConfiguration('tracking_window'),
             '--window-frames', LaunchConfiguration('window_frames'), '--rate', '1', '--adaptive',
             '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )

    # ---- 静态 TF ----
    # base_link 即 Fast-LIO 的 IMU 本体系;imu_link 是它的别名
    tf_base_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )
    # base_link→lidar:与 mid360_robot.yaml 的 extrinsic_T 同值同号
    # (LiDAR 原点在 IMU 系下的平移);外参在线估计稳定固化后同步更新这里
    tf_base_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['-0.011', '-0.02329', '0.04412', '0', '0', '0',
                   'base_link', 'lidar'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('initial_window', default_value='120',
            description='Initialization/recovery history seconds (candidate, not validated)'),
        DeclareLaunchArgument('tracking_window', default_value='30',
            description='Short prior-tracking window seconds (candidate, not validated)'),
        declare_use_sim_time,
        declare_rviz,
        declare_config_file,
        declare_window_frames,
        driver,
        fast_lio,
        rviz,
        tf_base_imu,
        tf_base_lidar,
        monitor,
        local_map_window,
    ])
