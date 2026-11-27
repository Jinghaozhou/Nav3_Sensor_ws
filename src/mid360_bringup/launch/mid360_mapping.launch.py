"""MID360 mapping with an optional Go2 body origin and nominal sensor mount."""
import json
from pathlib import Path
from datetime import datetime
import re
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mid360_bringup.mount_tf import go2_transforms, static_arguments


TOPIC_KEYS = ('lidar', 'imu', 'odom', 'local_cloud', 'body_cloud',
              'effect_cloud', 'laser_map', 'path', 'local_map', 'health',
              'gravity', 'local_map_global', 'window_status', 'localization_status',
              'keyframe_candidates', 'keyframe_status')
NEW_TOPIC_DEFAULTS = {
    'gravity': '/fast_lio/gravity',
    'local_map_global': '/fast_lio/local_map_global',
    'window_status': '/fast_lio/window_status',
    'localization_status': '/localization/status',
    'keyframe_candidates': '/localization/keyframe_candidates',
    'keyframe_status': '/localization/keyframe_status',
}
ABSOLUTE_TOPIC = re.compile(r'^/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$')


def default_keyframe_map():
    share = Path(get_package_share_directory('kiss_matcher_localization'))
    name = 'building_split_space.pcd'
    paths = [share/'maps'/name]
    paths.extend(parent/'src/localization/maps'/name for parent in share.parents)
    for path in paths:
        if path.is_file():
            return str(path)
    raise FileNotFoundError('KISS building_split_space.pcd is absent from installed and source maps; set keyframe_map')


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    enabled = lambda name: value(name).lower() == 'true'
    config = Path(value('config_file')).expanduser().resolve()
    config_data = yaml.safe_load(config.read_text())
    params = config_data['/**']['ros__parameters']
    topics = config_data['mid360_bringup']['ros__parameters']['topics']
    # Existing custom YAML files remain usable after adding the new interfaces.
    topics = {**NEW_TOPIC_DEFAULTS, **topics}
    for key in TOPIC_KEYS:
        name = topics.get(key)
        if not isinstance(name, str) or not ABSOLUTE_TOPIC.fullmatch(name):
            raise ValueError(f'topics.{key} must be an absolute ROS topic name: {name!r}')
    if len({topics[key] for key in TOPIC_KEYS}) != len(TOPIC_KEYS):
        raise ValueError('Each configured topic name must be unique')
    if params['mapping']['extrinsic_est_en']:
        raise RuntimeError('Unified static TF requires mapping.extrinsic_est_en: false')
    translation = np.asarray(params['mapping']['extrinsic_T'], dtype=float)
    rotation = np.asarray(params['mapping']['extrinsic_R'], dtype=float).reshape(3, 3)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('extrinsic_T must contain three finite values')
    if not np.isfinite(rotation).all() or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-4):
        raise ValueError('extrinsic_R must be a proper rotation matrix')
    go2 = value('robot_model') == 'go2'
    body_frame = 'imu_link' if go2 else 'base_link'
    if go2:
        mount = yaml.safe_load(Path(value('mount_config')).expanduser().read_text())
        imu_base, base_lidar = go2_transforms(translation, rotation, mount)
        # The SDK can pre-transform raw points. The mount belongs in TF only.
        driver_config = json.loads(Path(value('driver_config')).expanduser().read_text())
        for lidar in driver_config['lidar_configs']:
            extrinsic = lidar['extrinsic_parameter']
            if any(float(extrinsic[k]) != 0 for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')):
                raise ValueError('Go2 TF requires zero Livox driver extrinsic_parameter')
    output = Path(value('map_output_dir')).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    map_path = output / ('mid360_map_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.pcd')
    sim = enabled('use_sim_time')
    common = {'use_sim_time': sim}
    nodes = []
    if enabled('driver'):
        if sim:
            raise RuntimeError('Bag playback requires driver:=false use_sim_time:=true')
        nodes.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', parameters=[{
                'xfer_format': 1, 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0,
                'frame_id': 'lidar', 'imu_frame_id': 'imu_link',
                'user_config_path': str(Path(value('driver_config')).expanduser().resolve()),
                'cmdline_input_bd_code': 'livox0000000001',
                'lvx_file_path': '/home/livox/livox_test.lvx'}],
            remappings=[('/livox/lidar', topics['lidar']),
                        ('/livox/imu', topics['imu'])]))
    nodes.append(Node(package='fast_lio', executable='fastlio_mapping', output='screen',
        parameters=[str(config), common, {'world_frame':'odom', 'body_frame':body_frame,
            'common.lid_topic':topics['lidar'], 'common.imu_topic':topics['imu'],
            'map_file_path':str(map_path)}],
        remappings=[('/Odometry',topics['odom']),
                    ('/cloud_registered',topics['local_cloud']),
                    ('/cloud_registered_body',topics['body_cloud']),
                    ('/cloud_effected',topics['effect_cloud']),
                    ('/Laser_map',topics['laser_map']),
                    ('/fast_lio/gravity',topics['gravity']),
                    ('/path',topics['path'])]))
    if go2:
        transforms = [('imu_base_tf', 'imu_link', 'base_link', imu_base),
                      ('base_lidar_tf', 'base_link', 'lidar', base_lidar)]
    else:
        imu_lidar = np.eye(4)
        imu_lidar[:3, :3], imu_lidar[:3, 3] = rotation, translation
        transforms = [('base_imu_tf', 'base_link', 'imu_link', np.eye(4)),
                      ('base_lidar_tf', 'base_link', 'lidar', imu_lidar)]
    for name, parent, child, transform in transforms:
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                          parameters=[common], arguments=static_arguments(parent, child, transform)))
    if enabled('monitor'):
        nodes.append(Node(package='mid360_bringup', executable='livox_monitor', parameters=[common], output='screen',
            arguments=['--lidar-topic',topics['lidar'],'--imu-topic',topics['imu'],
                       '--odom-topic',topics['odom'],'--health-topic',topics['health']]))
    if enabled('local_map'):
        window_args = ['--window-time',value('window_time'),
                       '--tracking-window-time',value('tracking_window_time'),
                       '--window-frames',value('window_frames'),
                       '--max-buffer-points',value('max_buffer_points'),
                       '--voxel',value('local_map_voxel'),'--rate',value('local_map_rate'),
                       '--timing-csv',value('local_map_timing_csv'),
                       '--recovery-failures',value('recovery_failures'),
                       '--in-topic',topics['local_cloud'],'--out-topic',topics['local_map'],
                       '--global-topic',topics['local_map_global'],
                       '--window-status-topic',topics['window_status'],
                       '--localization-status-topic',topics['localization_status']]
        if enabled('adaptive_window'):
            window_args.append('--adaptive')
        nodes.append(Node(package='mid360_bringup', executable='local_map_window', parameters=[common], output='screen',
            arguments=window_args))
    if enabled('keyframe_candidates'):
        if topics['keyframe_candidates'] != '/localization/keyframe_candidates':
            raise ValueError('KISS currently subscribes to /localization/keyframe_candidates; keep this topic name')
        map_file = value('keyframe_map')
        if not map_file:
            map_file = default_keyframe_map()
        map_file = Path(map_file).expanduser().resolve()
        library = Path(value('keyframe_library')).expanduser().resolve()
        if not map_file.is_file() or not library.is_dir():
            raise FileNotFoundError(f'Keyframe map or library missing: {map_file}, {library}')
        keyframe_args = ['--library',str(library), '--map',str(map_file),
                         '--sensor-config',str(config), '--odom-body-frame',body_frame,
                         '--cloud-topic',topics['local_cloud'], '--odom-topic',topics['odom'],
                         '--gravity-topic',topics['gravity'],
                         '--candidates-topic',topics['keyframe_candidates'],
                         '--status-topic',topics['keyframe_status']]
        if value('keyframe_output_dir'):
            keyframe_args += ['--output',value('keyframe_output_dir')]
        nodes.append(Node(package='mid360_bringup', executable='keyframe_retriever',
                          parameters=[common], output='screen', arguments=keyframe_args))
    if enabled('rviz'):
        rviz_topics = {'odom':'/fast_lio/odom', 'path':'/path',
                       'laser_map':'/Laser_map', 'local_map':'/fast_lio/local_map',
                       'local_cloud':'/fast_lio/local_cloud'}
        nodes.append(Node(package='rviz2', executable='rviz2', parameters=[common],
            arguments=['-d',value('rviz_config')],
            remappings=[(default, topics[key]) for key, default in rviz_topics.items()
                        if default != topics[key]]))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_bringup'))
    defaults = {
        'driver':('true','Start physical lidar driver; false for bags'),
        'use_sim_time':('false','Use bag /clock'), 'rviz':('true','Start RViz'),
        'monitor':('true','Start health monitor'), 'local_map':('true','Start sliding local map'),
        'config_file':(str(share/'config/mid360_robot.yaml'),'Absolute or current-directory-relative YAML path'),
        'driver_config':(str(share/'config/MID360_config.json'),'Lidar network JSON path'),
        'robot_model':('imu','imu: legacy base_link at MID360 IMU; go2: Unitree body origin'),
        'mount_config':(str(share/'config/go2_mount.yaml'),'Go2 base_link to raw lidar nominal mount'),
        'rviz_config':(str(share/'rviz/fastlio_map.rviz'),'RViz configuration'),
        'map_output_dir':(str(Path.home()/'Nav3_Sensor_ws/maps'),'Writable map output directory'),
        'window_time':('120.0','Long local map time window seconds'),
        'tracking_window_time':('30.0','Tracking local map time window seconds'),
        'window_frames':('1500','Maximum retained frame count'),
        'max_buffer_points':('3000000','Maximum retained point count'),
        'local_map_voxel':('0.15','Local map voxel size meters'),
        'local_map_rate':('1.0','Local map publication Hz'),
        'local_map_timing_csv':('','Optional publication timing CSV for local map'),
        'adaptive_window':('true','Use KISS localization status to switch windows when available'),
        'recovery_failures':('3','Consecutive localization failures before using the long window'),
        'keyframe_candidates':('false','Publish external keyframe candidates for KISS keyframe initialization'),
        'keyframe_library':(str(share/'reference_maps/keyframe_libraries/space_2/merged_20261111'),
                            'Keyframe reference-library directory'),
        'keyframe_map':('','Reference map PCD; empty searches KISS installed and source maps'),
        'keyframe_output_dir':('','Optional directory for keyframe query CSV')}
    boolean = {'driver','use_sim_time','rviz','monitor','local_map','adaptive_window',
               'keyframe_candidates'}
    return LaunchDescription([DeclareLaunchArgument(n,default_value=v,description=d,
        **({'choices':['true','false']} if n in boolean else
           {'choices':['imu','go2']} if n == 'robot_model' else {})) for n,(v,d) in defaults.items()]
        + [OpaqueFunction(function=start)])
