"""Localization + continuous trajectory + successful match markers (radar separate)."""
from pathlib import Path
from datetime import datetime
import json
import shutil
import hashlib
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration, EmitEvent
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from launch.events import Shutdown
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from keyframe_database import default_library

ROOT=Path(__file__).resolve().parents[1]


def save_run_metadata(context):
    folder=Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    path=folder/'run_metadata.json'
    if path.exists():
        raise RuntimeError(f'Experiment metadata already exists: {path}; use a new experiment directory')
    map_file=Path(LaunchConfiguration('map_pcd').perform(context)).expanduser().resolve()
    config=Path(LaunchConfiguration('config_path').perform(context)).expanduser().resolve()
    metadata=dict(map_pcd=str(map_file), config_path=str(config),
        record_inputs=LaunchConfiguration('record_inputs').perform(context),
        use_sim_time=LaunchConfiguration('use_sim_time').perform(context),
        created_local=datetime.now().astimezone().isoformat())
    metadata.update(initial_region=LaunchConfiguration('initial_region').perform(context),
        coarse_registration_enabled=False)
    metadata['external_initialization_enabled']=LaunchConfiguration('external_initialization_enabled').perform(context).lower()=='true'
    if metadata['external_initialization_enabled']:
        race_path=folder/'race_versions.json'
        metadata['initialization_mode']='race'
        metadata['race_versions_sha256']=hashlib.sha256(race_path.read_bytes()).hexdigest()
        metadata['race_versions']=json.loads(race_path.read_text())
    metadata['candidate_source']=LaunchConfiguration('candidate_source').perform(context)
    metadata['region_multistart_enabled']=LaunchConfiguration('region_multistart_enabled').perform(context)
    seed_file=Path(LaunchConfiguration('region_seed_file').perform(context)).expanduser().resolve()
    metadata['region_seed_file']=str(seed_file)
    if seed_file.is_file():
        shutil.copy2(seed_file, folder/'region_initialization_profile.yaml')
        metadata['region_seed_sha256']=hashlib.sha256(seed_file.read_bytes()).hexdigest()
    labels=Path(LaunchConfiguration('region_labels_csv').perform(context)).expanduser().resolve()
    metadata['region_labels_csv']=str(labels)
    if labels.is_file():
        shutil.copy2(labels, folder/'region_labels.csv')
    if map_file.is_file():
        metadata.update(map_size_bytes=map_file.stat().st_size, map_mtime_ns=map_file.stat().st_mtime_ns)
        digest=hashlib.sha256()
        with map_file.open('rb') as source:
            for chunk in iter(lambda:source.read(8*1024*1024), b''):
                digest.update(chunk)
        metadata['map_sha256']=digest.hexdigest()
    if metadata['region_multistart_enabled'].lower() == 'true':
        if not seed_file.is_file():
            raise RuntimeError('Region initializer profile is missing')
        profile=yaml.safe_load(seed_file.read_text())
        expected=profile.get('map_sha256')
        if expected and expected != metadata.get('map_sha256'):
            raise RuntimeError('Region initialization profile belongs to a different map SHA-256')
    if config.is_file():
        shutil.copy2(config, folder/'localization_launch_config.yaml')
    with path.open('x') as stream:
        json.dump(metadata,stream,ensure_ascii=False,indent=2)
        stream.write('\n')
    enabled=LaunchConfiguration('keyframe_initialization_enabled').perform(context).lower()=='true'
    actions=[]
    if enabled:
        from keyframe_database import KeyframeDatabase,validate_sensor_config
        library=Path(LaunchConfiguration('keyframe_library').perform(context)).expanduser().resolve()
        if metadata['candidate_source']=='bbs':
            from bbs_database import BbsDatabase
            bbs_config=Path(LaunchConfiguration('bbs_config').perform(context)).expanduser().resolve()
            database=BbsDatabase(map_file,bbs_config)
            database.close()
            from prepare_race_map_cache import ensure_cache
            actions.append(SetLaunchConfiguration('map_cache_directory',str(ensure_cache(map_file))))
            metadata.update(bbs_experimental=True,bbs_config=str(bbs_config),bbs_model=str(database.path),
                bbs_maximum_candidates=database.search_config['maximum_candidates'],
                bbs_min_initial_overlap_percent=database.search_config['minimum_initial_overlap_percent'])
            shutil.copy2(bbs_config,folder/'bbs_initialization.json')
            shutil.copy2(database.path/'manifest.json',folder/'bbs_model_manifest.json')
            actions.append(SetLaunchConfiguration('hierarchical_min_initial_overlap',str(database.search_config['minimum_initial_overlap_percent'])))
        elif metadata['candidate_source']=='virtual':
            from virtual_database import VirtualDatabase
            from prepare_race_map_cache import ensure_cache
            virtual_config=Path(LaunchConfiguration('virtual_config').perform(context)).expanduser().resolve()
            database=VirtualDatabase(map_file,virtual_config)
            actions.append(SetLaunchConfiguration('map_cache_directory',str(ensure_cache(map_file))))
            shutil.copy2(virtual_config,folder/'virtual_initialization.json')
            shutil.copy2(database.path/'manifest.json',folder/'virtual_manifest.json')
            metadata.update(virtual_templates=True,virtual_template_count=len(database.templates),virtual_station_count=database.manifest['stations'],virtual_shortlist_budget=len(database.index),reference_source='virtual_not_measured_keyframes')
            actions.append(SetLaunchConfiguration('hierarchical_min_initial_overlap','80.0'))
        else:
            database=KeyframeDatabase(library,map_file)
        validate_sensor_config(ROOT/'mid360_robot.yaml',database)
        if LaunchConfiguration('initial_region').perform(context) not in (str(database.region_id),f'space_{database.region_id}'):
            raise RuntimeError('Keyframe trial initial_region must name the library region')
        metadata.update(hierarchical_min_initial_overlap=float(LaunchConfiguration('hierarchical_min_initial_overlap').perform(context)),hierarchical_initialization_enabled=LaunchConfiguration('hierarchical_initialization_enabled').perform(context).lower()=='true',keyframe_trial_unvalidated=True,keyframe_initialization_enabled=True,keyframe_library=str(database.path),keyframe_index_sha256=database.library_sha,keyframe_profile_sha256=database.profile_sha,keyframe_count=len(database.index))
        if metadata['candidate_source']=='virtual': metadata['hierarchical_min_initial_overlap']=80.0
        if metadata['candidate_source']=='bbs': metadata['hierarchical_min_initial_overlap']=database.search_config['minimum_initial_overlap_percent']
        # Metadata was created above with x; update only this newly created run file.
        path.write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
        for key,value in [('keyframe_library_sha256',database.library_sha),('keyframe_profile_sha256',database.profile_sha),('keyframe_region_id',str(database.region_id)),('keyframe_expected_candidates',str(len(database.index)))]:
            actions.append(SetLaunchConfiguration(key,value))
        actions.append(SetLaunchConfiguration('visualizer_local_topic','/localization/selected_cloud'))
    return actions


def generate_launch_description():
    loc=Path(get_package_share_directory('kiss_matcher_localization'))
    vis=Path(get_package_share_directory('localization_visualizer'))
    recorder=Path(get_package_share_directory('localization_path_recorder'))
    args=[DeclareLaunchArgument('map_cache_directory',default_value=''),DeclareLaunchArgument('race_bbs_profile_sha256',default_value=''),DeclareLaunchArgument('external_initialization_enabled',default_value='false'),DeclareLaunchArgument('race_bbs_sha256',default_value=''),DeclareLaunchArgument('race_keyframes_sha256',default_value=''),DeclareLaunchArgument('race_profile_sha256',default_value=''),DeclareLaunchArgument('visualizer_local_topic',default_value='/fast_lio/local_map'),
          DeclareLaunchArgument('candidate_source',default_value='keyframes'),
          DeclareLaunchArgument('virtual_config',default_value=str(ROOT/'config/virtual_initialization_space_10.json')),
          DeclareLaunchArgument('bbs_config',default_value=str(ROOT/'config/bbs_initialization.json')),
          DeclareLaunchArgument('hierarchical_min_initial_overlap',default_value='80.0'),
          DeclareLaunchArgument('hierarchical_initialization_enabled',default_value='false'),
          DeclareLaunchArgument('keyframe_initialization_enabled',default_value='false'),
          DeclareLaunchArgument('keyframe_library',default_value=str(default_library())),
          DeclareLaunchArgument('keyframe_library_sha256',default_value=''),
          DeclareLaunchArgument('keyframe_profile_sha256',default_value=''),
          DeclareLaunchArgument('keyframe_region_id',default_value='2'),
          DeclareLaunchArgument('keyframe_expected_candidates',default_value='28'),
          DeclareLaunchArgument('map_pcd',default_value=str(loc/'maps/building_split_space.pcd')),
          DeclareLaunchArgument('region_labels_csv',default_value=str(loc/'maps/region_labels.csv')),
          DeclareLaunchArgument('initial_region',default_value=''),
          DeclareLaunchArgument('region_multistart_enabled',default_value='false'),
          DeclareLaunchArgument('region_seed_file',default_value=str(ROOT/'config/region_initialization_space.yaml')),
          DeclareLaunchArgument('config_path',default_value=str(ROOT/'config/continuous_localization.yaml')),
          DeclareLaunchArgument('rviz',default_value='true'),
          DeclareLaunchArgument('use_sim_time',default_value='false'),
          DeclareLaunchArgument('record_inputs',default_value='false'),
          DeclareLaunchArgument('output',default_value=str(ROOT/'slam_output/localization_sessions'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')))]
    localizer=IncludeLaunchDescription(AnyLaunchDescriptionSource(str(loc/'launch/navigation_localization.launch.yaml')),
        launch_arguments={'map_pcd':LaunchConfiguration('map_pcd'),
                          **{k:LaunchConfiguration(k) for k in ['map_cache_directory','race_bbs_profile_sha256','external_initialization_enabled', 'race_bbs_sha256', 'race_keyframes_sha256', 'race_profile_sha256','candidate_source','hierarchical_min_initial_overlap','hierarchical_initialization_enabled','keyframe_initialization_enabled','keyframe_library_sha256','keyframe_profile_sha256','keyframe_region_id','keyframe_expected_candidates']},
                          'config_path':LaunchConfiguration('config_path'),
                          'region_labels_csv':LaunchConfiguration('region_labels_csv'),
                          'initial_region':LaunchConfiguration('initial_region'),
                          'region_multistart_enabled':LaunchConfiguration('region_multistart_enabled'),
                          'region_seed_file':LaunchConfiguration('region_seed_file'),
                          'local_map_topic':'/fast_lio/local_map',
                          'local_map_reliability':'reliable',
                          'metrics_csv':[LaunchConfiguration('output'),'/matching_metrics.csv'],
                          'record_inputs':LaunchConfiguration('record_inputs')}.items())
    visualizer=IncludeLaunchDescription(AnyLaunchDescriptionSource(str(vis/'launch/localization_visualizer.launch.yaml')),
        launch_arguments={'map_pcd':LaunchConfiguration('map_pcd'),
                          'local_map_topic':LaunchConfiguration('visualizer_local_topic'),
                          'local_map_reliability':'reliable',
                          'start_rviz':LaunchConfiguration('rviz'),
                          'rviz_path':str(ROOT/'rviz/continuous_localization.rviz')}.items())
    tf_path=IncludeLaunchDescription(AnyLaunchDescriptionSource(
        str(recorder/'launch/localization_path_recorder.launch.yaml')),
        launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time')}.items())
    # The left display canvas uses map coordinates; the right raw cloud is shifted
    # by the visualizer only. This identity link is for RViz, not a pose estimate.
    canvas_tf=Node(package='tf2_ros', executable='static_transform_publisher',
        name='comparison_canvas_tf', arguments=['--frame-id', 'map',
        '--child-frame-id', 'localization_comparison', '--x', '0', '--y', '0',
        '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0'], output='own_log')
    trace=ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/localization_trace.py'),
        '--output',LaunchConfiguration('output'),'--ros-args','-p',
        ['use_sim_time:=',LaunchConfiguration('use_sim_time')]],output='own_log')
    retriever=ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/keyframe_retriever.py'),
        '--library',LaunchConfiguration('keyframe_library'),'--map',LaunchConfiguration('map_pcd'),
        '--output',LaunchConfiguration('output'),'--ros-args','-p',['use_sim_time:=',LaunchConfiguration('use_sim_time')]],
        condition=IfCondition(PythonExpression(["'",LaunchConfiguration('keyframe_initialization_enabled'),"'.lower() == 'true' and '",LaunchConfiguration('candidate_source'),"' == 'keyframes'"])),output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='keyframe retriever exited'))])
    bbs_retriever=ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/bbs_retriever.py'),
        '--config',LaunchConfiguration('bbs_config'),'--map',LaunchConfiguration('map_pcd'),
        '--output',LaunchConfiguration('output'),'--ros-args','-p',['use_sim_time:=',LaunchConfiguration('use_sim_time')]],
        condition=IfCondition(PythonExpression(["'",LaunchConfiguration('keyframe_initialization_enabled'),"'.lower() == 'true' and '",LaunchConfiguration('candidate_source'),"' == 'bbs'"])),output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='BBS retriever exited'))])
    virtual_retriever=ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/virtual_retriever.py'),
        '--config',LaunchConfiguration('virtual_config'),'--map',LaunchConfiguration('map_pcd'),
        '--output',LaunchConfiguration('output'),'--ros-args','-p',['use_sim_time:=',LaunchConfiguration('use_sim_time')]],
        condition=IfCondition(PythonExpression(["'",LaunchConfiguration('keyframe_initialization_enabled'),"'.lower() == 'true' and '",LaunchConfiguration('candidate_source'),"' == 'virtual'"])),output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='virtual retriever exited'))])
    return LaunchDescription(args+[OpaqueFunction(function=save_run_metadata),
                                   SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
                                   retriever,bbs_retriever,virtual_retriever,localizer,visualizer,tf_path,canvas_tf,trace])
