"""BBS and virtual-template initialization race; a single owner tracks/publishes TF."""
import json
from datetime import datetime
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from keyframe_database import validate_sensor_config
from virtual_database import VirtualDatabase
from region_profiles import region_number, virtual_config_for_region, dynamic_profile
from bbs_database import BbsDatabase, bbs_config_for_region


def setup(context):
    value=lambda k:LaunchConfiguration(k).perform(context)
    region_id=region_number(value('initial_region'));region=f'space_{region_id}'
    output=Path(value('output')).expanduser().resolve();output.mkdir(parents=True,exist_ok=True)
    map_path=Path(value('map_pcd')).resolve()
    virtual_config=virtual_config_for_region(region);bbs_config=bbs_config_for_region(region)
    db=VirtualDatabase(map_path,virtual_config)
    bbs=BbsDatabase(map_path,bbs_config)
    try:
        validate_sensor_config(ROOT/'mid360_robot.yaml',db)
        validate_sensor_config(ROOT/'mid360_robot.yaml',bbs)
    finally:
        bbs.close()
    from prepare_race_map_cache import ensure_cache
    cache=ensure_cache(map_path)
    versions=dict(sources=['bbs','virtual'],map_sha256=db.map_sha,virtual_sha256=db.library_sha,bbs_sha256=bbs.library_sha,
        virtual_profile_sha256=db.profile_sha,bbs_profile_sha256=bbs.profile_sha,
        virtual_model=str(db.path),virtual_config=str(virtual_config),bbs_config=str(bbs_config),bbs_model=str(bbs.path),
        region=region,policy='first independently two-window-confirmed safe proposal wins',field_validated=False)
    path=output/'race_versions.json'
    with path.open('x') as f:json.dump(versions,f,indent=2)
    # Preserve exact branch profiles without confusing virtual templates with measured frames.
    import shutil
    for source,database,config in [('bbs',bbs,bbs_config),('virtual',db,virtual_config)]:
        shutil.copy2(config,output/f'race_{source}_config.json')
        shutil.copy2(database.path/'manifest.json',output/f'race_{source}_manifest.json')
    shared=dict(map_cache_directory=str(cache),map_pcd=str(map_path),region_labels_csv=str(__import__('workspace_paths').labels_file()),
        region_multistart_enabled=True,region_seed_file=str(dynamic_profile()),
        local_map_topic='/fast_lio/local_map',local_map_reliability='reliable',use_prior_tracking=False,
        keyframe_initialization_enabled=True,keyframe_region_id=region_id,initial_region=region,
        publish_tf=False,race_validator=True,periodic_interval_sec=0.,vgicp_threads=2,
        hierarchical_min_initial_overlap=80.,use_sim_time=value('use_sim_time').lower()=='true')
    actions=[]
    for source,database,script in [('bbs',bbs,'bbs_retriever.py'),('virtual',db,'virtual_retriever.py')]:
        folder=output/'race'/source;folder.mkdir(parents=True,exist_ok=True)
        ns='/localization_race/'+source
        params=dict(shared,candidate_source=source,keyframe_expected_candidates=len(database.index),
            keyframe_profile_sha256=database.profile_sha,keyframe_library_sha256=database.library_sha,hierarchical_initialization_enabled=source=='virtual',
            metrics_csv=str(folder/'matching_metrics.csv'),publish_attempt_inputs=True)
        actions.append(Node(package='kiss_matcher_localization',executable='navigation_localizer',namespace=ns,
            name='navigation_localizer',parameters=[str(ROOT/'config/continuous_localization.yaml'),params],
            remappings=[('/localization/keyframe_candidates',ns+'/keyframe_candidates')],output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='private race validator exited'))]))
        cmd=['/usr/bin/python3',str(ROOT/'scripts'/script),'--map',str(map_path),'--output',str(folder),'--race-controlled']
        cmd+=['--config',str(bbs_config if source=='bbs' else virtual_config)]
        cmd+=['--ros-args','-p','use_sim_time:='+value('use_sim_time'),'-r','/localization/keyframe_candidates:='+ns+'/keyframe_candidates',
            '-r','/localization/keyframe_status:='+ns+'/query_status','-r','/localization/status:='+ns+'/status']
        actions.append(ExecuteProcess(cmd=cmd,output='screen',on_exit=[EmitEvent(event=Shutdown(reason=source+' query producer exited'))]))
    owner={k:value(k) for k in ('output','rviz','use_sim_time','record_inputs','map_pcd','initial_region')}
    owner.update(map_cache_directory=str(cache),region_multistart_enabled='true',external_initialization_enabled='true',
        region_seed_file=str(dynamic_profile()),
        race_bbs_profile_sha256=bbs.profile_sha,race_bbs_sha256=bbs.library_sha,race_virtual_sha256=db.library_sha,race_virtual_profile_sha256=db.profile_sha)
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'launch/continuous_localization.launch.py')),launch_arguments=owner.items()))
    actions.append(ExecuteProcess(cmd=['/usr/bin/python3',str(ROOT/'scripts/initialization_race.py'),'--versions',str(path),'--output',str(output),
        '--ros-args','-p','use_sim_time:='+value('use_sim_time')],output='screen',on_exit=[EmitEvent(event=Shutdown(reason='race coordinator exited'))]))
    return actions


def generate_launch_description():
    defaults=dict(output=str(ROOT/'slam_output/localization_sessions'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')),rviz='true',use_sim_time='false',record_inputs='true',
        map_pcd=str(__import__('workspace_paths').map_file()),initial_region='space_2')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
