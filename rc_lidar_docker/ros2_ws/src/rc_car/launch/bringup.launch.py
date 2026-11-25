"""전체 실행: 라이다(sllidar C1) + 정적 TF + 차량제어 + 라이다 Odometry + Command Manager + 웹 브리지 + SLAM Toolbox

  ros2 launch rc_car bringup.launch.py                       # 새 지도 작성 (mapping)
  ros2 launch rc_car bringup.launch.py map:=/rc_lidar_docker/maps/room1               # 저장 지도 이어서 작성
  ros2 launch rc_car bringup.launch.py slam_mode:=localization map:=/rc_lidar_docker/maps/room1  # 위치추정만
  ros2 launch rc_car bringup.launch.py slam:=false lidar:=false                        # 차량/웹만 (하드웨어 점검)

기존 config/slam.yaml 은 수정하지 않고, 프레임 이름(odom/base_link/map)과 스캔 토픽만 덮어쓴 사본을 /tmp 에 만들어 사용.
odom → base_link 는 엔코더가 아니라 odometry_node(라이다 스캔매칭)가 발행하고, SLAM 은 차체·트레일러를
제외한 /scan_filtered 를 입력으로 쓴다.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    cfg_path = LaunchConfiguration('config').perform(context)
    with open(cfg_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    use_lidar = LaunchConfiguration('lidar').perform(context).lower() == 'true'
    use_slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    params = [{'config_file': cfg_path}]
    actions = []

    lc = cfg['lidar']
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_base_to_laser',
        arguments=['--x', str(lc['x']), '--y', str(lc['y']), '--z', str(lc['z']), '--yaw', str(lc['yaw']),
                   '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', lc['frame_id']]))
    for s in cfg['ultrasonic']['sensors']:
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=f'tf_base_to_us_{s["name"]}',
            arguments=['--x', str(s.get('x', 0.0)), '--y', str(s.get('y', 0.0)), '--z', '0.05',
                       '--yaw', str(s.get('yaw', 0.0)), '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', f'ultrasonic_{s["name"]}']))

    for exe in ('vehicle_node', 'odometry_node', 'command_manager', 'web_bridge'):
        actions.append(Node(package='rc_car', executable=exe, name=f'rc_{exe}', parameters=params,
                            output='screen', respawn=True, respawn_delay=2.0))

    if use_lidar:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')),
            launch_arguments={'serial_port': lc['serial_port'], 'frame_id': lc['frame_id']}.items()))

    if use_slam:
        with open(cfg['slam']['config_file'], 'r', encoding='utf-8') as f:
            sy = yaml.safe_load(f) or {}
        over = {'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
                'scan_topic': '/scan_filtered', 'use_sim_time': False,
                'mode': 'localization' if slam_mode == 'localization' else 'mapping'}
        if map_file:
            over.update({'map_file_name': map_file, 'map_start_at_dock': True})
        tops = [k for k, v in sy.items() if isinstance(v, dict) and 'ros__parameters' in v]
        if not tops:
            sy['slam_toolbox'] = {'ros__parameters': {}}
            tops = ['slam_toolbox']
        for k in tops:
            sy[k]['ros__parameters'].update(over)
        merged = '/tmp/rc_slam_merged.yaml'
        with open(merged, 'w', encoding='utf-8') as f:
            yaml.safe_dump(sy, f)
        launch_name = 'localization_launch.py' if slam_mode == 'localization' else 'online_async_launch.py'
        actions.append(LogInfo(msg=f'SLAM Toolbox {slam_mode} (params {merged}, map={map_file or "새 지도"})'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', launch_name)),
            launch_arguments={'slam_params_file': merged, 'use_sim_time': 'false'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='/rc_lidar_docker/config/vehicle.yaml'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('slam_mode', default_value='mapping', description='mapping | localization'),
        DeclareLaunchArgument('map', default_value='', description='저장된 posegraph 경로(확장자 제외)'),
        OpaqueFunction(function=_setup),
    ])
