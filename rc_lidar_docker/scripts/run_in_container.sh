#!/bin/bash
# [Docker 내부] 컨테이너 시작 시 자동 실행: 워크스페이스 빌드(최초 1회) → 전체 launch
set -e
source /opt/ros/jazzy/setup.bash
WS=/rc_lidar_docker/ros2_ws
cd "$WS"
# 기존 build/install 과 경로가 섞이지 않도록 별도 디렉터리 사용
if [ ! -f install_rc/setup.bash ] || [ ! -d install_rc/rc_car ] || [ "${RC_REBUILD:-0}" = "1" ]; then
  echo "[rc] colcon build (sllidar_ros2 + rc_car) ..."
  colcon build --symlink-install --build-base build_rc --install-base install_rc \
    --packages-select sllidar_ros2 rc_car
fi
source install_rc/setup.bash
echo "[rc] ros2 launch rc_car bringup.launch.py ${RC_LAUNCH_ARGS}"
exec ros2 launch rc_car bringup.launch.py ${RC_LAUNCH_ARGS}
