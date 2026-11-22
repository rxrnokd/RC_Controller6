#!/bin/bash
# [Raspberry Pi OS] RC카 실행 도우미
#   ./rc.sh up        컨테이너 시작 (부팅 후 자동 시작되므로 보통 불필요)
#   ./rc.sh down      정지 (모터 PWM 0 후 종료)
#   ./rc.sh logs      로그 보기
#   ./rc.sh restart   재시작
#   ./rc.sh rebuild   colcon 재빌드 후 시작 (새 파일/entry point 추가 시)
#   ./rc.sh shell     컨테이너 내부 셸 (ROS 환경 로드됨)
#   ./rc.sh check     하드웨어/부팅 설정 점검
cd "$(dirname "$0")"
IP=$(hostname -I 2>/dev/null | awk '{print $1}')

check() {
  echo "== pigpiod";  (systemctl is-active rc-pigpiod || systemctl is-active pigpiod) 2>/dev/null
  pigs t >/dev/null 2>&1 && echo "pigs 응답 OK" || echo "!! pigpiod 응답 없음 (sudo systemctl start rc-pigpiod)"
  echo "== 라이다";   ls -l /dev/ttyUSB* /dev/rplidar 2>/dev/null || echo "!! /dev/ttyUSB* 없음"
  CFG=/boot/firmware/config.txt; [ -f $CFG ] || CFG=/boot/config.txt
  CMD=/boot/firmware/cmdline.txt; [ -f $CMD ] || CMD=/boot/cmdline.txt
  echo "== 부팅 설정 ($CFG)"
  grep -Eq '^\s*dtparam=spi=on' $CFG && echo "!! SPI 켜짐 → GPIO9/10(MT6701 Z/B) 충돌. dtparam=spi=off 필요" || echo "SPI off OK"
  grep -Eq '^\s*enable_uart=1' $CFG && echo "!! UART 켜짐 → GPIO14/15(모터 엔코더) 충돌" || echo "UART off OK"
  grep -Eq '^\s*dtoverlay=w1-gpio' $CFG && echo "!! 1-Wire 켜짐 → GPIO4(모터1 엔코더A) 충돌" || echo "1-Wire off OK"
  grep -Eq 'console=(serial0|ttyAMA0|ttyS0)' $CMD && echo "!! 시리얼 콘솔 활성 ($CMD) → GPIO14/15 충돌" || echo "시리얼 콘솔 off OK"
  echo "== Docker"; docker compose ps
  echo "웹 UI: http://$IP:8080"
}

case "$1" in
  up|"")   docker compose up -d && echo "웹 UI: http://$IP:8080" ;;
  down)    docker compose down ;;
  logs)    docker compose logs -f --tail 100 ;;
  restart) docker compose restart && echo "웹 UI: http://$IP:8080" ;;
  rebuild) RC_REBUILD=1 docker compose up -d --force-recreate && docker compose logs -f --tail 100 ;;
  shell)   docker compose exec rc bash -c 'source /opt/ros/jazzy/setup.bash; source /rc_lidar_docker/ros2_ws/install_rc/setup.bash; exec bash' ;;
  check)   check ;;
  *)       sed -n '2,9p' "$0" ;;
esac
