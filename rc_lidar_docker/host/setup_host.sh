#!/bin/bash
# [Raspberry Pi OS] 최초 1회 실행: pigpiod 설치/등록 (localhost 전용, 부팅 시 자동 시작)
#   sudo bash host/setup_host.sh
set -e
if [ "$(id -u)" != 0 ]; then echo "sudo 로 실행하세요: sudo bash host/setup_host.sh"; exit 1; fi

PIGPIOD=$(command -v pigpiod || true)
if [ -z "$PIGPIOD" ]; then
  echo "[1/3] pigpio 설치 시도 (apt)"
  if apt-get install -y pigpio; then
    PIGPIOD=$(command -v pigpiod)
  else
    echo "apt 에 pigpio 패키지가 없습니다. 소스 빌드:"
    echo "  sudo apt install -y build-essential unzip wget"
    echo "  wget https://github.com/joan2937/pigpio/archive/master.zip && unzip master.zip"
    echo "  cd pigpio-master && make && sudo make install"
    echo "설치 후 이 스크립트를 다시 실행하세요."
    exit 1
  fi
fi
echo "pigpiod: $PIGPIOD"

echo "[2/3] systemd 서비스 등록 (rc-pigpiod: -l = localhost 전용, -s 5 = 5us 샘플링)"
systemctl disable --now pigpiod 2>/dev/null || true
cat > /etc/systemd/system/rc-pigpiod.service <<EOF
[Unit]
Description=pigpio daemon for RC car (localhost only)
After=network.target

[Service]
Type=forking
ExecStart=$PIGPIOD -l -s 5
ExecStop=/bin/kill -TERM \$MAINPID
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now rc-pigpiod
sleep 1
pigs t >/dev/null && echo "pigpiod 응답 OK" || echo "!! pigpiod 응답 없음: journalctl -u rc-pigpiod"

echo "[3/3] Docker 부팅 자동 시작"
systemctl enable docker >/dev/null 2>&1 || true
echo "완료. 부팅 설정 점검: ./rc.sh check"
