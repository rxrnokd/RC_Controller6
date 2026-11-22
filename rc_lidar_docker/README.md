# RC카 ROS 2 제어 · SLAM · 웹 UI (Raspberry Pi 4 + Docker ROS 2 Jazzy)

> 명령어 표기: **[Windows]** 개발 PC PowerShell · **[Pi]** Raspberry Pi OS 터미널(SSH) · **[Docker]** 컨테이너 내부 (`./rc.sh shell`)

---

## 0. 검증 상태 — 꼭 먼저 읽기

이 코드는 **Windows PC에서 작성**되었고 실제 Raspberry Pi·센서에서는 **아직 한 번도 실행되지 않았습니다.**

| 구분 | 상태 |
|---|---|
| Python/JS/셸 문법, YAML | ✅ Windows에서 확인 |
| WebSocket 프레이밍(서버 ↔ 노트북 클라이언트), 지도 압축/PGM 저장 | ✅ Windows 단위 테스트 |
| PI 제어·Anti-windup·출력 제한·타임아웃·E-STOP·엔코더 고장/극성 오류 래치·제어권 중재 로직 | ✅ ROS 스텁으로 로직만 확인 (실제 모터 아님) |
| pigpiod 명령 코드/워치독 스크립트 문법, 엔코더 엣지 처리량, PWM 출력 | ⛔ **하드웨어 미검증** |
| 서보 조향, MT6701 각도, US-100 거리 | ⛔ **하드웨어 미검증** |
| sllidar C1 launch 인자, SLAM Toolbox(Jazzy lifecycle) 연동, TF, 지도 저장/재사용 | ⛔ **미검증** |
| 기존 Docker 이미지와 docker-compose 조합 | ⛔ **미검증** (기존 Dockerfile 내용을 보지 못함) |
| 웹 UI 실제 브라우저 동작 | ⛔ **미검증** |
| 자동주차 알고리즘 (Hybrid A* / Planning MPC / Tracking MPC) | ✅ 수학 단위 테스트 + 노트북↔브리지 메시지 흐름 확인 (테스트용 가정 치수). ⛔ **실제 차량 구동 미검증·차단 상태** (§12) |
| **DC 모터 엔코더** | ❌ **현재 실차에서 데이터가 들어오지 않음** → PI 제어·odom·SLAM 위치·자동주차 구동 불가 |

**처음 실행할 때는 반드시 바퀴를 바닥에서 띄우고** §6 순서대로 하나씩 확인하세요.

---

## 1. 시스템 구조

```
 노트북 브라우저 (WASD 조종/모니터링) ─┐                    ┌─ 노트북 Python 자동주차 (§12)
                                      │  ws://Pi-IP:8080/ws │   python -m autopark run <Pi-IP>
                                      ▼  (단일 WebSocket,   ▼    같은 JSON 메시지 규격)
┌────────────────────── Docker (ROS 2 Jazzy, network_mode: host) ──────────────────────┐
│  web_bridge ──/rc/command_request──► command_manager ──/rc/drive_cmd(20Hz)──► vehicle_node │
│     ▲   (HTTP+WS, IP 제한)            (제어권·모드·타임아웃·E-STOP)          (공통 차량 제어부) │
│     │                                                                         │ PI, PWM,     │
│     └── /rc/vehicle_status /odom /map /scan TF ◄── odometry_node ◄─/joint_states┤ 서보, MT6701,│
│                                                  slam_toolbox ◄── /scan ◄ sllidar│ 초음파      │
└──────────────────────────────────────────────────────────────────────────────────┼──────────┘
                                                                  127.0.0.1:8888   │ 소켓
                                     Raspberry Pi OS:  pigpiod (DMA PWM · 5µs 엣지 샘플링) → GPIO
```

### Docker 안에서 GPIO/PWM/엔코더를 다룰 수 있나? (검토 결과)

| 방식 | 판단 |
|---|---|
| 컨테이너에서 `/dev/gpiochip0` + libgpiod/lgpio | GPIO 입출력·엣지 이벤트는 가능. 그러나 모터 PWM 핀 **GPIO16/20/21/26은 하드웨어 PWM 핀이 아님**(Pi 4 HW PWM은 12/13/18/19뿐) → Python 소프트웨어 PWM은 지터가 크고 CPU 사용 큼 |
| 컨테이너 안에서 pigpiod 실행 | `--privileged` + `/dev/mem` 필요, 컨테이너 재시작 시 DMA 정리 문제 |
| **호스트에서 pigpiod, 컨테이너는 소켓으로 접속 (채택)** | DMA 기반 PWM(모든 핀), 서보 펄스, 5µs 샘플링 엣지 타임스탬프. 컨테이너는 privileged 불필요. pigpiod 는 `-l`(localhost 전용) |

즉 **"하드웨어 타이밍이 중요한 부분만 Raspberry Pi OS(pigpiod)에서 실행"** 하고, 제어 로직·ROS·SLAM·웹은 모두 Docker 안에 있습니다. 추가 Python 패키지는 없습니다(pigpiod 프로토콜·WebSocket을 표준 라이브러리로 구현).

**노드 비정상 종료 대비**: pigpiod 는 클라이언트가 죽어도 마지막 PWM 을 계속 출력합니다. 그래서 pigpiod 내부에 **워치독 스크립트**를 올려 300 ms 동안 갱신이 없으면 모터 핀을 LOW(MDD3A 브레이크)로 만듭니다. 정상 종료 시에는 노드가 직접 PWM 0 을 씁니다.

---

## 2. 파일 구성 (기존 `~/rc_lidar_docker` 에 추가되는 파일)

```
rc_lidar_docker/
├── Dockerfile, config/slam.yaml, maps/, ros2_ws/src/sllidar_ros2   ← 기존 그대로 (수정 안 함)
├── docker-compose.yml          컨테이너 실행 정의 (기존 이미지 사용)
├── .env.example                RC_IMAGE 등 (cp .env.example .env)
├── rc.sh                       [Pi] up / down / logs / rebuild / shell / check
├── host/setup_host.sh          [Pi] 최초 1회: pigpiod 설치·서비스 등록
├── scripts/run_in_container.sh [Docker] 자동 빌드 + launch (컨테이너 시작 시 자동)
├── config/vehicle.yaml         ★ 모든 하드웨어 설정값 (핀, 감속비, 제한값, 보정값)
├── autopark/                   [노트북] 트레일러 후진 자동주차 알고리즘 (Pi 에서는 실행 안 함, §12)
│   ├── config.yaml             차량·트레일러 치수, 계획/MPC/속도/피드백 설정
│   ├── model.py                운동학, 충돌영역, 점유격자
│   ├── hybrid_astar.py         ① Trailer-aware Hybrid A*
│   ├── planning_mpc.py         ② Planning MPC (+ 속도 프로파일)
│   ├── tracking_mpc.py         ③ Tracking MPC
│   ├── __main__.py             실행(run/plan), WebSocket 연결, 피드백 판정, 자동주차 상태기계
│   ├── test_autopark.py        단위 테스트
│   └── requirements.txt        numpy, scipy, PyYAML
└── ros2_ws/src/rc_car/
    ├── launch/bringup.launch.py
    └── rc_car/ common.py · vehicle_node.py · odometry_node.py · command_manager.py · web_bridge.py · web/index.html
```

---

## 3. 하드웨어 · 데이터시트 확인 결과

### 첨부 자료에서 확인한 값
| 항목 | 값 | 출처 |
|---|---|---|
| DC 모터 | JGB37-520R30-12, 정격 12 V, 정격 280 rpm, 무부하 320 rpm, 정격 0.36 A, **구속 3.2 A** | 모터 사양표 |
| 모터 엔코더 | AB 2상 홀, **11 PPR(모터축)**, 엔코더 전원 3.3–5 V, PH2.0-6PIN | 사양표 |
| 감속비 | **1:30** → 바퀴 1회전 = 11×4×30 = **1320 count** | 사양표 (실측 확인 권장) |
| 모터 드라이버 | Cytron MDD3A: 4–16 V, 3 A 연속 / 5 A 피크(<5 s), 로직 High 1.7–12 V(3.3 V OK), PWM ≤ 20 kHz | MDD3A 데이터시트 |
| MDD3A 진리표 | A=H,B=L 정방향 · A=L,B=H 역방향 · L,L / H,H 브레이크 | 데이터시트 Table 3 |
| 서보 | LD-1501MG, 6–8.4 V, 구속 2.4–3 A, 펄스 **500–2500 µs ↔ 0–180°**, 위치 피드백 없음 | 서보 사양표 |
| 라이다 | RPLIDAR C1: 5 V 230–260 mA, 460800 bps, 10 Hz, 0.05–12 m | C1 데이터시트 |
| 섀시 | 전장 270 mm, 외폭 197 mm, 바퀴폭 30 mm, 바퀴 Ø75 mm. **휠베이스 표기 없음** | 섀시 도면 |

### 확인되지 않아 임의로 정하지 않은 값 (설정 필요)
| 항목 | 처리 |
|---|---|
| **배터리 전압** | `power.battery_voltage_max: null` → **설정 전에는 모터 출력 비활성화**. PWM 하드 상한 = min(1, 12 V / 배터리) 자동 계산 |
| MT6701 ABZ 분해능(PPR) | `trailer.ppr: null` → 설정 전에는 트레일러 각도 발행 안 함 (raw count 는 표시) |
| 휠베이스 | 도면 비례 추정 0.17 m, `wheelbase_measured: false` → UI에 '미보정' |
| 조향 링크 한계 / 각도 환산 | 서보 허용 500–2500 µs 안에서 **좁은 초기값 1300–1700 µs**, `calibrated: false` |
| 라이다 장착 위치 | 0 으로 두고 `measured: false` |
| US-100 사양 (범위/FOV) | 첨부 자료에 없음 → 일반 값으로 두고 '확인 필요' 주석 |

### ⚠ 배선 전 확인 (GPIO는 3.3 V 전용, 5 V 입력 시 손상)
- **모터 엔코더 전원을 3.3 V로** (사양 3.3–5 V). 5 V로 쓰면 A/B 출력이 5 V → 레벨 시프터 필요.
- **MT6701 전원 3.3 V**.
- **US-100 Trig/Echo 모드**(뒷면 점퍼 제거). 전원을 3.3 V로 하면 Echo도 3.3 V. 5 V로 쓰면 Echo에 분압 저항 필수.
- 서보는 **별도 6–8.4 V 전원** (구속 최대 3 A — Pi 5 V/MDD3A 5VO(최대 200 mA)로 공급 금지). **모든 GND 공통**.
- 모터 구속전류 3.2 A > MDD3A 연속 3 A → 바퀴가 막힌 채 출력하지 않도록 (PI 모드는 0.6 s 내 정지 감지).

### GPIO 사용 (GPIO_Pin.txt) 과 충돌 기능 → **반드시 꺼야 함**
| GPIO | 용도 | 충돌 | 조치 |
|---|---|---|---|
| 14, 15 | 모터1 엔코더B / 모터2 엔코더A | UART·시리얼 콘솔 | `raspi-config` → Interface → Serial: 로그인 셸 **No**, 하드웨어 **No** |
| 9, 10 | MT6701 Z / B | SPI0 | `dtparam=spi=off` (또는 해당 줄 삭제) |
| 4 | 모터1 엔코더A | 1-Wire | `dtoverlay=w1-gpio` 없어야 함 |
| 13 | 서보 | — | pigpio DMA 서보 펄스 |
| 16,20 / 21,26 | 모터1 / 모터2 PWM A,B | — | pigpio DMA PWM 1 kHz |
| 17/27, 22/23, 24/25, 5/6 | 초음파 1–4 trig/echo | — | — |
| 19 | MT6701 A | — | (PWM1 채널을 쓰는 오디오/오버레이가 없어야 함) |

`./rc.sh check` 가 위 항목을 자동 점검합니다.

---

## 4. 설치 (최초 1회)

### 4-1. [Windows] 파일을 Pi로 복사
```powershell
cd "$HOME\OneDrive\바탕 화면\RC_Controller6"
scp -r .\rc_lidar_docker\* pi@<Pi-IP>:~/rc_lidar_docker/
scp .\rc_lidar_docker\.env.example pi@<Pi-IP>:~/rc_lidar_docker/
```
기존 `Dockerfile`, `config/slam.yaml`, `maps/`, `sllidar_ros2` 는 덮어쓰지 않습니다(같은 이름 파일이 없음).

### 4-2. [Pi] 줄바꿈·권한 정리, pigpiod 설치
```bash
cd ~/rc_lidar_docker
sed -i 's/\r$//' rc.sh host/setup_host.sh scripts/run_in_container.sh   # Windows CRLF 대비
chmod +x rc.sh host/setup_host.sh scripts/run_in_container.sh
sudo bash host/setup_host.sh        # apt 에 pigpio 가 없으면 소스 빌드 방법을 출력함
```

### 4-3. [Pi] 부팅 설정 (GPIO 충돌 제거) 후 재부팅
```bash
sudo raspi-config        # Interface Options → Serial Port: 로그인 셸 No / 하드웨어 No,  SPI: No,  1-Wire: No
sudo reboot
./rc.sh check            # 모든 항목 OK 확인
```

### 4-4. [Pi] 기존 Docker 이미지 지정
```bash
docker images                       # 기존 ROS 2 Jazzy + SLAM Toolbox 이미지 이름 확인
cp .env.example .env
nano .env                           # RC_IMAGE=<기존 이미지 이름:태그>
```
`RC_IMAGE` 이미지가 없으면 `docker compose` 가 기존 `Dockerfile` 로 빌드합니다(새 패키지 설치 없음).

### 4-5. [Pi] 배터리 전압 설정 (필수)
```bash
nano config/vehicle.yaml            # power.battery_voltage_max: 12.6  (예: 3S LiPo 만충)
```

### 4-6. [Pi] 실행
```bash
./rc.sh up          # 최초에는 컨테이너 안에서 colcon build (sllidar_ros2 + rc_car, 수 분 소요)
./rc.sh logs        # "웹 UI: http://..." 로그 확인
```
노트북 브라우저에서 **`http://<Pi-IP>:8080`** 접속.

---

## 5. 전원 재공급 후

자동입니다: `rc-pigpiod`(systemd) → Docker(`restart: unless-stopped`) → 빌드 생략 → launch.
**전원 켜고 1분 정도 기다린 뒤 브라우저로 `http://<Pi-IP>:8080` 접속만 하면 됩니다.** (vehicle_node 는 pigpiod 가 늦게 떠도 2초마다 재접속)

| 작업 | 명령 [Pi] |
|---|---|
| 로그 | `./rc.sh logs` |
| 정지 / 시작 | `./rc.sh down` / `./rc.sh up` |
| 파이썬 파일 수정 후 | `./rc.sh restart` (symlink 설치라 재빌드 불필요) |
| 새 파일·launch·setup.py 변경 후 | `./rc.sh rebuild` |
| 저장 지도로 시작 | `.env` 에 `RC_LAUNCH_ARGS=map:=/rc_lidar_docker/maps/room1` 후 `./rc.sh up` |

---

## 6. 단계별 하드웨어 검증 (바퀴를 띄운 상태에서)

각 단계 통과 시 체크하세요. **체크 전에는 해당 기능을 '검증 완료'로 간주하지 마세요.**

1. **pigpiod**: [Pi] `pigs t` 가 숫자 출력. 로그에 `pigpiod vXX 연결`.
2. **하드웨어만 실행**: `.env` 에 `RC_LAUNCH_ARGS=slam:=false lidar:=false` → `./rc.sh up`. 웹 UI 접속, 상단 '엔코더' 표시 확인.
3. **엔코더 분해능**: 바퀴를 손으로 정확히 1바퀴 전진 방향으로 돌림 → UI '누적 회전' ≈ **+1.00** (count ≈ 1320).
   - 음수면 해당 모터 `invert_encoder: true`. 1.00 이 아니면 감속비/PPR 확인 후 `gear_ratio` 수정.
4. **Open-loop 방향**: UI 'Open-loop PWM' 선택, 최대 PWM 15 %, `W` 짧게 → 두 바퀴 모두 **전진 방향** 회전, 측정 RPM **양수**.
   - 바퀴가 반대로 돌면 `invert_output: true`. (보통 좌/우 중 한쪽만 바꿔야 함)
   - **3·4단계를 모두 통과하면** `vehicle.yaml` 의 `motors:` 아래에 `encoders_verified: true` 를 넣고 `./rc.sh restart`.
     이 값이 false 이거나 줄이 없으면 PI 속도제어, 속도와 `/odom` 발행(→ SLAM 위치), 자동주차가 모두 차단됩니다.
     (신호가 없는 엔코더가 카운트 0 을 내면 '정지 중'과 구분할 수 없기 때문입니다.)
5. **워치독**: Open-loop 20 %로 `W` 누른 채 [Pi] `docker kill -s KILL rc_car` → **0.3 s 내 바퀴 정지** 확인. (실패하면 운행 금지, §9 참고) 이후 `./rc.sh up`.
6. **PI 제어** (`encoders_verified: true` 필요): 'PI 속도제어' 선택, 최대 RPM 60 → `W` → 측정 RPM 이 목표를 따라가는지. 진동하면 `kp`/`ki` 감소, 느리면 증가.
7. **고장 감지**: PI 모드 주행 중 한쪽 엔코더 커넥터 분리 → `ENCODER_OR_STALL_*` 표시 + 정지 확인. 'E-STOP 해제/고장 리셋' 으로 복구.
8. **서보**: `A`/`D` 로 조향. 링크가 걸리지 않는 범위로 `pulse_min_us/max_us`, 직진이 되도록 `center_us` 조정. 바퀴 각도를 각도기로 재서 `us_per_rad` = (펄스 변화 µs) / (바퀴 각도 rad). 끝나면 `calibrated: true`.
9. **초음파**: 벽에서 30 cm, 100 cm 에 두고 값 비교. `TIMEOUT` 이면 배선/점퍼/전원 확인.
10. **MT6701**: 연결축을 천천히 360° 이상 돌려 'Z 간격 count' 확인 → `trailer.ppr = Z간격 / 4`. 트레일러를 일직선으로 놓고 '영점 설정'. 왼쪽으로 꺾었을 때 각도가 **+** 가 아니면 `invert: true`.
11. **치수**: 휠베이스·트랙 실측 → `wheelbase_m`, `track_width_m`, `*_measured: true`.
12. **라이다 TF**: 뒷차축 중심 기준 라이다 위치 실측 → `lidar.x/y/z/yaw`, `measured: true`. 차량 정면 물체가 UI 지도에서 차량 앞쪽에 보이는지 확인.
13. **Odometry**: 바닥에서 1 m 직진 → UI 위치/누적거리 ≈ 1 m. 제자리 원 주행 후 yaw 오차 확인 (`yaw_source` steering / wheel_diff 비교).
14. **SLAM**: `RC_LAUNCH_ARGS=` (기본) → 상단 칩 5개(라이다·엔코더·Odom·SLAM 위치·지도) 모두 초록일 때만 'SLAM 동작 중' 표시. 천천히 주행하며 지도 생성 → '지도 저장'.

---

## 7. 웹 UI 사용법

- **지도(가장 큰 영역)**: SLAM 지도 + 차량(파란 사각형, 노란 선=전방, 앞바퀴=명령 조향각) + 라이다 스캔(빨간 점) + 트레일러(보라, 각도 표시용). 휠=확대/축소, 드래그=이동, '차량 추적'.
- **조종**: `W/↑` 전진 · `S/↓` 후진 · `A/←` `D/→` 조향 · `Space` 정지 · `Esc` 비상정지. 한글 자판(ㅈㄴㅁㅇ)도 인식. 창이 포커스를 잃으면 키 입력 해제.
- **제어권**: 한 번에 한 브라우저만 조종. 다른 탭은 '보기 전용'. '제어권 가져오기'는 현재 제어자가 2초 이상 명령이 없을 때만 가능. **비상정지는 누구나** 가능.
- **속도 명령 %**: PI 모드에서는 (최대 RPM 환산 속도)의 %, Open-loop 모드에서는 (최대 PWM)의 %.
- **슬라이더**: 최대 RPM(≤ 정격 280), 최대 PWM(≤ 하드 상한), 최대 조향(≤ 보정된 한계). 서버가 상한으로 잘라 적용하고 실제 값을 다시 표시. **재부팅 시 config 값으로 복귀** (영구 변경은 `vehicle.yaml`).
- **PI ↔ Open-loop 전환**: MANUAL·정지 상태에서만. PI 모드에서 엔코더 고장 시 **자동으로 Open-loop로 바뀌지 않고 정지**합니다.
- **통신 끊김**: UI 에 빨간 배너. 차량은 0.5 s 명령 타임아웃(Command Manager) + 0.3 s(차량 제어부) 로 자동 정지. 브라우저 탭 종료 시 즉시 정지.
- **지도 저장**: `maps/<이름>.pgm/.yaml` (map_server 형식) + `<이름>.posegraph/.data` (SLAM Toolbox 재사용용).

---

## 8. 설정 (`config/vehicle.yaml`)

모든 하드웨어 값은 이 파일 하나에 있습니다. 주요 항목:

| 키 | 설명 |
|---|---|
| `power.battery_voltage_max` | **필수**. PWM 하드 상한 계산 |
| `motors.left/right` | 핀, `invert_output`, `invert_encoder` |
| `motors.encoder_ppr / gear_ratio` | 11 / 30 (데이터시트) |
| `motors.encoders_verified` | 엔코더 실측 확인 후 true (§6-4). false 면 PI·odom·자동주차 차단, Open-loop 만 가능 |
| `speed_control.kp/ki/kff` | PI 이득. 조건부 적분 Anti-windup |
| `speed_control.rpm_accel_limit / pwm_slew_per_s` | 급격한 출력 변화 제한 |
| `speed_control.stall_* / reverse_*` | 엔코더 고장·구속 / 방향 불일치(폭주 방지) 감지 |
| `steering.*` | 펄스 범위·중심·환산계수 |
| `trailer.ppr / invert` | MT6701 |
| `ultrasonic.*` | 측정 주기·타임아웃·센서 목록 |
| `lidar.*` | 포트, base_link→laser TF |
| `odometry.yaw_source` | `steering` (v·tanδ/L) 또는 `wheel_diff` ((vR−vL)/트랙) |
| `safety.*` | 명령 타임아웃, 모드 전환 정지 판정 |
| `autopark.enabled` | 알고리즘 준비 전까지 **false** |
| `web.port / allowed_networks` | 접속 허용 사설망 대역 |

---

## 9. ROS 2 인터페이스

| 토픽 | 타입 | 발행 | 내용 |
|---|---|---|---|
| `/scan` | LaserScan | sllidar | frame `laser` |
| `/odom` | Odometry | odometry_node | odom→base_link, 엔코더 정상일 때만 |
| `/tf` | | odometry_node, slam_toolbox | `map → odom → base_link → laser`, `base_link → ultrasonic_*` (static) |
| `/map` | OccupancyGrid | slam_toolbox | |
| `/joint_states` | JointState | vehicle_node | `rear_left_wheel`, `rear_right_wheel` (위치 rad, 속도 rad/s, effort=PWM duty), `trailer_joint` |
| `/trailer/angle` | Float64 | vehicle_node | MT6701 트레일러 상대각 rad (+ = 반시계) |
| `/steering/angle_cmd` | Float64 | vehicle_node | **명령** 조향각 rad (측정값 아님) |
| `/ultrasonic/us1..us4` | Range | vehicle_node | m, 실패 NaN / 범위초과 +inf / 근접 −inf |
| `/rc/vehicle_status` | String(JSON) | vehicle_node | 모터 목표/측정 RPM·PWM·누적회전·거리, 조향, 트레일러, 초음파, 고장 |
| `/rc/control_state` | String(JSON) | command_manager | 모드, 제어권, E-STOP, 자동주차 차단 사유 |
| `/rc/drive_cmd` | String(JSON) | command_manager | 최종 명령 (vehicle_node 만 구독) |

base_link 원점 = **뒷차축 중심(지면)**, +x 전방, +y 좌측, 조향각 + = 좌회전.

---

## 10. WebSocket 메시지 규격 (브라우저·노트북 공통)

`ws://<Pi-IP>:8080/ws`, 모든 메시지는 JSON 텍스트. 연결 후 **0.5 s 간격 `ping` 필수** (3 s 무수신 시 서버가 연결 종료 → 차량 정지).

**클라이언트 → 서버**
| type | 필드 | 설명 |
|---|---|---|
| `hello` | `role`: `ui`\|`autopark`, `name`, `ready` | 최초 1회 |
| `ping` | `t`, `ready`(autopark) | 하트비트 |
| `drive` | `v` [m/s], `steer` [rad] | PI 모드 / AUTO_PARK (공통 차량 제어 인터페이스) |
| `drive` | `throttle` [−1..1], `steer` | Open-loop (MANUAL UI 전용) |
| `stop` / `estop` / `estop_reset` | | 정지 / 비상정지(래치) / 해제·고장 리셋(UI만) |
| `mode` | `mode`: `MANUAL`\|`AUTO_PARK` | 전환 요청 (정지 확인 후 전환) |
| `claim` / `release` | `force` | 수동 제어권 |
| `set_control_mode` | `control_mode`: `PI`\|`OPEN_LOOP` | |
| `set_limits` | `max_rpm`, `max_pwm`(0–1), `max_steer_rad` | |
| `trailer_zero` / `save_map` | `name` | |

**서버 → 클라이언트**: `welcome`(id, config 요약) · `state`(10 Hz: `vehicle`(`stamp` 포함), `control`, `pose`{frame,x,y,yaw,`age_s`}, `odom`{x,y,yaw,v,w}, `slam`, `park`(자동주차 상태 중계), `you`) · `map`(int8 → zlib → base64, 갱신 시) · `scan`(5 Hz, 지도 좌표 점) · `park_paths` · `reply` · `pong`.

**자동주차 중계 메시지** (브리지는 전달만 함, 알고리즘은 노트북):
| type | 방향 | 내용 |
|---|---|---|
| `park_cmd` | UI → 브리지 → 노트북 | `action`: `plan`(+`goal`{x,y,yaw}=트레일러 차축 목표, `start`=검증용 시작자세 또는 null) / `start` / `stop` / `clear` |
| `park_status` | 노트북 → 브리지 → UI(`state.park`) | 단계, 메시지, 피드백 판정, 구동 차단 사유, 계획 요약, 추종 오차·명령·예측 |
| `park_paths` | 노트북 → 브리지 → UI | Hybrid A* 경로, Planning MPC 경로(구간·방향), 트레일러 궤적, 시작/목표 외곽선 |

---

## 11. 문제 해결

| 증상 | 확인 |
|---|---|
| UI 'pigpiod 연결 안 됨' | [Pi] `systemctl status rc-pigpiod`, `pigs t`. pigpiod 는 Pi 4 전용(Pi 5 미지원) |
| `WATCHDOG_SCRIPT_FAILED` | 로그의 pigpio 오류코드 확인. 임시로 `pigpio.watchdog_script: false` 가능하나 **§6-5 워치독 대체 수단 없이 운행 금지** |
| `MOTORS_DISABLED` | `battery_voltage_max` 미설정/범위 밖 |
| 엔코더 count 안 변함 | 엔코더 전원(3.3 V), GND 공통, `./rc.sh check` 의 UART/1-Wire 충돌 |
| `enc_errors` 증가 | 엣지 누락(노이즈/배선). 엔코더 선 짧게, 풀업 확인 |
| `DIRECTION_MISMATCH_*` | `invert_output`/`invert_encoder` 설정 (§6-3, 6-4) |
| 초음파 `TIMEOUT` | US-100 점퍼 제거(Trig/Echo 모드), Echo 전압, 핀 |
| MT6701 값 이상 | SPI 꺼짐 확인(GPIO9/10), 전원 3.3 V, ABZ 출력 모드 |
| `/scan` 없음 | [Pi] `ls /dev/ttyUSB*`, `vehicle.yaml` `lidar.serial_port`. sllidar 런치 인자명이 다르면 `bringup.launch.py` 의 `serial_port/frame_id` 수정 |
| SLAM 칩 'SLAM 위치' 빨강 | `/odom` 정상인지, `ros2 run tf2_ros tf2_echo map base_link` [Docker] |
| slam_toolbox 실행 안 됨 | [Docker] `ls /opt/ros/jazzy/share/slam_toolbox/launch` 에 `online_async_launch.py` 존재 확인. 병합 파라미터: `/tmp/rc_slam_merged.yaml` |
| 웹 접속 안 됨 | 같은 Wi-Fi인지, `allowed_networks` 에 노트북 IP 대역 포함, Pi 방화벽 |
| 지도 표시 안 됨 | 오래된 브라우저(DecompressionStream 필요: Chrome 80+, Safari 16.4+, Firefox 113+) |
| 빌드 오류 | `./rc.sh rebuild`, [Docker] `cd /rc_lidar_docker/ros2_ws && colcon build --symlink-install --build-base build_rc --install-base install_rc` |

### 유용한 명령 [Docker] (`./rc.sh shell`)
```bash
ros2 topic hz /scan
ros2 topic echo /rc/vehicle_status --once
ros2 topic echo /trailer/angle
ros2 run tf2_ros tf2_echo odom base_link
ros2 node list
```

---

## 12. 트레일러 후진 자동주차 (노트북)

### 12-1. 현재 상태 — 먼저 읽기
- **DC 모터 엔코더 데이터가 들어오지 않습니다.** 그 결과로 아래 기능을 쓸 수 없습니다.
  - `motors.encoders_verified` 가 true 가 아니면 엔코더 odom(`/odom`, `odom→base_link` TF)과 차량 속도를 발행하지 않습니다.
  - SLAM Toolbox 는 odom TF 가 있어야 스캔을 처리합니다. 그래서 **SLAM 지도와 위치(map→base_link)도 생기지 않습니다.**
  - 실제 차량 속도 피드백이 없습니다.
- 위치나 속도를 다른 값으로 **추정하거나 만들어내지 않습니다.** 그래서 **실제 AUTO_PARK 구동은 지금 차단됩니다.**
- 지금 할 수 있는 일은 **경로 계획과 알고리즘 검증**입니다. 저장된 지도나 웹 UI 에서 지정한 검증용 시작자세를 쓰며, 이렇게 만든 경로는 실제로 구동할 수 없습니다.

| 피드백 | 출처 | 지금 | 자동주차에 필요 |
|---|---|---|---|
| 위치 x, y, 방향 yaw | SLAM `map→base_link` (엔코더 odom 이 있어야 동작) | ❌ | 필수 |
| 차량 속도 | DC 모터 엔코더 | ❌ (고장) | 필수 (전환점 정지 판정, 속도 확인) |
| 트레일러 연결각 β | MT6701 (엔코더와 무관) | `trailer.ppr` 설정 후 가능 | 필수 |
| 조향각 | 서보 **명령값** (측정 피드백 없음) | 미보정 | 보정 필수 |
| 초음파 | US-100 | 가능 | 근접 시 정지에 사용 (필수는 아님) |

> 참고: 라이다만으로 odom 을 만드는 패키지(예: laser scan matcher 계열)를 추가하면 SLAM 위치는 엔코더 없이도 얻을 수 있습니다. 하지만 속도 피드백은 여전히 없습니다. 이 프로젝트는 SLAM Toolbox 를 수정하거나 재구현하지 않으므로, 이 방법은 넣지 않았습니다.

### 12-2. 알고리즘 구조
```
웹 UI: 목표(트레일러 차축 위치·방향) 지정 → park_cmd → [브리지 중계] → 노트북 python -m autopark run
  ① Hybrid A*     (x, y, θ, β) 4차원 탐색. 견인차·트레일러 운동학을 그대로 적분하는 모션 프리미티브(전/후진 × 조향 5단계)
                  · 매 substep 마다 차체·트레일러 충돌(원 근사, 안전거리)과 |β| ≤ hitch_angle_max(Jackknife 방지)를 검사
                  · 비용: 이동거리, 전/후진 전환, 조향 크기·변화, |β|.  마지막 구간은 후진(final_direction: reverse)
                  · 휴리스틱: 장애물을 반영한 2D Dijkstra(트레일러 차축 → 목표) + 트레일러 방향 오차 (가중 A*)
                  · 실패 사유 코드: START_IN_COLLISION, GOAL_IN_COLLISION, START_HITCH_LIMIT, NO_PATH, EXPANSION_LIMIT, TIMEOUT
  ② Planning MPC  구간(전진/후진)마다 조향 시퀀스와 구간 길이를 SLSQP 로 최적화
                  · 비용: A* 경로 근접 + 조향/조향변화 + 장애물 근접(연속 거리장) + 연결각 소프트 한계
                  · 제약: |δ| ≤ δmax, 조향 변화율, 구간 끝 자세 = 전환점/목표 (등식)
                  · 후진 구간은 개루프 불안정이라 '끝점 고정 + 시간 역방향(=전진) 적분'으로 최적화 → 최종 주차 자세가 목표와 정확히 일치
                  · 검사(충돌·연결각·연결부 불일치)를 통과하지 못하면 A* 경로를 기준으로 사용 (mpc_ok=false 로 표시)
                  · 속도 프로파일: 전/후진 최대속도, 가속도, 조향 변화율에 따른 감속, 전환점 정지·대기
  ③ Tracking MPC  20 Hz. 기준 궤적 주변의 선형 시변 모델 + 경계 제약 최소자승(lsq_linear)
                  · 상태오차 [x, y, θ, β] 를 줄이도록 목표 조향각(rad) 계산, 목표 속도(m/s) 는 속도 프로파일 + 오차에 따른 감속
                  · 연결각 한계 / 초음파 근접 → 정지,  경로 이탈 → 재계획(최대 max_replans 회) → 넘으면 정지
                  · 전환점: 실제 정지(엔코더 속도) 확인 후 다음 구간,  마지막 구간: 자세 오차가 허용치 이내면 PARKED
  출력: drive {v, steer} → Pi Command Manager → 공통 차량 제어부  (알고리즘이 PWM 을 직접 다루지 않음)
```
목표 자세는 **트레일러 차축 중심의 위치와 방향**이며, 연결각 0(일직선)을 목표로 합니다. 웹 UI 에서는 지도 위를 누른 위치가 트레일러 차축이고, **트레일러가 향할 방향(견인차가 있을 쪽)으로 드래그**합니다.

### 12-3. 안전 장치 (AUTO_PARK 구동 조건)
다음 **모든 조건**을 만족할 때만 노트북이 `ready=true` 와 `mode: AUTO_PARK` 를 보냅니다.
1. Pi `config/vehicle.yaml` 의 `autopark.enabled: true`. **하드웨어 검증 전까지 false 를 유지하세요.**
2. SLAM 위치가 map 프레임이고, 나이가 0.5 s 이하
3. 엔코더 속도가 유효함 (`encoders_ok`)
4. MT6701 연결각이 유효함
5. 조향이 보정됨 (`steering.calibrated: true`)
6. E-STOP 이 걸려 있지 않음
7. 검증용 시작자세가 아니라 현재 실제 위치에서 계획한 경로
8. 계획 이후 차량이 움직이지 않았음

Command Manager 도 별도로 검사합니다. 엔코더, odom, SLAM 위치, 트레일러각 중 하나라도 끊기면 **정지한 뒤 MANUAL 로 복귀**하며, Open-loop 로 전환하지 않습니다. 그 밖의 정지 조건은 다음과 같습니다.
- 노트북 프로그램이 끊기거나 명령이 0.5 s 이상 오지 않으면 차량이 정지합니다.
- 웹 UI 의 '자동주차 중지' 는 노트북이 끊겨 있어도 Pi 에 직접 stop 과 MANUAL 을 요청합니다.
- E-STOP 은 언제나 사용할 수 있습니다.

### 12-4. 설치 · 실행 — [노트북] Windows/macOS
```powershell
cd rc_lidar_docker                     # 프로젝트 폴더 (Pi 로 복사한 것과 같은 폴더)
python -m pip install -r autopark\requirements.txt     # numpy, scipy, PyYAML
python -m unittest autopark.test_autopark -v           # 단위 테스트 (약 5초)
python -m autopark run <Pi-IP>                         # 자동주차 클라이언트 실행 (Ctrl+C 종료)
```
Pi 쪽에서는 `web_bridge.py`, `command_manager.py`, `web/index.html` 이 바뀌었습니다. 이 파일들을 다시 복사한 뒤 [Pi] `./rc.sh restart` 를 실행하세요. 웹 UI 가 바뀌지 않으면 `./rc.sh rebuild` 를 실행합니다.

**웹 UI 사용 순서** ('자동주차' 카드):
1. 카드 제목 옆에 '노트북 연결됨' 이 보이는지 확인합니다.
2. **목표 지정**을 누르고, 지도에서 트레일러 차축 위치를 누른 채 트레일러 방향으로 드래그합니다.
3. (SLAM 위치가 없을 때만) **검증용 시작자세**를 같은 방식으로 지정합니다. 이 자세는 견인차 뒷차축 기준이고, 연결각은 MT6701 값을 쓰며 없으면 0 으로 둡니다.
4. **경로 계획**을 누르면 지도에 경로가 표시됩니다.
   - 주황: Hybrid A*
   - 파랑: Planning MPC (실선은 전진, 점선은 후진)
   - 보라: 트레일러 궤적
   - '계획' 줄에는 길이, 구간, MPC 성공 여부, 계산 시간이 나옵니다.
5. **자동주차 시작** 버튼은 §12-3 조건을 모두 만족할 때만 활성화됩니다. 차단되면 사유가 카드에 노란색으로 표시됩니다.
6. 주행 중에는 녹색 선(Tracking MPC 예측), 횡·방향·연결각 오차, 진행률이 표시됩니다.

### 12-5. 오프라인 경로 검증 (구동 없음) — [노트북]
Pi 웹 UI 에서 '지도 저장' 으로 만든 `maps/<이름>.yaml` 과 `.pgm` 을 노트북으로 복사해서 씁니다.
```powershell
python -m autopark plan --map maps\room1.yaml --start 1.2 1.6 0 0 --goal 2.5 0.25 1.5708 --svg plan.svg
python -m autopark plan --empty 5x3 --start 3.5 1.5 0 0 --goal 1.6 1.5 0      # 빈 공간
```
- `--start X Y YAW BETA`: 견인차 뒷차축 자세와 연결각
- `--goal X Y YAW`: 트레일러 차축 목표 자세
- SVG 파일은 브라우저로 열면 지도, A* 경로, MPC 경로, 트레일러 궤적, 시작/목표 외곽선을 보여 줍니다.

### 12-6. 설정 (`autopark/config.yaml`)
**확인되지 않은 치수는 `null` 로 비워 두었고, 비어 있으면 경로 계획이 거부됩니다.** 누락 항목은 웹 UI 와 터미널에 표시됩니다.

| 키 | 설명 |
|---|---|
| `vehicle.wheelbase_m` | Pi `vehicle.yaml` 과 같은 값 (현재 도면 추정 0.17, 실측 필요) |
| `vehicle.steer_max_rad` | **null**. 조향 보정 후의 실제 바퀴 최대각. Pi 가 보고하는 조향 한계가 더 작으면 그 값을 씀 |
| `vehicle.rear_overhang_m` | 뒷차축에서 차량 뒤끝까지 (추정값) |
| `trailer.hitch_offset_m` | **null**. 뒷차축에서 히치 핀까지 (뒤쪽이 +) |
| `trailer.length_m` | **null**. 히치 핀에서 트레일러 차축까지 |
| `trailer.front_m / rear_m / width_m` | **null**. 트레일러 차축 기준 차체 앞·뒤 길이, 폭 |
| `trailer.hitch_angle_max_rad` | **null**. 운용 연결각 한계 (기구적 한계보다 작게) |
| `hybrid_astar.*` | 격자 해상도, 이동 단위, 비용, 목표 허용오차, 시간·확장 한도 |
| `planning_mpc.*` | 가중치, 이산화, 구간 연결 허용오차 |
| `speed.*` | 운용 속도 정책 (전진 0.15 / 후진 0.08 m/s 등). 데이터시트 값이 아니므로 하드웨어 검증 후 조정 |
| `tracking_mpc.*` | 예측 지평, 가중치, 이탈·도착 허용오차, 재계획 횟수, 초음파 정지 거리 |
| `feedback.*` | 구동 허용 조건 (데이터 나이, 엔코더 속도 필수, 조향 보정 필수) |

치수 측정 방법: 차량과 트레일러를 일직선으로 두고 줄자로 잽니다. MT6701 영점(§6-10)을 맞춘 뒤 트레일러를 왼쪽으로 꺾었을 때 `/trailer/angle` 이 + 인지 확인하세요. 부호가 이 모델과 같아야 합니다(β = 트레일러 방향 − 차량 방향).

### 12-7. 실제 RC카 적용 순서
1. **엔코더 복구** (§6-1~4, 엔코더 진단 절차). 바퀴 1회전 ≈ +1.00 을 확인한 뒤 `encoders_verified: true` 로 바꿉니다. 웹 UI 의 '엔코더' 표시가 초록이어야 합니다.
2. PI 속도 제어 확인 (§6-6), 조향 보정 (§6-8), MT6701 `ppr` 과 영점 설정 (§6-10)
3. odom 1 m 직진 오차 확인 → SLAM 위치 안정성 확인 (상단 칩 5개 모두 초록)
4. `autopark/config.yaml` 에 트레일러 치수 입력 → `python -m unittest autopark.test_autopark`
5. 저장 지도로 오프라인 계획 검증 (§12-5)
6. 실제 위치로 '경로 계획' 만 실행해 경로가 지도와 맞는지 확인합니다. 아직 구동하지 않습니다.
7. Pi `autopark.enabled: true` → 넓은 공간, 낮은 속도(`speed.*` 를 더 낮게)에서 직선 후진 목표부터 시작합니다. 손은 E-STOP 위에 둡니다.
8. 오차 기록을 보며 `tracking_mpc` 가중치를 조정 → 전환점이 있는 경로 → 주차칸 순서로 진행

### 12-8. 검증 상태
| 항목 | 상태 |
|---|---|
| 운동학 (직진, 회전율, 가역성, 후진 불안정성), 목표 변환, 충돌 원이 사각형을 덮는지 | ✅ 단위 테스트 |
| Hybrid A*: 직선 후진, 직각 주차칸(왼쪽·오른쪽 접근), 평행 포켓, 실패 사유 코드 | ✅ 테스트 지도. 평행 포켓에서는 Planning MPC 가 거부되고 A* 경로를 사용함 (목표 오차 약 9 cm, A* 허용치 이내) |
| Planning MPC: 충돌·연결각·조향·조향변화율 제약, 최종 자세 일치, 속도 프로파일 | ✅ 테스트 지도 |
| Tracking MPC: 선형화 정확도, 기준 위 추종, 조향 변화율 제한, 연결각·초음파 정지, 이탈 재계획, 주차 완료 판정 | ✅ 단위 테스트 (폐루프 주행 시뮬레이션은 하지 않음) |
| 노트북 ↔ 브리지 메시지 흐름, 엔코더 고장 시 구동 차단 (drive/AUTO_PARK 요청 없음) | ✅ 대체 서버로 확인 |
| 실제 차량 추종 성능, 트레일러 실제 치수 반영, 웹 UI 지도 조작 | ⛔ **미검증** |

테스트에 쓴 치수(트레일러 L2=0.25 m 등)는 `test_autopark.py` 안에서만 쓰는 가정값이며, `config.yaml` 에는 넣지 않았습니다.
