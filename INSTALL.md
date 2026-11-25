# 설치 · 실행 방법

GitHub 저장소: <https://github.com/rxrnokd/RC_Controller6>

이 문서는 **설치와 실행 방법만** 다룹니다. 하드웨어 단계별 점검, 설정값 의미, 문제 해결은 [`rc_lidar_docker/README.md`](rc_lidar_docker/README.md) 를 보세요.

각 단계 앞의 표시는 **명령을 입력하는 곳**입니다.

| 표시 | 어디서 |
|---|---|
| **[PC]** | 개발용 Windows PC — PowerShell |
| **[Pi]** | 라즈베리파이 터미널 (Windows 에서 `ssh` 로 접속한 창) |
| **[컨테이너]** | 라즈베리파이의 Docker 컨테이너 안 (`./rc.sh shell` 로 들어간 창) |
| **[브라우저]** | 노트북/PC 웹 브라우저 (Chrome 권장) |

---

## 0. 전체 흐름 — 폴더가 두 개인 이유

```
[PC] 코드 수정 → git push ──► GitHub ──► [Pi] git pull (~/RC_Controller6)
                                              │ rsync 로 복사
                                              ▼
                                         ~/rc_lidar_docker   ← 실제로 실행하는 폴더
                                         (기존 Dockerfile · config/slam.yaml · maps · sllidar_ros2
                                          + 이 저장소의 파일)
```

- **`~/RC_Controller6`** : GitHub 에서 받은 원본. **여기서는 파일을 직접 고치지 않습니다** (고치면 다음 `git pull` 이 충돌).
- **`~/rc_lidar_docker`** : 원래 쓰던 Docker·SLAM 폴더. 저장소에는 기존 `Dockerfile`, `config/slam.yaml`, `sllidar_ros2` 가 들어 있지 않으므로, 저장소 파일을 이 폴더에 **덧붙여 복사**해서 실행합니다. 기존 파일은 이름이 겹치지 않아 지워지거나 덮어써지지 않습니다.
- **`config/vehicle.yaml`(배터리 전압 등 내 차량 설정)과 `.env` 는 복사 대상에서 빼서**, 업데이트해도 Pi 에서 고친 값이 유지되게 합니다.

---

## 1. [Pi] 최초 설치 (한 번만)

### 1-1. Pi 에 접속
**[PC]** PowerShell 을 열고:
```powershell
ssh <사용자이름>@<Pi-IP>
# 예) ssh pi@192.168.0.23
```
- Pi IP 를 모르면 Pi 에 모니터를 연결해 터미널에서 `hostname -I` 를 실행하거나, 공유기 관리 페이지의 접속 기기 목록에서 확인합니다.
- 처음 접속 시 `Are you sure you want to continue connecting` 이 나오면 `yes` 입력 후 비밀번호 입력.

이후 1장의 명령은 모두 이 ssh 창(**[Pi]**)에서 입력합니다.

### 1-2. 기존 폴더 확인
**[Pi]**
```bash
ls ~/rc_lidar_docker
ls ~/rc_lidar_docker/config/slam.yaml
ls ~/rc_lidar_docker/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
docker images
```
- `Dockerfile`, `config`, `ros2_ws` 가 보이고 위 두 파일이 "No such file" 없이 출력되어야 합니다.
- `docker images` 에 기존 ROS 2 Jazzy + SLAM Toolbox 이미지 이름(REPOSITORY:TAG)을 메모해 둡니다. (1-8 에서 사용)
- **기존 폴더가 없다면 여기서 멈추세요.** 이 저장소만으로는 Docker 이미지·라이다 드라이버가 갖춰지지 않습니다.

### 1-3. 기존 폴더 백업
**[Pi]**
```bash
cp -a ~/rc_lidar_docker ~/rc_lidar_docker_backup_$(date +%Y%m%d)
```

### 1-4. git · rsync 설치 후 저장소 받기
**[Pi]**
```bash
sudo apt update
sudo apt install -y git rsync
cd ~
git clone https://github.com/rxrnokd/RC_Controller6.git
ls ~/RC_Controller6/rc_lidar_docker        # rc.sh, docker-compose.yml, config ... 가 보이면 OK
```
> 저장소가 비공개(private)라면 `git clone` 시 GitHub 사용자명과 **Personal Access Token**(비밀번호 대신)을 입력해야 합니다. GitHub → Settings → Developer settings → Personal access tokens 에서 발급.

### 1-5. 실행 폴더로 복사
**[Pi]**
```bash
rsync -av --exclude config/vehicle.yaml ~/RC_Controller6/rc_lidar_docker/ ~/rc_lidar_docker/
cp -n ~/RC_Controller6/rc_lidar_docker/config/vehicle.yaml ~/rc_lidar_docker/config/
```
- 첫 줄: 저장소 파일을 실행 폴더에 복사 (기존 파일은 그대로, `vehicle.yaml` 은 제외).
- 둘째 줄: `vehicle.yaml` 이 **아직 없을 때만** 복사 (`-n` = 이미 있으면 덮어쓰지 않음).
- 경로 끝의 `/` 를 빼먹지 마세요 (`rc_lidar_docker/` 처럼).
- 줄바꿈(LF)과 실행 권한은 저장소에 맞춰져 있어 `sed`·`chmod` 가 필요 없습니다.

이전에 다른 방법(scp 등)으로 넣은 `vehicle.yaml` 이 이미 있다면 새 버전과 차이를 확인해 두세요:
```bash
diff ~/rc_lidar_docker/config/vehicle.yaml ~/RC_Controller6/rc_lidar_docker/config/vehicle.yaml
```
새 항목(`use_encoders`, `odometry:`, `lidar.mask_*`)이 없어도 기본값으로 동작합니다.

### 1-6. pigpiod 설치 (GPIO·PWM 담당, Pi OS 에서 실행)
**[Pi]**
```bash
cd ~/rc_lidar_docker
sudo bash host/setup_host.sh
pigs t                    # 숫자가 출력되면 OK
```
- apt 에 pigpio 가 없다는 메시지가 나오면, 화면에 출력된 소스 빌드 명령을 그대로 실행한 뒤 `sudo bash host/setup_host.sh` 를 다시 실행합니다.
- 부팅 시 자동 시작되도록 등록됩니다 (`rc-pigpiod` 서비스).

### 1-7. 부팅 설정 (SPI 끄기) 후 재부팅
MT6701(트레일러 각도)이 GPIO9/10 을 쓰므로 SPI 를 꺼야 합니다.

**[Pi]**
```bash
sudo raspi-config
```
→ `3 Interface Options` → `SPI` → **No** → `Finish` → 재부팅 여부에 **Yes**.
(DC 모터 엔코더를 쓰지 않으므로 Serial·1-Wire 는 건드리지 않아도 됩니다.)

재부팅되면 ssh 연결이 끊깁니다. 1분쯤 뒤 **[PC]** 에서 다시 `ssh <사용자이름>@<Pi-IP>` 로 접속하고:

**[Pi]**
```bash
cd ~/rc_lidar_docker
./rc.sh check
```
- `pigs 응답 OK`, `SPI off OK`, `/dev/ttyUSB0`(라이다) 가 보이면 OK.
- `UART 켜짐`, `1-Wire 켜짐`, `시리얼 콘솔 활성` 경고는 **엔코더 핀 관련이라 지금 구성에서는 무시해도 됩니다.**

### 1-8. 사용할 Docker 이미지 지정
**[Pi]**
```bash
cd ~/rc_lidar_docker
cp .env.example .env
nano .env
```
`RC_IMAGE=` 뒤를 1-2 에서 메모한 이미지 이름으로 바꿉니다. 예) `RC_IMAGE=rc_lidar_docker:latest`
- nano 저장: `Ctrl+O` → `Enter`, 종료: `Ctrl+X`.
- 지정한 이미지가 없으면 기존 `Dockerfile` 로 새로 빌드합니다(시간이 오래 걸림).

### 1-9. 배터리 전압 입력 (필수 — 안 하면 모터가 돌지 않음)
**[Pi]**
```bash
nano ~/rc_lidar_docker/config/vehicle.yaml
```
`power:` 아래 `battery_voltage_max: null` 을 실제 배터리 만충 전압으로 바꿉니다.
예) 3S LiPo → `battery_voltage_max: 12.6`, 2S LiPo → `8.4`. 저장 후 종료.

라이다 포트가 `/dev/ttyUSB0` 가 아니면(1-7 의 `./rc.sh check` 결과) 같은 파일의 `lidar: serial_port:` 도 고칩니다.

### 1-10. 첫 실행
**[Pi]**
```bash
cd ~/rc_lidar_docker
./rc.sh up
./rc.sh logs
```
- 처음에는 컨테이너 안에서 빌드(colcon build)를 하느라 **수 분** 걸립니다.
- 로그에 `웹 UI: http://...:8080` 이 나오면 준비 완료.
- `./rc.sh logs` 는 로그를 계속 보여 주는 명령입니다. `Ctrl+C` 로 빠져나와도 **차량 프로그램은 계속 실행**됩니다.

### 1-11. 라이다 위치추정에 필요한 numpy · scipy 확인
**[Pi]**
```bash
cd ~/rc_lidar_docker
./rc.sh shell
```
**[컨테이너]** (프롬프트가 바뀜)
```bash
python3 -c "import numpy, scipy; print('OK', numpy.__version__, scipy.__version__)"
exit
```
- `OK ...` 가 나오면 끝.
- `ModuleNotFoundError` 가 나오면 **[Pi]** 에서 설치 후 이미지에 저장합니다 (안 하면 지도·위치가 나오지 않거나 느림):
```bash
docker exec -it rc_car bash -c "apt-get update && apt-get install -y python3-numpy python3-scipy"
docker commit rc_car <1-8 에서 RC_IMAGE 에 적은 이름>
# 예) docker commit rc_car rc_lidar_docker:latest
cd ~/rc_lidar_docker && ./rc.sh restart
```
(기존 `Dockerfile` 을 관리한다면 `apt-get install -y python3-numpy python3-scipy` 줄을 추가해 다시 빌드하는 방법도 됩니다.)

---

## 2. [브라우저] 웹 화면 접속 · 조종

1. 노트북을 **Pi 와 같은 Wi-Fi(같은 공유기)** 에 연결합니다.
2. **[브라우저]** 주소창에 `http://<Pi-IP>:8080` 입력.
3. 상단에 `연결됨 ○○ms` 가 보이면 정상.

**처음 구동할 때는 반드시 차를 들어 바퀴를 바닥에서 띄운 상태로** 확인하세요.

| 키 | 동작 |
|---|---|
| `W` / `↑` | 전진 |
| `S` / `↓` | 후진 |
| `A` `D` / `←` `→` | 조향 |
| `Space` | 정지 |
| `Esc` 또는 빨간 **비상정지** 버튼 | 비상정지 (해제는 'E-STOP 해제' 버튼) |

- 키를 처음 누르면 그 브라우저가 제어권을 가집니다. 다른 탭·기기는 보기 전용입니다.
- 엔코더를 쓰지 않으므로 **Open-loop PWM** 으로만 움직입니다. '최대 PWM' 슬라이더를 낮게(10~20 %) 두고 시작하세요.
- 바퀴가 막혀도 자동으로 멈추지 않습니다. 이상하면 즉시 키를 떼거나 `Esc`.

---

## 3. [PC] 자동주차 프로그램 (선택 — 경로 계획·검증만, 실제 구동은 차단됨)

### 3-1. 준비 (한 번만)
1. **Git for Windows** 설치: <https://git-scm.com/download/win> (옵션은 기본값).
2. **Python 3.10 이상** 설치: <https://www.python.org/downloads/> — 설치 첫 화면에서 **"Add python.exe to PATH" 체크**.
3. **[PC]** 새 PowerShell 창에서 받기:
```powershell
cd $HOME\Documents
git clone https://github.com/rxrnokd/RC_Controller6.git
cd RC_Controller6\rc_lidar_docker
python -m pip install -r autopark\requirements.txt
```
(코드를 작성한 개발 PC 라면 clone 대신 기존 `RC_Controller6\rc_lidar_docker` 폴더에서 `pip install` 만 하면 됩니다.)

### 3-2. 테스트 (선택)
**[PC]** `RC_Controller6\rc_lidar_docker` 폴더에서:
```powershell
python -m unittest autopark.test_autopark -v
python ros2_ws\src\rc_car\test\test_scan_matcher.py -v
```
마지막 줄이 `OK` 면 정상.

### 3-3. 실행
**[PC]** `RC_Controller6\rc_lidar_docker` 폴더에서 (Pi 가 실행 중이어야 함):
```powershell
python -m autopark run <Pi-IP>
```
→ **[브라우저]** 웹 화면 오른쪽 '자동주차' 카드에 '노트북 연결됨' 이 표시됩니다. 종료는 PowerShell 창에서 `Ctrl+C`.

macOS 는 같은 명령에서 `python` → `python3`, 경로 구분자 `\` → `/` 로 바꿔 실행합니다.

---

## 4. 평소 사용

| 하고 싶은 것 | 어디서 | 명령 |
|---|---|---|
| 켜기 | — | Pi 전원을 켜고 **1분** 기다린 뒤 [브라우저] `http://<Pi-IP>:8080` (pigpiod·컨테이너 자동 시작) |
| 로그 보기 | [Pi] | `cd ~/rc_lidar_docker && ./rc.sh logs` (`Ctrl+C` 로 빠져나옴) |
| 정지 | [Pi] | `cd ~/rc_lidar_docker && ./rc.sh down` |
| 다시 시작 | [Pi] | `cd ~/rc_lidar_docker && ./rc.sh up` |
| 재시작 (설정 파일 수정 후) | [Pi] | `cd ~/rc_lidar_docker && ./rc.sh restart` |
| 상태 점검 | [Pi] | `cd ~/rc_lidar_docker && ./rc.sh check` |
| 컨테이너 안으로 | [Pi] | `cd ~/rc_lidar_docker && ./rc.sh shell` (나올 때 `exit`) |
| 끄기 | [Pi] | `sudo shutdown -h now` → 초록 LED 가 꺼진 뒤 전원 분리 |

### 실행 모드 바꾸기
**[Pi]** `nano ~/rc_lidar_docker/.env` 에서 `RC_LAUNCH_ARGS=` 를 바꾼 뒤 `cd ~/rc_lidar_docker && ./rc.sh up`.

| `RC_LAUNCH_ARGS=` | 의미 |
|---|---|
| (비움) | 기본: 라이다 + 라이다 위치추정 + SLAM 새 지도 작성 |
| `slam:=false lidar:=false` | 모터·서보·센서만 (라이다 없이 하드웨어 점검) |
| `slam:=false` | 라이다·위치추정까지만 (지도 작성 안 함) |
| `map:=/rc_lidar_docker/maps/room1` | 저장한 지도 `room1` 이어서 작성 |
| `slam_mode:=localization map:=/rc_lidar_docker/maps/room1` | 저장한 지도에서 위치추정만 |

지도는 웹 화면 '지도' 카드의 **지도 저장** 으로 `~/rc_lidar_docker/maps/` 에 저장됩니다.

---

## 5. 코드 업데이트 (GitHub 경유)

### 5-1. [PC] 수정한 코드 올리기
**[PC]** 저장소 폴더(`RC_Controller6`)에서:
```powershell
cd "$HOME\OneDrive\바탕 화면\RC_Controller6"
git add -A
git commit -m "변경 내용 요약"
git push
```

### 5-2. [Pi] 받아서 적용
**[Pi]**
```bash
cd ~/RC_Controller6 && git pull
rsync -av --exclude config/vehicle.yaml ~/RC_Controller6/rc_lidar_docker/ ~/rc_lidar_docker/
cd ~/rc_lidar_docker && ./rc.sh rebuild
```
- `./rc.sh rebuild` 는 다시 빌드하고 로그를 보여 줍니다. `웹 UI: ...` 가 나오면 `Ctrl+C` 로 빠져나와도 됩니다.
- `vehicle.yaml` 에 새 항목이 생겼는지 궁금하면:
  `diff ~/rc_lidar_docker/config/vehicle.yaml ~/RC_Controller6/rc_lidar_docker/config/vehicle.yaml`
- `git pull` 이 충돌 메시지를 내면 `~/RC_Controller6` 안의 파일을 직접 고친 경우입니다. 원본으로 되돌리기: `cd ~/RC_Controller6 && git checkout -- . && git pull`
  (내 설정은 `~/rc_lidar_docker` 쪽에 있으므로 영향 없음)

### 5-3. [PC] 다른 노트북에서 최신 코드 받기
**[PC]**
```powershell
cd $HOME\Documents\RC_Controller6
git pull
```

---

## 6. 막혔을 때 빠른 확인

| 증상 | 확인 |
|---|---|
| 브라우저 접속 안 됨 | 같은 Wi-Fi 인지, [Pi] `./rc.sh logs` 에 `웹 UI:` 가 나왔는지, IP 가 맞는지 (`hostname -I`) |
| 화면에 'pigpiod 연결 안 됨' | [Pi] `pigs t`, `sudo systemctl restart rc-pigpiod` |
| 모터가 안 돎 ('모터 비활성') | 1-9 배터리 전압 입력 여부 |
| 상단 '라이다' 빨강 | [Pi] `ls /dev/ttyUSB*`, `vehicle.yaml` 의 `lidar.serial_port` |
| 'Odom(라이다)' 빨강 / 지도가 안 생김 | 1-11 numpy·scipy, 천천히 주행, 주변에 벽·물체가 보이는지 |
| `./rc.sh: Permission denied` | [Pi] `chmod +x ~/rc_lidar_docker/rc.sh ~/rc_lidar_docker/host/setup_host.sh ~/rc_lidar_docker/scripts/run_in_container.sh` |
| 빌드 오류 | [Pi] `cd ~/rc_lidar_docker && ./rc.sh rebuild` 로그의 첫 에러 줄 확인 |

자세한 문제 해결은 [`rc_lidar_docker/README.md`](rc_lidar_docker/README.md) §11 을 보세요.
