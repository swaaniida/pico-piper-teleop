# PICO–PiPER–Tracer Teleoperation

오른쪽 PICO 컨트롤러로 PiPER를, 왼쪽 컨트롤러로 Tracer Mini를 조작한다.
손목 InnoMaker RGB 영상과 로봇/XR 데이터를 같은 episode에 저장한다.

## 조작

- 오른쪽 grip: 누르는 동안 이동
- 오른쪽 trigger: 그리퍼
- 왼쪽 grip + 왼쪽 stick: Tracer 전진/후진
- 왼쪽 grip + 오른쪽 stick: Tracer 회전
- A 버튼 또는 터미널 Enter: episode 시작/ZIP init 복귀 후 종료
- B 버튼: 즉시 전체 정지 후 ZIP start 자세 복귀, 도착 확인 뒤 토크 해제 및 프로그램 종료
- X 버튼: Tracer 범퍼 E-stop 해제
- Ctrl-C: ZIP start 복귀 후 토크 해제

PiPER는 grip을 잡은 순간을 기준으로 오른손의 위치와 손목 회전을 함께 추종한다.

## 구성

```text
single_piper_teleop/
  teleop.py       제어 및 데이터 기록
  wrist_camera.py 손목 InnoMaker RGB
  validate.py     에피소드 검증
  recover.py      CAN 중단 후 복구
  preflight.py    실행 전 점검
tracer_mini/      Tracer PICO 제어와 E-stop 해제
config/           CAN 어댑터 serial 고정 규칙
assets/           PiPER URDF
src/              PiPER SDK, XR binding submodule
```

## 새 Ubuntu에서 설치

테스트 환경은 Ubuntu 22.04, Python 3.10, `piper_sdk 0.6.1`, PiPER firmware
`S-V1.5-8`, ROS2 Humble이다. 먼저 Miniconda와 ROS2 Humble을 설치하고,
XRoboToolkit PC Service를 `/opt/apps/roboticsservice`에 복원한다.

```bash
sudo apt update
sudo apt install -y git can-utils ethtool build-essential cmake python3-colcon-common-extensions

git clone --recurse-submodules https://github.com/swaaniida/pico-piper-teleop.git ~/piper_teleop
cd ~/piper_teleop
conda create -n piper-xr python=3.10 pip -y
conda activate piper-xr
./install.sh
```

submodule 없이 clone한 경우:

```bash
git submodule update --init --recursive
./install.sh
```

XRoboToolkit PC Service는 저장소에 포함되지 않으며 `./install.sh` 실행 전에 있어야 한다.

두 CAN 어댑터 이름은 USB serial로 고정한다. 규칙 설치 후 어댑터를 다시 연결하거나
Ubuntu를 재부팅한다.

```bash
sudo install -m 0644 ~/piper_teleop/config/10-piper-can.link /etc/systemd/network/
sudo install -m 0644 ~/piper_teleop/config/11-tracer-can.link /etc/systemd/network/
sudo udevadm control --reload
```

Tracer ROS2 workspace는 실제 사용한 커밋으로 맞춘다.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/tracer_ros2_ws/src

git clone https://github.com/westonrobot/ugv_sdk.git ~/tracer_ros2_ws/src/ugv_sdk
git -C ~/tracer_ros2_ws/src/ugv_sdk checkout 58436e9c1732474566e249ce7f726e12e26304d6

git clone https://github.com/agilexrobotics/tracer_ros2.git ~/tracer_ros2_ws/src/tracer_ros2
git -C ~/tracer_ros2_ws/src/tracer_ros2 checkout 832ea9a48fe725d47d766a099d358062ea2f0f31
git -C ~/tracer_ros2_ws/src/tracer_ros2 apply \
  ~/piper_teleop/tracer_mini/tracer_vehicle_state.patch

cd ~/tracer_ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## PICO 설정

```text
Tracking: Controller ON
Mode: None
Data & Control: Send ON
Switch w/ A button: OFF
Remote Vision Listen: OFF
Data Collection Record: OFF
```

PICO Send 대상은 VM IP로 설정한다.

## 실행

### 터미널 1: XR service

```bash
cd /opt/apps/roboticsservice
env \
  LD_LIBRARY_PATH=/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib:/opt/apps/roboticsservice/SDK/x64 \
  QT_PLUGIN_PATH=/opt/apps/roboticsservice/plugins \
  QT_QML_PATH=/opt/apps/roboticsservice/qml \
  ./RoboticsServiceProcess
```

### 터미널 2: CAN

```bash
sudo ip link set piper_can down 2>/dev/null || true
sudo ip link set piper_can type can bitrate 1000000
sudo ip link set piper_can up
ip -details link show piper_can
timeout 3 candump piper_can

sudo ip link set tracer_can down 2>/dev/null || true
sudo ip link set tracer_can type can bitrate 500000
sudo ip link set tracer_can up
timeout 3 candump tracer_can
```

`piper_can`이 UP, 1 Mbps이고 PiPER 프레임이 계속 들어오는지 확인한다.

```bash
cd ~/piper_teleop
python -m single_piper_teleop.preflight
```

### 터미널 3: dry-run

```bash
cd ~/piper_teleop
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.teleop --dry-run
```

컨트롤러 pose, grip, trigger, A/B 버튼, InnoMaker 기록을 확인한다. dry-run은
로봇 명령을 보내지 않는다.

### 터미널 3: Tracer ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/tracer_ros2_ws/install/setup.bash
ros2 launch tracer_base tracer_mini_base.launch.py port_name:=tracer_can
```

현재 Tracer ROS2 source에는 설치 단계의 `tracer_vehicle_state.patch`가 적용되어 있어야 한다.

### 터미널 4: PiPER 단독 live

로봇 작업 반경을 비우고 B 버튼을 준비한다.

```bash
cd ~/piper_teleop
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.teleop
```

`READY`가 나오면 오른쪽 grip을 누른 상태에서 조작한다.

### 터미널 4: PiPER + Tracer 통합 live

PiPER 제어와 IK는 메인 프로세스에서 실행하고, ROS2 `/cmd_vel` publish와
`/tracer_status` 수신은 별도 자식 프로세스에서 실행한다. 명령 갱신이 200 ms 이상
끊기면 자식 프로세스가 Tracer에 0속도를 보낸다.

```bash
source /opt/ros/humble/setup.bash
source ~/tracer_ros2_ws/install/setup.bash
cd ~/piper_teleop
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.teleop --with-tracer
```

연결되면 참조 ZIP과 같이 `start → init` 순서로 이동하고 init 자세에서 대기한다.
두 자세는 각각 `[-0.51, 0.49, 0.00, 3.96, 4.83, -8.33]°`,
`[-1.86, 23.84, -60.0, 1.60, 50.65, 0.0]°`이다.
원본 ZIP의 start J3 `+1.60°`는 이 PiPER에서 `0°` 경계에 걸리므로,
실측 도달 자세인 `0°`를 사용한다.

```text
A: episode 시작
A: init 복귀 확인 → episode 저장 종료
A: 다음 episode 시작
B: 즉시 hold → start 복귀 확인 → STANDBY → 토크 해제 → 프로그램 종료
```

## 종료

PICO의 B 버튼이나 터미널의 Ctrl-C로 전체 세션을 종료한다.

```text
명령 중단 → ZIP start 복귀(50%) → 도착 확인 → STANDBY → 토크 해제
```

토크 해제 때 팔이 처질 수 있으므로 팔을 지지한다. CAN 또는 피드백이 끊긴 경우에는
확인할 수 없는 자세로 복귀하거나 토크를 해제하지 않는다.

## 데이터

```text
data/episodes/<episode-id>/
  metadata.json
  samples.jsonl
  camera/metadata.json
  camera/color/*.jpg
  global_camera/metadata.json
  global_camera/color/*.jpg
  validation.json
```

기록 항목:

- PICO pose, grip, trigger, XR timestamp
- PC monotonic/wall timestamp
- 명령 관절과 그리퍼 stroke(mm)
- PiPER 관절, gripper, end pose, 상태, error, enable
- Tracer 속도 명령, 상태, E-stop 상태
- 손목 InnoMaker RGB 경로, frame number, PC monotonic/wall timestamp
- 고정 Logitech C920 RGB 경로, frame number, PC monotonic/wall timestamp

에피소드 시작 시 손목 InnoMaker U20CAM과 global Logitech C920가 함께 기록된다.
두 카메라는 USB serial 기반 `/dev/v4l/by-id` 경로로 구분한다. 손목 카메라는 기본
`640x480`, `30 Hz`, JPEG이며 depth는 기록하지 않는다. Logitech은 기본 `1280x720`,
`15 Hz`, JPEG이다. global 카메라를 의도적으로 제외할 때만 실행 명령에
`--no-global-camera`를 붙인다.

최근 에피소드 검증:

```bash
cd ~/piper_teleop
episode=$(find data/episodes -mindepth 1 -maxdepth 1 -type d | sort | tail -1)
/home/swaaniida/miniconda3/envs/piper-xr/bin/python \
  -m single_piper_teleop.validate "$episode"
```

`"valid": true`인 에피소드만 사용한다.

## CAN 중단 후 복구

1. 팔을 지지한다.
2. CAN을 다시 연결하고 1 Mbps로 활성화한다.
3. `candump piper_can` 피드백을 확인한다.
4. 중단된 에피소드의 `metadata.json`에서 `session_start_joint_deg`를 확인한다.
5. 복귀 경로를 비우고 6개 값을 전달한다.

```bash
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.recover \
  --target J1 J2 J3 J4 J5 J6
```

복구는 50% 속도, 최대 5° waypoint로 진행한다. 목표 도착 확인 후 STANDBY와
토크 해제를 수행한다.

## 주의

- `ResetPiper()` 사용 금지
- firmware, zero, joint limit, CAN offset, installation pose, master/slave 설정 변경 금지
- 오류 또는 피드백 정지 시 새 목표 전송 금지
- 실제 동작은 작은 범위부터 확인

## VM 삭제 전 확인

Git에는 소스, submodule 커밋, ROS2 패치와 CAN serial 규칙만 저장한다. 다음 항목은
용량 또는 장비 종속성 때문에 저장소에 포함하지 않는다.

- `/opt/apps/roboticsservice`: XRoboToolkit PC Service 설치본
- `data/`: 수집 episode와 카메라 원본
- `config/tracer_return_route.json`, `config/tracer_start_pose.json`: 현재 공간에만 유효한 Tracer 경로
- PICO에 설치된 XRoboToolkit 앱과 설정

VM을 지우기 전에 위 항목 중 필요한 것은 별도 디스크에 복사하고, GitHub에서 새 clone이
되는지 확인한다. Tracer 경로는 새 환경에서 다시 teach하는 것이 안전하다.
