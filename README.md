# PICO–PiPER–Tracer Teleoperation

오른쪽 PICO 컨트롤러로 PiPER를, 왼쪽 컨트롤러로 Tracer Mini를 조작한다.
손목 RealSense 영상과 로봇/XR 데이터를 같은 episode에 저장한다.

## 조작

- 오른쪽 grip: 누르는 동안 이동
- 오른쪽 trigger: 그리퍼
- 왼쪽 grip + 왼쪽 stick: Tracer 전진/후진
- 왼쪽 grip + 오른쪽 stick: Tracer 회전
- A 버튼: 기본 home 복귀 후 토크 해제
- B 버튼: 현재 자세 hold 및 전체 일시정지; grip 해제·stick 중앙 후 다시 누르면 재개
- X 버튼: Tracer 범퍼 E-stop 해제
- Ctrl-C: 기본 home 복귀 후 토크 해제

## 구성

```text
single_piper_teleop/
  teleop.py       제어 및 데이터 기록
  realsense.py    손목 RGB-D
  validate.py     에피소드 검증
  recover.py      CAN 중단 후 복구
  preflight.py    실행 전 점검
tracer_mini/      Tracer PICO 제어와 E-stop 해제
assets/           PiPER URDF
src/              PiPER SDK, XR binding submodule
```

## 설치

테스트 환경은 Ubuntu 22.04, Python 3.10, `piper_sdk 0.6.1`, PiPER firmware
`S-V1.5-8`이다.

```bash
git clone --recurse-submodules https://github.com/swaaniida/pico-piper-teleop.git ~/piper_teleop
cd ~/piper_teleop
conda activate piper-xr
./install.sh
```

submodule 없이 clone한 경우:

```bash
git submodule update --init --recursive
./install.sh
```

XRoboToolkit PC Service는 `/opt/apps/roboticsservice`에 별도로 설치되어 있어야 한다.

## PICO 설정

```text
Tracking: Controller ON
Mode: Object
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

컨트롤러 pose, grip, trigger, A/B 버튼, RealSense 기록을 확인한다. dry-run은
로봇 명령을 보내지 않는다.

### 터미널 3: Tracer ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/tracer_ros2_ws/install/setup.bash
ros2 launch tracer_base tracer_mini_base.launch.py port_name:=tracer_can
```

현재 Tracer ROS2 source의 `TracerStatus.msg`에는 `uint8 vehicle_state`가 있어야 하며,
`tracer_messenger.hpp`가 `state.system_state.vehicle_state`를 publish해야 한다.

```bash
git -C ~/tracer_ros2_ws/src/tracer_ros2 apply \
  ~/piper_teleop/tracer_mini/tracer_vehicle_state.patch
cd ~/tracer_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tracer_msgs tracer_base
```

### 터미널 4: PiPER 단독 live

로봇 작업 반경을 비우고 B 버튼을 준비한다.

```bash
cd ~/piper_teleop
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.teleop
```

`READY`가 나오면 오른쪽 grip을 누른 상태에서 조작한다.

### 터미널 4: PiPER + Tracer 통합 live

```bash
source /opt/ros/humble/setup.bash
source ~/tracer_ros2_ws/install/setup.bash
cd ~/piper_teleop
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.teleop --with-tracer
```

참조 ZIP의 고정 start/init pose를 사용할 때만:

```bash
/home/swaaniida/miniconda3/envs/piper-xr/bin/python -u \
  -m single_piper_teleop.teleop --zip-auto-pose
```

이 pose는 해당 로봇에서 먼저 검증해야 한다.

## 종료

터미널에서 Ctrl-C를 한 번 누르거나 PICO의 A 버튼을 누른다. 둘 다 같은 정상
종료 절차를 실행한다.

```text
명령 중단 → 기본 home 복귀(50%) → 도착 확인 → STANDBY → 토크 해제
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
  camera/depth/*.png
  validation.json
```

기록 항목:

- PICO pose, grip, trigger, XR timestamp
- PC monotonic/wall timestamp
- 명령 관절과 그리퍼
- PiPER 관절, gripper, end pose, 상태, error, enable
- Tracer 속도 명령, 상태, E-stop 상태
- RGB/depth 경로, frame number, camera/PC timestamp

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

자세한 내용은 [PROJECT_CONTEXT](docs/PROJECT_CONTEXT.md)와
[SINGLE_PIPER_IK](docs/SINGLE_PIPER_IK.md)에 정리되어 있다.
