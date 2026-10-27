# PICO–PiPER–Tracer Teleoperation

PICO local/global 카메라 전체 시야 표시, 앱 빌드·설치와 전송 구조는
[PICO_CAMERA_VIEW.md](docs/PICO_CAMERA_VIEW.md)에 정리했다. Unity 변경 패치와
builder는 `deployment/pico-local-view/`에 있다.

오른쪽 PICO 컨트롤러로 PiPER를, 왼쪽 컨트롤러로 Tracer Mini를 조작한다.
손목 InnoMaker RGB 영상과 로봇/XR 데이터를 같은 episode에 저장한다.
새 InnoMaker U20CAM을 global camera로 함께 사용한다.

완전 초기 준비와 수집 시작은 각각 한 명령으로 실행한다.

```bash
./setup
./start
```

방금 저장한 episode를 수집 목록에서 제거하려면 `./rmlast`를 실행한다.
가장 최근 삭제본 하나는 USB에 백업되며, 다음 `./start`는 제거된 episode와
동일한 수집 위치부터 다시 시작한다. 잘못 제거했다면 `./restorelast`로 실제
데이터와 수집 순서를 함께 복구한다.

`start`는 이전 종료 뒤 `tracer_can`이 `ERROR-PASSIVE`로 남아 있으면 해당 CAN만
자동으로 down/up 복구한다. 이 경우에만 sudo 암호를 요구하며 전체 `setup`을 다시
실행할 필요가 없다. 복구 후에도 `ERROR-ACTIVE`가 아니면 Tracer 전원과 CAN 케이블을
확인하도록 실행을 중단한다.

일반 `./start`는 UUID `67EC-79D7`의 episode USB가 정상 마운트되고 쓰기 가능하며
5 GiB 이상 남아 있을 때만 실행된다. 새 데이터는 USB의
`piper_tracer/episodes/<episode-id>`에 직접 기록한다. USB가 없거나 읽기 전용으로
바뀌면 내부 디스크로 조용히 대체하지 않고 실행을 중단한다. init 보정은 데이터를
기록하지 않으므로 USB 검사를 생략한다.

바닥 집기용 init 자세는 다음 보정 명령으로 저장한다. 기존 init으로 안전하게
진입한 뒤 오른쪽 grip을 잡고 자세를 조정한다. 보정 중에는 J2·J3·J5만 움직이며
J1·J4·J6과 그리퍼 개폐값은 진입 순간 값으로 고정되고 Tracer 명령도 차단된다. grip을 놓고 A를 누르면
`config/floor_pick_init.json`에 현재 실측 관절값이 저장되며, 이후 `./start`의
A2 단계가 이 자세를 manipulation init으로 사용한다. 시작 자세는 기존 locomotion
init을 유지한다. B는 기존과 동일한 안전 종료이다.

```bash
./calibrate-init
```

Tracer 좌측 파란 컵 위의 공 놓기 직전 자세는 다음 명령으로 지정한다. 기존
locomotion init에서 시작하며 Tracer는 잠긴다. 오른쪽 grip으로 EE pose를 맞춘 뒤
grip을 놓고 A를 누르면 관절값과 link6 위치·quaternion·transform이
`config/cup_drop_pose.json`에 저장된다. 저장된 EE pose를 확인한 뒤 안전한 상향
이격·컵 상부 접근 waypoint를 생성해 episode 상태 머신에 적용한다.

```bash
./calibrate-drop
```

Global 화면의 녹색 반원은 locomotion 정지용 aid이다. 기존 박스의 가로 폭을
지름으로 하는 위쪽 반원이며, 중심선은 조금 아래로 배치한다. 화면에만 표시되고
원본 학습 이미지에는 기록되지 않는다.
비기록 공 세팅 단계에서는 높이 35.5cm, 수평 기준 하향 60°, U20CAM 수평 FOV
106°의 투영 모델로 다음 공 위치를 자홍색 점으로 표시한다. 시야 밖 목표는 화면
경계에 `OUT`으로 표시한다. 이 점 역시 미리보기에만 표시된다.
100회 수집 계획의 공 거리는 `45·50·60·65·70cm`, 좌우 위치는
`-20·-10·0·10·20cm`이다. 중앙 D70은 도달했던 episode 2를 유지하면서,
도달하지 못한 기존 R30/D75 결합 극단을 R20/D70 안쪽으로 축소했다.

## 조작

- 오른쪽 grip: 누르는 동안 이동
- 오른쪽 trigger: 그리퍼
- 왼쪽 grip + 왼쪽 stick: Tracer 전진/후진
- 왼쪽 grip + 오른쪽 stick: Tracer 회전
- 프로그램을 처음 켠 `SET BALL` 단계에서도 왼쪽 grip+stick으로 Tracer를 배치할 수 있으며 PiPER는 잠겨 있다.
- A1: 기존 locomotion init에서 episode 녹화 시작 (`locomotion`)
- A2: 공이 Global aid에 들어오면 locomotion 완료 기록 후 manipulation init 진입
- A3: 공 집기 완료 기록 후 locomotion init → 컵 상부 경유점 → 투하 자세 → 그리퍼 자동 개방과 동시에 cup clearance로 상승 → locomotion init 복귀 → episode 저장 종료
- A2/A3의 모든 자동 trajectory도 50 Hz로 실측 state, commanded action, 카메라 참조를 기록하며 `control_source=automatic`과 세부 subphase를 저장
- 각 sample은 PiPER 관절·그리퍼 실측값, absolute 관절·그리퍼 명령, Tracer `cmd_vel`과 `/odom`, local/global 원본 프레임 참조를 함께 저장한다. 원본은 변환하지 않으며 학습 시 동일 episode에서 absolute, stepwise delta, chunkwise delta action을 파생한다.
- 카메라는 캡처/JPEG 인코딩과 USB 쓰기를 분리한 10초 RAM queue를 사용하고, `samples.jsonl`도 별도 비동기 writer가 기록한다. queue 초과나 쓰기 실패는 프레임을 조용히 버리지 않고 episode 오류로 처리하며, 정상 종료는 모든 queue drain과 JSONL `fsync` 이후에만 확정한다.
- 첫 학습 baseline은 Mobile ALOHA ACT와 같이 phase/progress auxiliary head 없이 두 카메라와 proprioception에서 whole-body absolute action chunk를 예측한다. `episode_phase`, `control_source`, `trajectory_subphase`는 분석·구간별 평가용 metadata이며 baseline 입력이나 출력으로 사용하지 않는다.
- A3 이후: 비기록 상태에서 joystick으로 공과 Tracer를 원위치로 초기화
- 초기화 후 A1: PiPER init 복귀·도착 확인 후 다음 episode 녹화 시작
- 프로그램 시작 직후와 A3 종료 뒤에도 카메라 미리보기를 유지하며, 다음 공 위치 안내를 보고 세팅한 후 A1으로 녹화를 시작
- A2 manipulation init 및 A3/다음 A1의 locomotion init 복귀: 관절 최대 `25°/s`, 속도 제한 50%로 50 Hz 보간
- B 버튼: 즉시 전체 정지지·카메라 기록 중단 후 USB queue 정리를 기다리지 않고 ZIP start 복귀, 토크 해제 뒤 미완성 episode 정리 및 프로그램 종료
- X 버튼: 사용하지 않음. 잘못 저장한 최근 episode는 프로그램 종료 후 `./rmlast`로 제거
- Y 버튼: Tracer 범퍼 E-stop 해제
- 오른쪽 조이스틱 클릭: PICO 표시를 local/global camera로 전환
- episode 녹화 중 local/global InnoMaker 영상을 함께 표시
- A3 이후 수동 원상복구 중에도 미리보기는 유지하고 REC 표시와 프레임 저장만 중지
- 미리보기 상단에 현재 단계와 다음 A 입력을 짧게 표시
- 미리보기 창의 Q 또는 Esc: 안전 종료 요청
- Ctrl-C: B와 동일하게 즉시 hold → ZIP start 복귀 → 토크 해제 → 종료

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
`[-1.0, 23.84, -60.0, 0.0, 50.65, -1.5]°`이다.
원본 ZIP의 start J3 `+1.60°`는 이 PiPER에서 `0°` 경계에 걸리므로,
실측 도달 자세인 `0°`를 사용한다.

```text
A1: episode 시작 → locomotion
A2: locomotion_complete 기록 → manipulation init → manipulation
A3: pick_complete 기록 → locomotion init → 컵 상부 안전점 → 투하 자세 → 자동 개방과 동시에 안전점으로 상승 → locomotion init → episode 저장 종료
비기록 수동 초기화 후 A1: PiPER init 복귀 확인 → 다음 episode 시작
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
- global InnoMaker RGB 경로, frame number, PC monotonic/wall timestamp

에피소드 시작 시 손목과 global InnoMaker U20CAM을 함께 기록한다. 두 장치가 모두
동일한 `SN0001` serial을 제공하므로 USB 물리 포트 기반 `/dev/v4l/by-path`로
구분한다. 손목은 USB `3.4.4`, global은 USB `3.3` 포트에 고정한다.

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

## 학습 checkpoint와 추론 PC

Eddy의 checkpoint binary는 Git commit이 아니라 GitHub Release에 있다. 현재 모델은
`deployment/models/`의 manifest가 source of truth다. 추론 PC에서는 아래처럼 모델을
직접 받고, 실제 명령을 전혀 보내지 않는 shadow 검증부터 수행한다.

```bash
python3 tools/fetch_rollout_model.py act-downsample-240x320-b16-step-200000
./rollout-shadow \
  checkpoints/act-downsample-240x320-b16-step-200000/\
floor-1ball-act-downsample-240x320-b16-step-200000 \
  --seconds 30
```

전체 계약, exact version, checksum, 단계별 live 승인 조건은
`docs/INFERENCE_ROLLOUT.md`를 따른다. 현재 `rollout-shadow`에는 의도적으로 actuator
송신 코드가 없다.

게시된 세 ACT 모델을 같은 local/global frame과 robot state로 비교하면서 영상을
저장하려면 다음을 사용한다.

```bash
./rollout-fetch-all
./rollout-setup
./rollout-compare --seconds 60
```

각 실행은 `log/rollout-shadow-<timestamp>/`에 `local.avi`, `global.avi`, 모델별
action/latency가 든 `samples.jsonl`을 저장한다. 세 번째 batch-64 모델은 200k가 아닌
정상 종료된 50k checkpoint다.

## VM 삭제 전 확인

Git에는 소스, submodule 커밋, ROS2 패치와 CAN serial 규칙만 저장한다. 다음 항목은
용량 또는 장비 종속성 때문에 저장소에 포함하지 않는다.

- `/opt/apps/roboticsservice`: XRoboToolkit PC Service 설치본
- `data/`: 수집 episode와 카메라 원본
- `config/tracer_return_route.json`, `config/tracer_start_pose.json`: 현재 공간에만 유효한 Tracer 경로
- PICO에 설치된 XRoboToolkit 앱과 설정

VM을 지우기 전에 위 항목 중 필요한 것은 별도 디스크에 복사하고, GitHub에서 새 clone이
되는지 확인한다. Tracer 경로는 새 환경에서 다시 teach하는 것이 안전하다.
