# PiPER–Tracer 실환경 추론 배포 기준

이 문서는 Eddy에서 학습한 `floor-1ball` 정책을 별도 추론 PC로 옮겨, 데이터
수집 때와 동일한 PiPER–Tracer–InnoMaker 구조에서 안전하게 rollout하기 위한 기준이다.

## 결론

소스와 작은 설정은 GitHub에 두고, checkpoint는 **GitHub Release asset**으로
배포한다. checkpoint를 일반 Git commit에 넣지 않는다. 각 모델은 반드시
`pretrained_model/`, 학습 설정, dataset statistics, 배포 manifest, SHA-256을 한
bundle로 묶는다.

추론 코드는 기존 teleoperation main loop에 임시로 끼워 넣지 않는다. 카메라와
hardware helper는 공유하되, 별도 rollout entry point에서 다음 순서를 강제한다.

```text
preflight → 카메라/feedback 준비 → PiPER locomotion init → 사용자 ARM 확인
→ policy reset → autonomous rollout → hold → ZIP start 복귀 → torque release
```

실제 checkpoint 형식과 학습 config를 가져오기 전에는 로봇 명령을 연결하지 않는다.

## 1. 학습과 추론에서 같아야 하는 계약

GPU 종류는 달라도 되지만 아래 항목은 완전히 같아야 한다.

| 항목 | 현재 데이터 계약 |
|---|---|
| Dataset | `swaaniida/floor-1ball`, LeRobot v3 |
| Control rate | 50 Hz |
| Local image | RGB, CHW policy input, source 640×480 |
| Global image | RGB, CHW policy input, source 640×480 |
| State | 9D float: joints rad ×6, measured gripper mm, base measured v/w |
| Absolute action | 9D float: joint target rad ×6, gripper target mm, base cmd v/w |
| Camera key order | `observation.images.local`, `observation.images.global` |
| State key | `observation.state` |
| Action key | `action` |
| Phase/progress | baseline policy 입력 아님 |

모델마다 absolute/stepwise-delta/chunkwise-delta 중 무엇을 예측하는지 manifest에
명시한다. 추론 adapter는 그 표현만 physical absolute command로 복원한다. 표현을
파일명으로 추측하면 안 된다.

정규화 mean/std는 학습 dataset의 통계여야 한다. 추론 PC에서 새로 계산하거나
수동으로 근사하지 않는다. LeRobot checkpoint는 `config.json`, weights와 통계 기반
normalization buffer를 함께 보존해야 한다.

## 2. 현재 로봇 하드웨어 계약

### PiPER

- SocketCAN 이름: `piper_can`
- bitrate: 1,000,000 bit/s
- SDK: repository submodule의 `piper_sdk`
- command 단위 변환: policy rad → degree → SDK `degree × 1000`
- gripper: policy mm → SDK `mm × 1000`
- 유효 범위: gripper 0–70 mm
- 시작 순서:
  - ZIP start: `[-0.51, 0.49, 0.0, 3.96, 4.83, -8.33]°`
  - locomotion init: `[-1.0, 23.84, -60.0, 0.0, 50.65, -1.5]°`
- task-specific pose:
  - `config/floor_pick_init.json`
  - `config/cup_drop_pose.json`

### Tracer Mini

- SocketCAN 이름: `tracer_can`
- bitrate: 500,000 bit/s
- ROS topics: `/cmd_vel`, `/tracer_status`, `/odom`
- 현재 학습 범위: linear ±0.15 m/s, angular ±0.40 rad/s
- command watchdog: 200 ms 이상 갱신이 없으면 반드시 0속도
- `tracer_mini/tracer_vehicle_state.patch`가 적용된 정확한 ROS workspace 필요
  - `ugv_sdk`: `58436e9c1732474566e249ce7f726e12e26304d6`
  - `tracer_ros2`: `832ea9a48fe725d47d766a099d358062ea2f0f31`

### Cameras

두 물리 카메라의 `/dev/videoN` 번호는 재부팅 때 달라질 수 있다. 추론 PC에서는
`/dev/v4l/by-id` 또는 `/dev/v4l/by-path`로 물리 장치를 확인하고, v4l2loopback
출력만 `/dev/video10`, `/dev/video11`로 고정한다.

- Wrist/local: 물리 1280×720 MJPEG 30 Hz → `scale=640:480`
- Global: 물리 1280×720 MJPEG 30 Hz → `scale=640:360` 후 위아래 60 px black pad
- policy에 넣는 영상에는 visual aid, REC, status text를 burn-in하지 않는다.
- OpenCV BGR을 그대로 넣지 말고 학습과 동일한 RGB로 변환한다.
- 50 Hz policy tick에서 카메라가 새 프레임을 내지 않으면 가장 최근의 유효한
  30 Hz frame을 사용한다. 단, frame age가 safety threshold를 넘으면 rollout을
  중단한다.

현재 fanout의 정확한 명령은 `setup`, 캡처 구현은
`single_piper_teleop/wrist_camera.py`와 `single_piper_teleop/global_camera.py`가
source of truth다.

## 3. 추론 PC에 필요한 repository 파일

다음 파일은 반드시 같은 Git commit으로 받아야 한다.

```text
README.md
pyproject.toml
requirements-single-piper.txt
install.sh
setup                         # 참고용; 추론 전용 setup은 별도로 만들 것
config/
assets/piper_kinematics.urdf
single_piper_teleop/
tracer_mini/
src/piper_sdk/                # submodule
src/XRoboToolkit-PC-Service-Pybind/  # autonomous rollout에는 보통 불필요
docs/INFERENCE_ROLLOUT.md
deployment/checkpoint-manifest.example.json
tools/verify_rollout_bundle.py
```

Autonomous inference에는 PICO pose나 XRoboToolkit이 필요하지 않다. 다만 PICO를
카메라 monitor/E-stop UI로 계속 쓸 경우 video sender만 선택적으로 유지하고,
controller tracking을 policy input에 섞지 않는다.

현재 working tree에는 수집 중 생성된 미커밋 변경과 untracked 실행 파일이 있다.
추론 PC 배포 전 `setup`, `start`, camera code, Tracer patch, calibration JSON을 포함한
재현 가능한 commit/tag를 먼저 만들어야 한다. `.runtime`, logs, episode data,
credential은 commit하지 않는다.

## 4. checkpoint를 GitHub에 올리는 형식

GitHub repository에는 아래 작은 파일만 commit한다.

```text
deployment/models/<model-id>/
  manifest.json
  SHA256SUMS
  metrics.json
  training-command.txt
```

큰 bundle은 GitHub Release asset으로 올린다.

```text
floor-1ball-<model-id>.tar.zst
floor-1ball-<model-id>.tar.zst.sha256
```

Release asset은 파일 하나당 2 GiB 미만이어야 한다. 2 GiB를 넘으면 `split`으로
나누거나 Hugging Face model repository를 사용한다. 일반 Git에 checkpoint binary를
commit하지 않는다.

Eddy에서 모델마다 다음 항목을 수집한다.

```text
<run>/checkpoints/<selected-step>/pretrained_model/
<run>/checkpoints/<selected-step>/pretrained_model/config.json
<run>/checkpoints/<selected-step>/pretrained_model/model.safetensors
<run>/checkpoints/<selected-step>/pretrained_model/train_config.json (존재하는 경우)
실제로 사용한 train command/config
dataset revision/commit
LeRobot, torch, torchvision, CUDA 버전
Git source commit
validation loss와 실제 rollout 전 선택 근거
```

LeRobot에서 선택 checkpoint의 `pretrained_model/` 전체가 배포 단위다. weights 파일
하나만 복사하면 normalization과 feature config가 빠질 수 있다.

예시 packaging:

```bash
MODEL_ID=act-absolute-step-100000
SELECTED=/path/to/run/checkpoints/100000/pretrained_model
STAGE=/tmp/floor-1ball-$MODEL_ID

mkdir -p "$STAGE"
cp -a "$SELECTED" "$STAGE/pretrained_model"
cp deployment/models/$MODEL_ID/manifest.json "$STAGE/manifest.json"
(cd "$STAGE" && find . -type f ! -name SHA256SUMS -print0 \
  | sort -z | xargs -0 sha256sum > SHA256SUMS)
tar --zstd -C /tmp -cf "floor-1ball-$MODEL_ID.tar.zst" "floor-1ball-$MODEL_ID"
sha256sum "floor-1ball-$MODEL_ID.tar.zst" \
  > "floor-1ball-$MODEL_ID.tar.zst.sha256"
```

GitHub CLI로 release에 첨부:

```bash
gh release create "floor-1ball-$MODEL_ID" \
  "floor-1ball-$MODEL_ID.tar.zst" \
  "floor-1ball-$MODEL_ID.tar.zst.sha256" \
  --title "floor-1ball $MODEL_ID" \
  --notes "See deployment/models/$MODEL_ID/manifest.json"
```

## 5. 추론 PC에서 checkpoint 받기

```bash
MODEL_ID=act-absolute-step-100000
mkdir -p ~/piper-tracer/checkpoints/$MODEL_ID
cd ~/piper-tracer/checkpoints/$MODEL_ID

gh release download "floor-1ball-$MODEL_ID"
sha256sum -c "floor-1ball-$MODEL_ID.tar.zst.sha256"
tar --zstd -xf "floor-1ball-$MODEL_ID.tar.zst"

python ~/piper-tracer/tools/verify_rollout_bundle.py \
  "floor-1ball-$MODEL_ID"
```

Release와 source tag를 동일한 model ID로 묶고 manifest의 `source_git_commit`을
checkout한다.

```bash
git fetch --tags
SOURCE_COMMIT="$(python -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["source_git_commit"])' \
  "floor-1ball-$MODEL_ID/manifest.json")"
git checkout "$SOURCE_COMMIT"
git submodule update --init --recursive
```

## 6. 추론 전 소프트웨어 호환성

가장 안전한 방식은 Eddy 학습 image의 digest를 그대로 추론 PC에서 쓰는 것이다.
GPU가 달라도 NVIDIA driver가 container CUDA runtime을 지원하면 된다.

반드시 기록·비교할 것:

```bash
nvidia-smi
python -c 'import torch; print(torch.__version__, torch.version.cuda)'
python -c 'import lerobot; print(lerobot.__version__)'
pip freeze > inference-pip-freeze.txt
git rev-parse HEAD
git submodule status --recursive
```

학습과 추론의 LeRobot major/minor를 임의로 바꾸지 않는다. 최신 LeRobot CLI의
`lerobot-rollout`은 표준 robot plugin을 전제로 하므로, 현재 custom PiPER–Tracer
hardware adapter가 완성되기 전에는 이 repository의 전용 rollout loop를 사용해야
한다.

## 7. 전용 rollout loop 요구사항

두 checkpoint의 config가 확인되어 `single_piper_rollout`에 no-motion adapter를
구현했다. 현재 공개 entry point는 의도적으로 **shadow 전용**이며 motion command를
전혀 보내지 않는다. live command adapter는 아래 Gate 0/1 결과와 ACT replan 방식을
확정한 뒤 별도 승인으로 연결한다.

1. `ACTPolicy.from_pretrained(<local pretrained_model>)` 공식 loader 사용
2. `policy.reset()`을 episode마다 한 번 호출
3. 두 카메라와 measured state를 정확한 key/shape/device로 구성
4. normalization은 checkpoint에 저장된 processor/statistics 사용
5. `torch.inference_mode()`와 eval mode
6. chunk queue의 `n_action_steps`, temporal ensemble 여부를 checkpoint config대로 적용
7. output representation을 manifest대로 physical absolute action으로 decode
8. PiPER/Tracer 각각 별도 clipping, slew-rate limit, stale-input watchdog
9. command loop deadline miss가 연속되면 즉시 hold/zero
10. Ctrl-C/B/물리 E-stop에서 Tracer zero → PiPER hold → guarded return 선택
11. rollout 최대 시간과 operator-confirmed 종료
12. command/state/image timestamp와 safety event를 별도 evaluation log에 저장

### 현재 게시된 모델

| model ID | Release tag | 입력 | 특이사항 |
|---|---|---|---|
| `act-absolute-step-200000` | `floor-1ball-act-absolute-step-200000` | 480×640 | 원본 해상도 |
| `act-downsample-240x320-b16-step-200000` | `floor-1ball-act-downsample-240x320-b16-step-200000` | 240×320 | torchvision antialias resize |

둘 다 LeRobot 0.4.4, absolute 9D action, `chunk_size=100`,
`n_action_steps=100`, temporal ensemble 없음이다. validation/physical rollout metric은
없으므로 마지막 training loss만으로 우열을 정하지 않는다.

### 다운로드와 no-motion 실행

```bash
cd ~/piper-tracer
python3 tools/fetch_rollout_model.py act-downsample-240x320-b16-step-200000

# 추론 PC의 USB path가 다르면 실제 by-path 값을 지정한다.
PIPER_LOCAL_CAMERA=/dev/v4l/by-path/<local> \
PIPER_GLOBAL_CAMERA=/dev/v4l/by-path/<global> ./rollout-setup

python3 tools/verify_rollout_bundle.py \
  checkpoints/act-downsample-240x320-b16-step-200000/\
floor-1ball-act-downsample-240x320-b16-step-200000

./rollout-shadow \
  checkpoints/act-downsample-240x320-b16-step-200000/\
floor-1ball-act-downsample-240x320-b16-step-200000 \
  --seconds 30
```

`rollout-shadow`는 PiPER/Tracer feedback과 두 카메라를 읽고 policy output 및 final
safety gate 결과만 `log/rollout-shadow.jsonl`에 기록한다. PiPER enable, JointCtrl,
GripperCtrl, `/cmd_vel` 명령은 호출하지 않는다.

`rollout-setup`도 XRoboToolkit/PICO service를 시작하지 않는다. SocketCAN, Tracer ROS,
학습과 동일한 두 camera fanout만 준비한 뒤 feedback을 read-only로 확인한다.

ACT의 현재 `select_action()`은 checkpoint에 저장된 100개 action queue를 소비하므로
첫 추론 이후 약 2초 동안 새 관측이 제어에 반영되지 않는다. 이 동작은 학습 config와
일치하지만 실환경에 안전하거나 최적이라고 검증된 것은 아니다. live 연결 전 다음
세 방식 중 하나를 동일한 recorded observation으로 비교한다.

- 원 설정: 100 step open-loop
- 짧은 replan horizon(예: 5 또는 10 step) 후 queue reset
- 매 tick chunk prediction + temporal ensemble

이 선택은 policy 의미를 바꾸므로 shadow latency·action 연속성과 separated actuation을
통과하기 전 기본값을 조용히 변경하지 않는다.

## 8. 안전 제한은 모델 밖에 둔다

모델 출력은 신뢰하지 않는다. 아래 제한은 policy와 독립된 final command gate에서
항상 적용한다.

- 모든 값 finite 검사
- PiPER hardware joint limits 내부인지 검사
- 학습 데이터 범위 및 별도 운영 envelope 검사
- tick당 joint target 변화량 제한
- gripper 0–70 mm clamp
- Tracer linear ±0.15 m/s, angular ±0.40 rad/s clamp
- camera/state feedback stale 검사
- PiPER error/status 검사
- inference timeout과 loop overrun 검사
- E-stop/keyboard stop은 model thread보다 높은 우선순위

처음에는 바퀴를 지면에서 띄우고 PiPER torque off 상태로 observation/output만 기록하는
shadow test를 한다. 그 다음 PiPER만 낮은 speed limit, 그 다음 Tracer만, 마지막에
통합 rollout 순서로 진행한다.

## 9. 실환경 검증 단계

### Gate 0: offline

- bundle checksum 통과
- checkpoint load 통과
- dataset sample replay에서 output shape/finite 통과
- 같은 입력을 Eddy와 추론 PC에 넣었을 때 허용 오차 내 output 일치

### Gate 1: hardware no-motion

- CAN feedback만 연결
- local/global frame의 색상·crop·pad·camera identity 확인
- state 9D 순서와 단위 확인
- policy output을 보내지 않고 log만 남김

### Gate 2: separated actuation

- PiPER 고정/Tracer zero 상태에서 arm command gate 확인
- PiPER hold 상태에서 Tracer command gate와 200 ms watchdog 확인
- 낮은 속도와 넓은 작업 공간에서 수행

### Gate 3: integrated guarded rollout

- 물리 E-stop 담당자 배치
- 한 episode씩 operator ARM
- 초기 10회는 성공 여부와 intervention 원인을 수동 기록
- 자동 복귀가 학습 target에 포함됐더라도 rollout maximum duration을 둠

현재 dataset은 49 episodes이며 episode 종료 classifier가 따로 없다. 정책이 init
근처로 돌아왔다는 이유만으로 자동 torque-off하지 않는다. 최대 시간 또는 명시적
안전 종료 조건을 사용한다.

## 10. Eddy에서 받은 정보와 남은 항목

다음 항목은 두 Release/metadata commit에서 확보됐다.

- model ID와 policy type
- selected checkpoint directory
- 전체 `pretrained_model/` 파일 목록
- training command/config
- LeRobot version/commit
- action representation
- `chunk_size`, `n_action_steps`, temporal ensemble 설정
- image resize/crop/augmentation 중 inference에 남는 transform
- dataset revision 및 train episode count(49/99/100 등)
- validation 기준과 selected step
- checkpoint당 파일 크기

남은 핵심 항목은 validation/실환경 rollout 지표와 Eddy reference fixture output이다.
소스 metadata는 branch `publish-floor-1ball-act-checkpoints-20260913`, commit
`3d93cd363e221fe7abe46e911b8ccef01747d3aa`에서 왔다.
