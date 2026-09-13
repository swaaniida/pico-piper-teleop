# 추론 PC 지시: floor-1ball ACT 세 모델 비교

## 목표

GitHub 저장소 `swaaniida/pico-piper-teleop`의
`inference-rollout-20260914` 브랜치에서 세 checkpoint를 직접 받고, 동일한
local/global 영상과 동일한 measured state로 no-motion shadow 비교를 수행한다.

이 단계에서는 실제 로봇을 움직이지 않는다. PiPER `EnableArm`, `JointCtrl`,
`GripperCtrl` 및 Tracer `/cmd_vel` command를 보내지 않는다.

## 시작

```bash
git clone --recurse-submodules \
  --branch inference-rollout-20260914 \
  https://github.com/swaaniida/pico-piper-teleop.git ~/piper-tracer
cd ~/piper-tracer
git rev-parse HEAD
git submodule update --init --recursive
```

이미 clone했다면:

```bash
cd ~/piper-tracer
git fetch origin
git switch inference-rollout-20260914
git pull --ff-only
git submodule update --init --recursive
```

기대 commit은 `44cc62784dc25f374d680c63748656b163a51609` 이상이다.

## 먼저 읽을 파일

1. `AGENTS.md`가 존재하면 `AGENTS.md`
2. `README.md`
3. `docs/INFERENCE_ROLLOUT.md`
4. `prompts/INFERENCE_PC_BRINGUP.md`
5. `deployment/models/*/manifest.json`

## 비교 모델

- `act-absolute-step-200000`
- `act-downsample-240x320-b16-step-200000`
- `act-downsample-240x320-b64-step-50000`

세 번째 모델은 batch 64의 정상 종료된 **50000-step** checkpoint다. 200000-step으로
표기하거나 추측하지 않는다.

모두 LeRobot 0.4.4, torch 2.6.0, torchvision 0.21.0 기준이다. 각 Release와
manifest의 SHA-256을 반드시 검증한다.

## 실행 순서

1. GPU/driver/CUDA와 Python package version을 기록한다.
2. 다음 명령으로 세 checkpoint를 받는다.

```bash
./rollout-fetch-all
```

3. 추론 PC에서 두 InnoMaker의 물리 `/dev/v4l/by-path`를 정확히 식별한다. 현재 PC와
   USB topology가 다르면 문서의 기존 path를 그대로 사용하지 않는다.
4. 실제 path를 지정해 no-motion setup을 수행한다.

```bash
PIPER_LOCAL_CAMERA=/dev/v4l/by-path/<local> \
PIPER_GLOBAL_CAMERA=/dev/v4l/by-path/<global> \
./rollout-setup
```

5. PiPER/Tracer feedback과 local/global frame을 read-only로 확인한다.
6. 세 모델 shadow 비교를 실행한다.

```bash
./rollout-compare --seconds 60
```

## 결과 계약

`log/rollout-shadow-<timestamp>/`에 다음이 있어야 한다.

```text
local.avi
global.avi
samples.jsonl
metadata.json
```

두 영상의 frame N과 JSONL의 `video_frame_index=N`은 같은 observation이다.
각 모델은 이 동일 observation과 동일 9D state를 받아야 한다.

다음을 분석하고 보고한다.

- source commit과 submodule commit
- 모델별 Release asset SHA-256
- GPU/driver/CUDA/torch/torchvision/LeRobot version
- 두 camera의 물리 identity와 tensor shape/color/order
- 모델별 inference latency p50/p95/max
- 전체 tick과 50 Hz deadline miss 수·비율
- NaN/Inf 및 safety rejection 여부
- raw action과 gated action의 clipping 횟수·크기
- 모델 간 joint/gripper/base action 차이
- action discontinuity와 100-step chunk 경계 jump
- local/global 영상 frame 수와 JSONL 대응 여부
- GPU peak VRAM

현재 세 모델은 `chunk_size=100`, `n_action_steps=100`, temporal ensemble 없음이다.
50 Hz 기준 약 2초 open-loop가 될 수 있으므로 실기에 안전하다고 가정하지 않는다.

## 중단 조건

이 프롬프트의 범위는 offline/no-motion shadow까지다. 실제 actuator command를 연결하거나
PiPER torque를 enable하지 말고, 결과와 문제를 사용자에게 먼저 보고한다. 실제 rollout은
모델별로 같은 초기조건에서 한 모델씩 수행하며 별도 승인과 live safety gate가 필요하다.
