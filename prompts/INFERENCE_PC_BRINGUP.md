# 추론 PC 작업 지시: PiPER–Tracer guarded rollout 준비

## 목표

GitHub의 소스와 지정된 checkpoint Release를 직접 받아, 수집 때와 동일한
PiPER–Tracer–dual-InnoMaker 구조에서 rollout을 준비한다.

## 작업 시작 시 사용자에게 받아야 할 값

```text
SOURCE_REPO=swaaniida/pico-piper-teleop
SOURCE_REF=<rollout source branch/commit supplied by the operator>
MODEL_ID=act-absolute-step-200000 또는 act-downsample-240x320-b16-step-200000
```

값을 추측하지 않는다.

## 먼저 읽을 파일

1. `AGENTS.md`가 있으면 가장 먼저 읽기
2. `README.md`
3. `docs/INFERENCE_ROLLOUT.md`
4. `deployment/models/<MODEL_ID>/manifest.json`

## 작업 원칙

- 실제 로봇은 사용자의 명시적 승인 전까지 움직이지 않는다.
- 먼저 offline 및 no-motion 검증만 수행한다.
- 추론 PC에서 학습하거나 dataset 통계를 다시 계산하지 않는다.
- checkpoint config, processor, normalization을 수정하지 않는다.
- 카메라 순서·색상·resize/pad와 state/action 단위를 manifest와 대조한다.
- 기존 CAN network interface를 이름만 보고 재설정하지 말고 USB serial부터 확인한다.

## 순서

### 1. 소스 고정

```bash
git clone --recurse-submodules https://github.com/swaaniida/pico-piper-teleop.git ~/piper-tracer
cd ~/piper-tracer
git checkout <SOURCE_REF>
git submodule update --init --recursive
git status --short
```

### 2. checkpoint를 GitHub에서 직접 받기

```bash
cd ~/piper-tracer
python3 tools/fetch_rollout_model.py <MODEL_ID>
```

출력된 bundle 경로로 다음 검증기를 실행한다.

```bash
python ~/piper-tracer/tools/verify_rollout_bundle.py <bundle-directory>
```

### 3. 환경 재현

- Ubuntu/ROS/CAN/camera 요구사항을 `README.md`대로 설치
- 학습 container digest 또는 manifest에 고정된 LeRobot/PyTorch 버전 사용
- `nvidia-smi`, torch CUDA, LeRobot version 기록
- 두 CAN adapter USB serial과 link rule 대조
- 물리 카메라 identity를 by-id/by-path로 대조

### 4. Offline gate

- checkpoint 공식 loader로 load
- dataset sample 또는 기록된 fixture 하나로 forward
- 모든 output finite 및 shape 9D 확인
- 모델별 action representation decode unit test
- 같은 fixture에 대한 Eddy reference output이 있으면 수치 비교

### 5. Hardware no-motion gate

- PiPER/Tracer feedback만 읽기
- command 송신 함수는 mock/disabled 유지
- local/global camera 화면과 tensor dump 확인
- state 순서와 단위 확인
- 50 Hz loop latency, inference latency, frame age 측정

### 6. Shadow rollout

- policy output을 계산하고 log하지만 hardware command는 보내지 않음
- joint/gripper/base safety gate의 accept/reject 결과 기록
- stop 입력과 stale watchdog을 fault injection으로 검증

### 7. 실제 motion 전 중단

위 결과를 사용자에게 보고한다. 사용자가 명시적으로 승인한 뒤에만
`docs/INFERENCE_ROLLOUT.md`의 separated actuation gate로 넘어간다.

## 최초 보고에 반드시 포함할 내용

```text
source commit/submodules
model manifest and Release checksum
GPU/driver/CUDA/torch/LeRobot versions
camera identities and tensor shapes
CAN identities and feedback rates
policy latency p50/p95/max
50 Hz deadline miss count
safety gate/fault injection results
remaining blockers
```
