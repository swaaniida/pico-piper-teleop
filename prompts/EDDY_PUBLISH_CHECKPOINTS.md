# Eddy 작업 지시: `floor-1ball` checkpoint 직접 배포

## 목표

Eddy 로컬에 있는 학습 checkpoint를 다른 PC로 복사하지 말고 Eddy에서 직접
GitHub repository `swaaniida/pico-piper-teleop`의 Release asset으로 배포한다.
소스 repository에는 작은 manifest와 학습 설정만 commit하고 model binary는
commit하지 않는다.

## 먼저 읽을 파일

1. `docs/INFERENCE_ROLLOUT.md`
2. `deployment/checkpoint-manifest.example.json`
3. 학습 run의 `train_config.json`과 실제 실행 명령
4. 학습 dataset의 `meta/info.json`

## 절대 하지 않을 것

- checkpoint를 일반 Git commit에 추가하지 않는다.
- dataset 전체를 GitHub에 중복 업로드하지 않는다.
- checkpoint, config, 통계 파일을 임의로 변환하거나 이름만 보고 설정을 추측하지 않는다.
- 기존 학습 결과를 삭제하거나 덮어쓰지 않는다.
- token, password, W&B credential을 commit하지 않는다.

## 작업 순서

### 1. 저장소 준비

```bash
cd ~/piper-tracer
git remote -v
git fetch origin
git status --short
gh auth status
```

현재 작업이 있으면 보존하고, 충돌 없이 별도 branch에서 metadata만 작성한다.

### 2. 모든 후보 모델 조사

각 학습 run에 대해 아래 표를 작성한다.

```text
model_id
policy_type
action_representation
selected_step
checkpoint_path
checkpoint_size
training_dataset_repo/revision/episode_count
lerobot_version
torch/torchvision/CUDA versions
chunk_size/n_action_steps
temporal_ensemble_coeff
image transforms
validation metric/loss
source Git commit
```

`absolute`, `stepwise_delta`, `chunkwise_delta` 모델을 이름이 아니라 실제 config와
training transform으로 판별한다. selected checkpoint가 여러 개면 모두 올리지 말고
우선 best/last 후보와 선택 근거를 보고한다.

### 3. 모델별 manifest 작성

```bash
mkdir -p deployment/models/<model-id>
cp deployment/checkpoint-manifest.example.json \
   deployment/models/<model-id>/manifest.json
```

모든 `REQUIRED` 값을 실제 값으로 채운다. 추가로 다음 파일을 둔다.

```text
deployment/models/<model-id>/manifest.json
deployment/models/<model-id>/metrics.json
deployment/models/<model-id>/training-command.txt
deployment/models/<model-id>/environment.txt
```

`environment.txt`에는 최소한 다음 결과를 기록한다.

```bash
python -c 'import lerobot,torch,torchvision; print(lerobot.__version__, torch.__version__, torchvision.__version__, torch.version.cuda)'
git rev-parse HEAD
git submodule status --recursive
```

### 4. checkpoint bundle 생성

LeRobot 선택 checkpoint의 `pretrained_model/` 디렉터리 전체를 복사한다.
weights만 복사하지 않는다.

```text
floor-1ball-<model-id>/
  pretrained_model/
  manifest.json
  metrics.json
  training-command.txt
  environment.txt
  SHA256SUMS
```

모든 파일의 SHA-256을 만든 뒤 `.tar.zst`로 묶는다. 생성 후 원본과 bundle의
파일 목록·크기·checksum을 검증한다. Release asset 하나가 2 GiB 이상이면 1900 MiB
이하 조각으로 나누고 각 조각 checksum을 만든다.

### 5. metadata commit/push

checkpoint binary가 staging되지 않았는지 확인한다.

```bash
git status --short
git diff --cached --stat
git diff --cached --check
```

manifest/config 문서만 commit하고 branch를 push한다. 기존 remote 변경을 강제로
덮어쓰지 않는다. 필요하면 PR을 만든다.

### 6. Eddy에서 GitHub Release 직접 업로드

```bash
gh release create floor-1ball-<model-id> \
  floor-1ball-<model-id>.tar.zst \
  floor-1ball-<model-id>.tar.zst.sha256 \
  --repo swaaniida/pico-piper-teleop \
  --title "floor-1ball <model-id>" \
  --notes "See deployment/models/<model-id>/manifest.json"
```

분할했다면 모든 조각과 checksum 파일을 같은 Release에 첨부한다.

### 7. 원격 검증

```bash
gh release view floor-1ball-<model-id> \
  --repo swaaniida/pico-piper-teleop
```

임시 디렉터리에 Release를 다시 내려받아 checksum과
`tools/verify_rollout_bundle.py`를 통과시킨다. 이 검증은 Eddy 안에서 하며 다른 PC로
checkpoint를 옮기지 않는다.

## 완료 보고 형식

```text
model_id:
Git branch/commit:
Release tag/URL:
asset filename/size/SHA-256:
policy/action representation:
dataset revision/episodes:
selected step/reason:
LeRobot/PyTorch/CUDA:
chunk_size/n_action_steps/ensemble:
remote re-download verification:
remaining uncertainty:
```

Release URL과 metadata commit이 준비되면 여기서 GitHub만 참조해 전용 rollout
adapter를 구현할 수 있다.
