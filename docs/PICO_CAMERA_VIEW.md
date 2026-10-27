# PICO 카메라 투영 및 재현

PiPER 손목(local)과 고정(global) InnoMaker 영상을 PICO에서 보면서 같은 앱의
XRoboToolkit tracking으로 teleoperation한다. 별도 viewer 앱으로 전환하면 tracking
앱이 background로 이동할 수 있어 Unity client 안에 영상 표시를 통합했다.

## 데이터 흐름

```text
physical local → ffmpeg scale 640:480 → /dev/video10 → teleop + PICO sender
physical global → scale 640:360 + pad → /dev/video11 → teleop + PICO sender
sender → H.264/TCP 12345 → PICO Listen/decoder/좌우 eye canvas
PICO 오른쪽 stick click → UDP toggle:12346 → sender local/global 전환
PICO tracking → PC RoboticsService → localhost:60061 → teleop
```

물리 장치는 fanout 하나가 점유한다. teleop와 sender는 공유 video10/11을 읽는다.
두 물리 카메라는 동일 serial을 제공하므로 USB by-path로 구분한다.

## 앱과 화면

- 앱: `XRoboToolkit Local View`
- package: `com.swaaniida.xrobotoolkit.localview`
- 빌드 소스 버전: `1.1.1-local.7`, version code `10007`
- 기존 `com.xrobotoolkit.client`와 별도로 설치된다.
- 앱에서 `LOCAL_INNOMAKER` source를 선택하고 Listen을 시작한다.
- 각 눈 960×540, 전체 1920×540 side-by-side H.264로 같은 mono 영상을 보낸다.
- 실제 stereo/360 panorama가 아닌 카메라 평면 영상의 전체 시야 표시다.
- passthrough 기능 자체를 끄는 대신 불투명 eye canvas가 배경을 가린다.

## 실행

기존 `./setup`은 fanout, sender, XR service와 Tracer ROS까지 시작한다.
Wi-Fi/IP/USB by-path는 기존 PC에 고정돼 있으므로 새 PC에서는 실제 설정을 확인한다.
카메라 표시만 준비할 때 로봇 수집 `./start`는 필요 없다.

fanout 준비 후 sender만 실행:

```bash
~/miniconda3/envs/piper-xr/bin/python tools/pico_h264_sender.py \
  --pico <PICO_IP> --device /dev/video10 --alternate-device /dev/video11 \
  --input-format yuyv422 --width 640 --height 480 \
  --fps 30 --bitrate 4M --toggle-port 12346
```

PC→PICO TCP 12345와 PICO→PC UDP 12346이 연결돼야 한다. 기존 주소는 PC
`192.168.0.83`, PICO `192.168.0.229`였으며 새 네트워크에 그대로 적용하지 않는다.

오른쪽 joystick click으로 local/global을 전환한다. 앱의
`toggle-objects-script.cs`에는 toggle 목적 PC IP가 고정돼 있어 네트워크가 바뀌면
수정 후 빌드해야 한다. teleop도 같은 클릭을 받아 sender에 SIGUSR1을 보낸다.
두 경로가 같은 클릭에 함께 반응하면 중복 전환 가능성이 있어 로그 확인이 필요하다.
B는 teleoperation 안전 종료에 사용한다.

## 전송과 원본 저장

`tools/pico_h264_sender.py`는 ffmpeg libx264로 인코딩하고 Annex-B NAL을 access unit으로
묶어 `4-byte big-endian length + access-unit bytes`로 TCP 전송한다.
global 영상에는 sender가 반원 aid를 합성한다. 공유 장치와 episode 원본 이미지에는
aid가 들어가지 않는다.

## 깜빡임 대응 코드

- decoder texture를 Unity-owned RenderTexture로 복사해 표시 texture를 유지
- fullscreen shader alpha를 1로 고정
- 중복 RawImage 숨김, eye canvas plane distance를 0.75 m로 이동
- eye texture resolution scale을 1.5에서 1.0으로 낮춤

이는 현재 소스에 반영된 대응이며 이 문서 정리 시 기기에서 재검증하지 않았다.
sender GOP 기본값은 1초다. 코드의 긴 GOP 주석과 다르므로 필요하면
`--keyframe-seconds`로 명시한다.

## Unity 앱 재현

기준 저장소: https://github.com/XR-Robotics/XRoboToolkit-Unity-Client

기준 commit: `cdc53166b0bf412efae71046c6a225eb5091605f`

`deployment/pico-local-view/`에 기준 대비 변경 패치와 추가 builder를 보존했다.
Unity Library, SDK 중복본, APK, keystore는 포함하지 않는다.

```bash
git clone https://github.com/XR-Robotics/XRoboToolkit-Unity-Client.git ~/XRoboToolkit-LocalView
git -C ~/XRoboToolkit-LocalView checkout cdc53166b0bf412efae71046c6a225eb5091605f
git -C ~/XRoboToolkit-LocalView apply /path/to/piper_tracer/deployment/pico-local-view/unity-local-view.patch
cp -a /path/to/piper_tracer/deployment/pico-local-view/Assets/. ~/XRoboToolkit-LocalView/Assets/
```

Unity **2022.3.16f1**와 Android Build Support를 설치하고 빌드한다.

```bash
<UNITY_EDITOR> -batchmode -nographics -quit \
  -projectPath ~/XRoboToolkit-LocalView \
  -executeMethod LocalViewBuilder.BuildAndroid -logFile /tmp/pico-localview-build.log
adb install -r ~/XRoboToolkit-LocalView/Builds/Android/XRoboToolkit-Local-View.apk
```

builder는 OpenGLES3, development build와 기본 debug signing을 사용한다.
기존 설치본과 signing key가 다르면 업데이트가 거절될 수 있다. 기존 앱의 설정을
보존하려면 기존 서명키를 사용해야 한다.

## 핵심 파일

- `tools/pico_h264_sender.py`: 영상 송신, aid, 카메라 선택
- `setup`: 공유 카메라와 서비스 시작
- `single_piper_teleop/teleop.py`: tracking과 joystick click SIGUSR1
- `deployment/pico-local-view/unity-local-view.patch`: 앱 표시·decoder·shader 변경
- `deployment/pico-local-view/Assets/Editor/Build/LocalViewBuilder.cs`: 별도 앱 빌드

`tools/pico_local_view.py`는 이전 browser 방식이다. 별도 native viewer와 RobotVision-PC
checkout은 현재 Unity 통합 방식 실행에 필요하지 않아 이 배포에서 제외했다.
