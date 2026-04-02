# isaac_ros_visual_slam_orb

ORB-SLAM3 + CUDA backend for Isaac ROS Visual SLAM.
Isaac ROS Visual SLAM (cuVSLAM)과 동일한 ROS2 토픽/서비스/TF 인터페이스를 제공하면서, ORB-SLAM3를 SLAM 엔진으로 사용합니다.

## Supported Platforms

| Platform | Camera | GPU |
|----------|--------|-----|
| Jetson AGX Orin (JetPack 6, CUDA 12.x) | Intel RealSense D456 | Tegra (Ampere) |

## Sensor Modes

| Mode | Description |
|------|-------------|
| `mono` | Monocular (left IR only) |
| `mono-imu` | Monocular + IMU |
| `stereo` | Stereo (left + right IR) |
| `stereo-imu` | Stereo-Inertial (left + right IR + IMU) |
| `rgbd` | RGB-D (color + depth) |
| `rgbd-imu` | RGB-D-Inertial (color + depth + IMU) |

## Quick Start (Makefile)

Deploy 이미지가 이미 빌드되어 있다면, 프로젝트 루트에서 `make`로 바로 실행할 수 있습니다:

```bash
cd src/isaac_ros_visual_slam_orb

make              # stereo 모드 (기본)
make stereo-imu   # stereo + IMU
make rgbd         # RGB-D 모드
make mono         # monocular 모드
make shell        # 디버그용 interactive shell
make build        # deploy 이미지 재빌드
make down         # 실행 중인 컨테이너 중지/제거
make help         # 전체 타겟 목록
```

---

## Tutorial 1: Development Container에서 실행

### 1.1 Dev 컨테이너 진입

```bash
cd /mnt/nova_ssd/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh
```

### 1.2 ORB-SLAM3 빌드 (최초 1회)

컨테이너 내부에서 실행:

```bash
# ORB-SLAM3 + Pangolin + DBoW2 + g2o 빌드 및 설치
bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh
```

CUDA OpenCV가 필요한 경우 (GPU 가속 ORB 추출, 빌드 약 40분):

```bash
bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh --with-cuda-opencv
```

### 1.3 ROS 패키지 빌드

```bash
colcon build --packages-up-to isaac_ros_visual_slam_orb
source install/setup.bash
```

### 1.4 실행

RealSense D456을 USB3 포트에 연결한 상태에서:

```bash
# Stereo 모드 (기본)
ros2 launch isaac_ros_visual_slam_orb \
    isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo

# Stereo-IMU 모드
ros2 launch isaac_ros_visual_slam_orb \
    isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo-imu

# RGB-D 모드
ros2 launch isaac_ros_visual_slam_orb \
    isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=rgbd

# Monocular 모드
ros2 launch isaac_ros_visual_slam_orb \
    isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=mono
```

Pangolin 뷰어 윈도우와 OpenCV 윈도우가 X11을 통해 표시됩니다.

Pangolin 뷰어를 끄고 RViz만 사용하려면:

```bash
ros2 launch isaac_ros_visual_slam_orb \
    isaac_ros_visual_slam_orb_realsense_d456.launch.py mode:=stereo \
    enable_viewer:=false run_rviz:=true
```

### 1.5 토픽 확인

별도 터미널에서:

```bash
# 카메라 토픽
ros2 topic list | grep camera

# SLAM 출력 토픽
ros2 topic echo /visual_slam/tracking/odometry     # Odometry
ros2 topic echo /visual_slam/status                 # SLAM 상태
ros2 topic echo /visual_slam/vis/slam_path          # 경로 시각화
```

---

## Tutorial 2: Deploy 이미지로 실행

독립적인 deploy Docker 이미지를 빌드하여 dev 환경 없이 실행하는 방법입니다.

### 2.1 사전 준비: Dev 컨테이너에서 빌드

먼저 Tutorial 1의 1.1~1.3 단계를 완료하여 ROS 패키지를 빌드합니다.

```bash
# Dev 컨테이너 진입
./src/isaac_ros_common/scripts/run_dev.sh

# (컨테이너 내부)
bash src/isaac_ros_visual_slam_orb/scripts/install_orb_slam3.sh
colcon build --packages-up-to isaac_ros_visual_slam_orb
exit
```

### 2.2 Deploy 이미지 빌드

호스트에서 실행:

```bash
# 기본 빌드 (ORB-SLAM3 라이브러리 tarball이 이미 있는 경우)
bash src/isaac_ros_visual_slam_orb/scripts/build_deploy.sh

# ORB-SLAM3 라이브러리 추출 + 이미지 빌드 (한번에)
bash src/isaac_ros_visual_slam_orb/scripts/build_deploy.sh --build-libs

# 강제 재빌드
bash src/isaac_ros_visual_slam_orb/scripts/build_deploy.sh --build-libs --force
```

### 2.3 Deploy 컨테이너 실행

```bash
# Stereo 모드 (기본)
bash src/isaac_ros_visual_slam_orb/scripts/run_deploy_orb.sh

# 센서 모드 지정
bash src/isaac_ros_visual_slam_orb/scripts/run_deploy_orb.sh --mode stereo-imu
bash src/isaac_ros_visual_slam_orb/scripts/run_deploy_orb.sh --mode rgbd
bash src/isaac_ros_visual_slam_orb/scripts/run_deploy_orb.sh --mode mono

# Interactive shell (디버깅용)
bash src/isaac_ros_visual_slam_orb/scripts/run_deploy_orb.sh -s

# 추가 launch 인자 전달
bash src/isaac_ros_visual_slam_orb/scripts/run_deploy_orb.sh --mode stereo enable_imu_fusion:=false
```

### 2.4 GUI 확인

컨테이너 실행 시 자동으로:
- **Pangolin 뷰어**: 3D 맵 포인트, 카메라 궤적, 키프레임 시각화
- **OpenCV 윈도우**: ORB 특징점 추출 결과 표시

X11 포워딩이 자동 설정되므로 호스트 디스플레이에 윈도우가 표시됩니다.

> **참고**: SSH 원격 접속 시 `ssh -X` 또는 `ssh -Y`로 X11 포워딩을 활성화해야 합니다.

---

## Deploy 스크립트 상세

### extract_orb_slam3_libs.sh

Dev 컨테이너에서 빌드된 ORB-SLAM3/Pangolin 라이브러리를 호스트로 추출합니다.

```bash
# 라이브러리가 이미 빌드된 컨테이너에서 추출
bash scripts/extract_orb_slam3_libs.sh

# 빌드 + 추출 (한번에)
bash scripts/extract_orb_slam3_libs.sh --build

# 강제 재추출
bash scripts/extract_orb_slam3_libs.sh --force
```

추출되는 파일:
- `/opt/orb_slam3/lib/` — libORB_SLAM3.so, libDBoW2.so, libg2o.so
- `/opt/orb_slam3/Vocabulary/` — ORBvoc.txt
- `/usr/local/lib/libpango*.so*` — Pangolin 라이브러리

### build_deploy.sh

`docker_deploy.sh`를 호출하여 deploy Docker 이미지를 빌드합니다.

```bash
bash scripts/build_deploy.sh [--name IMAGE_NAME] [--build-libs] [--force]
```

### run_deploy_orb.sh

Deploy 컨테이너를 실행합니다. X11/OpenGL, RealSense USB, Jetson GPU가 자동 설정됩니다.

```bash
bash scripts/run_deploy_orb.sh [-s|--shell] [-m|--mode MODE] [launch_args...]
```

---

## Launch Parameters

주요 launch 파라미터 목록:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `stereo` | 센서 모드 (`mono`, `mono-imu`, `stereo`, `stereo-imu`, `rgbd`, `rgbd-imu`) |
| `enable_viewer` | `true` | Pangolin 3D 맵 뷰어 |
| `run_rviz` | `false` | RViz2 실행 여부 |
| `enable_slam_visualization` | `true` | SLAM 시각화 출력 |
| `enable_observations_view` | `true` | 현재 프레임 특징점 표시 |
| `enable_landmarks_view` | `true` | 맵 포인트 표시 |
| `gaussian_blur` | `false` | 진동 노이즈 감소용 블러 |
| `image_jitter_threshold_ms` | `40.0` | 스테레오 동기화 허용 오차 (ms) |
| `gyro_noise_density` | `0.001` | 자이로 노이즈 밀도 (IMU 모드) |
| `accel_noise_density` | `0.01` | 가속도계 노이즈 밀도 (IMU 모드) |
| `calibration_frequency` | `200.0` | IMU 캘리브레이션 주파수 (Hz) |

---

## 디렉토리 구조

```text
isaac_ros_visual_slam_orb/
├── CMakeLists.txt
├── package.xml
├── Makefile
├── README.md
├── include/isaac_ros_visual_slam_orb/
│   ├── visual_slam_node.hpp
│   ├── slam_backend.hpp
│   ├── orb_slam3_backend.hpp
│   ├── slam_types.hpp
│   └── impl/
│       ├── cuda_image_pipeline.hpp
│       ├── cuda_orb_extractor.hpp
│       └── ros_orb_conversion.hpp
├── src/
│   ├── visual_slam_node.cpp
│   ├── visual_slam_main.cpp
│   ├── orb_slam3_backend.cpp
│   └── impl/
│       ├── cuda_image_pipeline.cpp
│       ├── cuda_orb_extractor.cpp
│       └── ros_orb_conversion.cpp
├── launch/
│   ├── isaac_ros_visual_slam_orb_realsense.launch.py
│   └── isaac_ros_visual_slam_orb_realsense_d456.launch.py
├── config/
│   └── realsense_d456.yaml
├── rviz/
│   └── realsense_d456.rviz
└── scripts/
    ├── install_orb_slam3.sh
    ├── extract_orb_slam3_libs.sh
    ├── build_deploy.sh
    ├── run_deploy_orb.sh
    └── deploy_entrypoint.sh
```

## Troubleshooting

### Pangolin 뷰어가 표시되지 않음

```bash
# 호스트에서 X11 접근 허용
xhost +local:root

# DISPLAY 환경변수 확인
echo $DISPLAY
```

### RealSense D456 인식 안 됨

```bash
# USB 장치 확인
lsusb | grep -i intel

# RealSense 노드 토픽 확인
ros2 topic list | grep camera
```

### ORB-SLAM3 라이브러리 로드 실패

```bash
# 컨테이너 내부에서 확인
ldconfig -p | grep -i orb
ls -la /opt/orb_slam3/lib/
echo $LD_LIBRARY_PATH
```
