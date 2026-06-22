# TurtleBot3 Vision Line Follower

본 저장소는 **ROS 2 Humble 기반 TurtleBot3 비전 라인 추종 프로젝트**를 정리한 저장소입니다.

카메라 영상에서 주행 라인을 검출한 뒤, 라인의 중심 위치와 유효성을 기반으로 TurtleBot3의 `/cmd_vel` 속도 명령을 생성하여 라인을 따라 주행하도록 구성하였습니다.

본 프로젝트는 OpenCV 기반 영상 처리와 ROS 2 노드 통신 구조를 활용하여, 카메라 기반 자율주행의 기본 흐름을 실습하고 검증하는 것을 목표로 합니다.

> Tested on: ROS 2 Humble / Ubuntu 22.04

---

## Demo Video

아래 썸네일을 클릭하면 TurtleBot3 라인 추종 데모 영상을 확인할 수 있습니다.

<a href="https://youtube.com/shorts/grw7qH93KJ0">
  <img src="https://img.youtube.com/vi/grw7qH93KJ0/hqdefault.jpg" width="900" alt="TurtleBot3 Vision Line Follower Demo">
</a>

---

## 프로젝트 개요

본 프로젝트는 TurtleBot3에 장착된 카메라 영상을 입력으로 받아 라인을 검출하고, 검출된 라인의 위치를 기준으로 로봇의 선속도와 각속도를 제어하는 구조입니다.

카메라 영상은 OpenCV를 이용해 처리하며, bird’s-eye view 변환과 contour filtering을 통해 주행 라인 후보를 추출합니다. 이후 라인의 중심이 이미지 중앙에서 얼마나 벗어났는지 계산하고, 이를 기반으로 TurtleBot3가 라인을 따라가도록 `/cmd_vel` 명령을 생성합니다.

전체 동작 흐름은 다음과 같습니다.

```text
Camera Image
    ↓
OpenCV-based Line Detection
    ↓
Line Center Error Calculation
    ↓
Twist Command Generation
    ↓
TurtleBot3 Line Following
```

---

## 시스템 구조

본 패키지는 크게 두 개의 ROS 2 노드로 구성됩니다.

### 1. `camera_viewer`

`camera_viewer` 노드는 카메라 영상을 구독하고, 영상 처리 과정을 통해 라인을 검출합니다.

주요 기능은 다음과 같습니다.

* `/camera/image_raw` 토픽 구독
* OpenCV 기반 영상 처리
* Bird’s-eye view 변환
* 어두운 라인 영역 검출
* contour filtering 기반 라인 후보 추출
* 라인 중심 오차 계산
* 라인 검출 결과 publish
* 디버그 이미지 publish

---

### 2. `line_follower`

`line_follower` 노드는 `camera_viewer`에서 publish한 라인 검출 결과를 구독하고, TurtleBot3가 라인을 따라가도록 `/cmd_vel` 명령을 생성합니다.

주요 기능은 다음과 같습니다.

* `/camera/line_detection` 토픽 구독
* 라인 중심 오차 기반 조향 제어
* 라인 검출 실패 시 정지 또는 감속 처리
* `geometry_msgs/Twist` 형태의 `/cmd_vel` publish

---

## Node Graph

전체 노드 및 토픽 흐름은 다음과 같습니다.

```text
/camera/image_raw
        ↓
  camera_viewer
        ↓
/camera/line_detection
        ↓
  line_follower
        ↓
     /cmd_vel
```

디버깅용 영상 토픽은 다음과 같이 별도로 publish됩니다.

```text
camera_viewer
    └── /line_follower/debug_image
```

필요한 경우 `rqt_image_view`를 이용하여 디버그 영상을 확인할 수 있습니다.

---

## 주요 토픽

### Subscribed Topics

| Topic               | Type                | Description          |
| ------------------- | ------------------- | -------------------- |
| `/camera/image_raw` | `sensor_msgs/Image` | TurtleBot3 카메라 원본 영상 |

---

### Published Topics

| Topic                        | Type                    | Description           |
| ---------------------------- | ----------------------- | --------------------- |
| `/camera/line_detection`     | `geometry_msgs/Vector3` | 라인 검출 결과              |
| `/line_follower/debug_image` | `sensor_msgs/Image`     | 영상 처리 결과 확인용 디버그 이미지  |
| `/cmd_vel`                   | `geometry_msgs/Twist`   | TurtleBot3 주행 제어 명령   |
| `/image_processed`           | `sensor_msgs/Image`     | 처리된 영상                |
| `/camera/image_bev`          | `sensor_msgs/Image`     | Bird’s-eye view 변환 영상 |

---

## `/camera/line_detection` 메시지 구조

`/camera/line_detection` 토픽은 `geometry_msgs/Vector3` 타입을 사용합니다.

각 필드의 의미는 다음과 같습니다.

```text
x: horizontal error
y: line coverage ratio
z: validity
```

### Field Description

| Field | Meaning                                           |
| ----- | ------------------------------------------------- |
| `x`   | 이미지 중심 기준 라인 중심의 가로 방향 오차                         |
| `y`   | ROI 영역에서 라인이 차지하는 비율                              |
| `z`   | 라인 검출 유효성, `1.0 = detected`, `0.0 = not detected` |

즉, `x` 값은 로봇이 왼쪽 또는 오른쪽으로 얼마나 조향해야 하는지 판단하는 데 사용됩니다.
`z` 값이 `0.0`이면 라인이 검출되지 않은 상태이므로, 로봇은 정지하거나 감속하도록 처리할 수 있습니다.

---

## 제어 방식

본 프로젝트의 기본 제어 아이디어는 단순합니다.

```text
line_error = line_center - image_center
angular_z  = -Kp * line_error
linear_x   = constant forward speed
```

라인이 이미지 중앙보다 왼쪽에 있으면 로봇은 왼쪽으로 회전하고,
라인이 이미지 중앙보다 오른쪽에 있으면 로봇은 오른쪽으로 회전합니다.

라인이 안정적으로 검출되는 동안에는 일정한 전진 속도를 유지하고,
라인이 검출되지 않는 경우에는 안전을 위해 정지하거나 속도를 낮추는 방식으로 동작합니다.

---

## Quick Start

### 1. Workspace 생성

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

### 2. 저장소 clone

```bash
git clone https://github.com/YJS906/turtlebot3_line_follower.git
```

---

### 3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 4. TurtleBot3 환경 변수 설정

사용하는 TurtleBot3 모델에 맞게 환경 변수를 설정합니다.

예를 들어 TurtleBot3 Burger를 사용하는 경우:

```bash
export TURTLEBOT3_MODEL=burger
```

Waffle Pi를 사용하는 경우:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

매번 자동으로 적용하고 싶다면 `~/.bashrc`에 추가할 수 있습니다.

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## 실행 방법

### 1. 카메라 노드 실행

```bash
ros2 run turtlebot3_line_follower camera_viewer
```

---

### 2. 라인 추종 노드 실행

새 터미널을 열고 다음 명령을 실행합니다.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlebot3_line_follower line_follower
```

---

### 3. 디버그 이미지 확인

디버그 영상을 확인하려면 다음 명령을 사용할 수 있습니다.

```bash
rqt_image_view
```

이후 다음 토픽 중 하나를 선택하여 영상 처리 결과를 확인합니다.

```text
/line_follower/debug_image
/image_processed
/camera/image_bev
```

---

## 실행 전 확인 사항

라인 추종을 실행하기 전에 다음 항목을 확인해야 합니다.

### 1. 카메라 토픽 확인

```bash
ros2 topic list
```

카메라 토픽이 다음 이름으로 publish되는지 확인합니다.

```text
/camera/image_raw
```

토픽 이름이 다를 경우, 코드 또는 launch 설정에서 카메라 토픽 이름을 실제 환경에 맞게 수정해야 합니다.

---

### 2. `/cmd_vel` 토픽 확인

TurtleBot3는 일반적으로 `/cmd_vel` 토픽을 통해 속도 명령을 받습니다.

```bash
ros2 topic echo /cmd_vel
```

라인 추종 노드를 실행했을 때 `geometry_msgs/Twist` 메시지가 출력되면 정상적으로 제어 명령이 생성되고 있는 상태입니다.

---

### 3. 라인 검출 결과 확인

```bash
ros2 topic echo /camera/line_detection
```

라인이 검출되면 `z` 값이 `1.0`에 가깝게 출력됩니다.

예시:

```text
x: -0.12
y: 0.35
z: 1.0
```

라인이 검출되지 않으면 `z` 값이 `0.0`으로 출력될 수 있습니다.

---

## 영상 처리 흐름

`camera_viewer` 노드의 영상 처리 흐름은 다음과 같습니다.

```text
1. 카메라 이미지 수신
2. OpenCV 이미지로 변환
3. 관심 영역 ROI 설정
4. Bird’s-eye view 변환
5. 색상 또는 밝기 기반 라인 후보 추출
6. contour filtering 수행
7. 라인 중심 좌표 계산
8. 이미지 중심과 라인 중심의 오차 계산
9. /camera/line_detection publish
10. debug image publish
```

---

## 튜닝 포인트

라인 추종 성능은 카메라 위치, 조명, 라인 색상, 바닥 재질, 주행 속도에 따라 달라질 수 있습니다.

주요 튜닝 포인트는 다음과 같습니다.

| 항목                 | 설명                          |
| ------------------ | --------------------------- |
| ROI 영역             | 영상에서 라인을 찾을 영역              |
| Threshold 값        | 어두운 라인 또는 특정 색상 라인을 분리하는 기준 |
| Contour area       | 너무 작은 잡음을 제거하기 위한 최소 면적     |
| Linear speed       | 로봇의 전진 속도                   |
| Angular gain       | 라인 중심 오차에 대한 회전 민감도         |
| Lost-line behavior | 라인 검출 실패 시 정지 또는 감속 방식      |

---

## 디버깅 방법

### 카메라 영상이 들어오지 않는 경우

```bash
ros2 topic list
ros2 topic echo /camera/image_raw
```

카메라 토픽 이름이 `/camera/image_raw`가 아닐 수 있으므로 실제 토픽 이름을 먼저 확인해야 합니다.

---

### 로봇이 움직이지 않는 경우

```bash
ros2 topic echo /cmd_vel
```

`/cmd_vel` 메시지가 출력되는데 로봇이 움직이지 않는다면 TurtleBot3 bringup 상태를 확인해야 합니다.

```bash
ros2 topic list
```

TurtleBot3 관련 bringup 노드가 정상적으로 실행 중인지 확인합니다.

---

### 라인이 검출되지 않는 경우

`rqt_image_view`를 실행하여 디버그 이미지를 확인합니다.

```bash
rqt_image_view
```

확인할 토픽:

```text
/line_follower/debug_image
/image_processed
/camera/image_bev
```

라인이 너무 어둡거나 밝게 보이지 않는 경우 threshold 값을 조정해야 합니다.
조명 변화가 큰 환경에서는 HSV 기반 색상 필터 또는 adaptive threshold 적용을 고려할 수 있습니다.

---

## Repository Structure

일반적인 ROS 2 Python 패키지 구조는 다음과 같습니다.

```text
turtlebot3_line_follower/
├── README.md
├── package.xml
├── setup.py
├── resource/
│   └── turtlebot3_line_follower
├── turtlebot3_line_follower/
│   ├── __init__.py
│   ├── camera_viewer.py
│   └── line_follower.py
└── launch/
    └── line_follower.launch.py
```

주요 파일의 역할은 다음과 같습니다.

| Path                             | Description                        |
| -------------------------------- | ---------------------------------- |
| `camera_viewer.py`               | 카메라 영상 처리 및 라인 검출 노드               |
| `line_follower.py`               | 라인 검출 결과 기반 TurtleBot3 주행 제어 노드    |
| `launch/line_follower.launch.py` | 카메라 처리 및 라인 추종 노드 실행용 launch 파일    |
| `package.xml`                    | ROS 2 패키지 의존성 정보                   |
| `setup.py`                       | Python package 및 console script 설정 |

---

## Dependencies

본 프로젝트는 다음 패키지 및 라이브러리를 사용합니다.

```text
ROS 2 Humble
TurtleBot3 packages
OpenCV
cv_bridge
sensor_msgs
geometry_msgs
rclpy
```

필요한 ROS 2 패키지는 다음과 같이 설치할 수 있습니다.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-bringup \
  ros-humble-rqt-image-view
```

OpenCV가 설치되어 있지 않다면 다음 명령으로 설치할 수 있습니다.

```bash
sudo apt install -y python3-opencv
```

---

## Notes

본 저장소는 TurtleBot3의 카메라 기반 라인 추종을 구현하기 위한 실습 및 프로젝트용 코드입니다.

복잡한 주행 환경을 위한 고수준 자율주행 알고리즘보다는,
카메라 영상 처리 → 라인 검출 → 제어 명령 생성으로 이어지는 기본적인 비전 기반 자율주행 파이프라인을 이해하는 데 초점을 두고 있습니다.

실제 주행 환경에서는 다음 요소에 따라 성능이 달라질 수 있습니다.

* 조명 변화
* 라인 색상과 바닥 색상의 대비
* 카메라 장착 각도
* TurtleBot3 주행 속도
* 바닥 반사
* 라인의 두께와 곡률

따라서 실제 환경에서 안정적으로 동작시키기 위해서는 threshold, ROI, contour filtering, angular gain 값을 환경에 맞게 조정하는 과정이 필요합니다.

---

## Project Purpose

이 프로젝트를 통해 다음 내용을 학습하고 검증할 수 있습니다.

* ROS 2 Python 노드 작성
* TurtleBot3 `/cmd_vel` 제어
* OpenCV 기반 카메라 영상 처리
* Bird’s-eye view 변환
* Contour 기반 라인 검출
* Topic 기반 노드 간 데이터 흐름
* 비전 기반 모바일 로봇 제어 구조

---

## License

본 저장소의 라이선스는 별도로 명시되어 있지 않습니다.
필요한 경우 프로젝트 목적에 맞게 `LICENSE` 파일을 추가할 수 있습니다.
