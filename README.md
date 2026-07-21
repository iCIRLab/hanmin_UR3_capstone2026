# hanmin_UR3_capstone2026

# UR3 Control

ROS 2 Humble과 공식 `ur_robot_driver`를 사용하여 실제 Universal Robots
UR3를 정해진 관절 자세로 움직이는 키보드 제어 패키지입니다.

- `h`: Home 자세
- `s`: S/scan 자세
- `f`: 실행 중인 궤적 취소
- `q` 또는 `Esc`: 프로그램 종료

MoveIt은 사용하지 않으며, 목표 관절각을
`scaled_joint_trajectory_controller`의 `FollowJointTrajectory` 액션으로
직접 전달합니다.

## 환경

| 항목 | 구성 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Robot | Universal Robots UR3 CB3 |
| Robot IP | `192.168.56.1` |
| ROS PC IP | `192.168.56.2` |
| Driver | `ros-humble-ur-robot-driver` 2.13.2 |

## 패키지 구조

```text
src/ur3_control/
├── launch/ur3_driver.launch.py
├── ur3_control/ur3_pose_control.py
├── config/ur3_calibration.yaml
├── package.xml
├── setup.py
├── setup.cfg
└── resource/ur3_control
```

- [`ur3_driver.launch.py`](src/ur3_control/launch/ur3_driver.launch.py): 공식
  `ur_control.launch.py`를 UR3 네트워크·보정값으로 실행합니다.
- [`ur3_pose_control.py`](src/ur3_control/ur3_control/ur3_pose_control.py):
  키 입력, 목표 자세 생성, 액션 전송 및 취소를 담당합니다.
- [`ur3_calibration.yaml`](src/ur3_control/config/ur3_calibration.yaml): 실험에
  사용한 UR3에서 추출한 기구학 보정값입니다.

## 제어 흐름

```text
키보드 h/s
→ 목표 관절각과 도착시간 생성
→ FollowJointTrajectory 액션 전송
→ scaled_joint_trajectory_controller의 spline 보간
→ URPositionHardwareInterface
→ Reverse Interface
→ URScript servoj()
→ 실제 UR3 관절 구동
```

## 목표 자세

관절 순서는 `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`,
`wrist_2`, `wrist_3`입니다.

| 키 | 자세 | 목표 관절각 (deg) |
|---|---|---|
| `h` | Home | `[0, -90, 0, -90, 0, 0]` |
| `s` | S/scan | `[0, -90, 90, -135, -90, 0]` |

## 동작 영상

- [Home 자세 동작 영상]


https://github.com/user-attachments/assets/1c371d5e-03e2-4ec6-8125-aa4cbd60cf48



- [S/scan 자세 동작 영상]

https://github.com/user-attachments/assets/9da0c4cd-142f-4727-a1fc-7b31ce2118e9

## 설치 및 빌드

```bash
sudo apt update
sudo apt install ros-humble-ur-robot-driver

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

터미널 1에서 UR3 드라이버를 실행합니다.

```bash
ros2 launch ur3_control ur3_driver.launch.py \
  robot_ip:=192.168.56.1 \
  reverse_ip:=192.168.56.2 \
  headless_mode:=true \
  launch_rviz:=false
```

터미널 2에서 키보드 제어 노드를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ur3_control ur3_pose_control
```

UR 속도는 티치펜던트의 속도 슬라이더 또는
`/io_and_status_controller/set_speed_slider` 서비스로 제한할 수 있습니다.
실물 시험에서는 30% 속도를 사용했습니다.

> `f` 키는 ROS 궤적 취소 기능이며 안전 비상정지를 대신하지 않습니다.
> 로봇별 calibration 파일을 확인하고, 작업 공간을 비운 상태에서 실행해야 합니다.
