# hanmin_UR3_capstone2026

**MPlib과 MoveIt 2를 하나의 계획–실행 경로 위에서 비교하는 UR3 매니퓰레이션 스택 (ROS 2 Humble).**

[English README](README.md)

## Demo

### Real UR3 — Home 자세

<!-- V1: 기존 리포의 Home 자세 영상 URL을 이 줄에 단독으로 붙입니다 -->

### Real UR3 — S/scan 자세

<!-- V2: 기존 리포의 S/scan 자세 영상 URL -->

### 목표 시나리오

<!-- V3: 기존 리포의 시나리오 영상 URL -->

### MuJoCo — MPlib 계획·실행

<!-- V4: MuJoCo에서 MPlib으로 Home→Scan 계획 후 실행하는 영상 URL -->

### MuJoCo — MoveIt 2 계획·실행

<!-- V5: 같은 목표를 MoveIt 2로 실행한 영상 URL -->

---

## 기준 자료

- [haosulab/MPlib](https://github.com/haosulab/MPlib)
- [MPlib Getting Started](https://motion-planning-lib.readthedocs.io/latest/tutorials/getting_started.html)

---

## 시스템 개요

<!-- IMG1: assets/mujoco_scene.png — 작업대에 장착된 UR3의 MuJoCo 장면 전체 -->

```text
┌──────────────────────────────────────────────────────────┐
│                 hanmin_UR3_capstone2026                  │
│                                                          │
│   named_goal / joint qpos / TCP pose                     │
│          ↓                                               │
│   ┌────────────────┐        ┌────────────────────────┐   │
│   │ MPlib 0.2.1    │        │ MoveIt 2 (OMPL)        │   │
│   │ in-process     │   또는  │ /plan_kinematic_path   │   │
│   │ FCL world      │        │ PlanningScene          │   │
│   └───────┬────────┘        └───────────┬────────────┘   │
│           └───────────┬─────────────────┘                │
│                       ↓                                  │
│        /capstone_planning/planned_trajectory             │
│                       ↓                                  │
│        궤적 안전 검증기 (관절 순서·한계·시작오차·단조시간)     │
│                       ↓                                  │
│        FollowJointTrajectory  (단일 실행기)                │
│                       ↓                                  │
│   ┌────────────────┐        ┌────────────────────────┐   │
│   │ MuJoCo JTC     │   또는  │ 실제 UR3 scaled JTC     │   │
│   └────────────────┘        └────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

### 주요 기능

- **플래너 동등 비교**: 두 플래너가 같은 모델·한계·검증기·실행기를 공유하므로 계획 품질 차이만 남습니다
- **이중 실행 게이트**: `plan_only`와 `execute`를 함께 바꿔야만 컨트롤러로 궤적이 전송됩니다
- **보정 모델 단일화**: 실장비에서 추출한 `ur3_calibration.yaml`이 URDF와 MJCF 양쪽에 반영됩니다
- **단일 실행 경로**: MoveIt의 실행 액션을 차단해 실제 전송은 `trajectory_executor` 한 곳에서만 일어납니다
- **MuJoCo 백엔드**: 공식 `mujoco_ros2_control`의 `SystemInterface`를 사용한 전용 UR3 장면

---

## 하드웨어

| 구성 | 모델 | 상태 |
|---|---|---|
| 로봇 팔 | Universal Robots UR3 (CB3) | `ur_robot_driver` 2.13.2, `scaled_joint_trajectory_controller` |
| 카메라 | Intel RealSense D435i | 스트림 확인까지 구현, 좌표 발행 미구현 |
| LiDAR | 미정 | 추후 검토 |

---

## 패키지 구조

```text
src/
├── capstone_bringup/            # 통합 런치 (MuJoCo·실장비)
│   └── launch/
│       ├── ur3_arm.launch.py         # 백엔드 + 플래너 + 실행기 통합 진입점
│       ├── ur3_driver.launch.py      # 실제 UR3 드라이버
│       └── d435i_camera_view.launch.py
├── capstone_planning/           # 공통 모델·한계, 두 플래너
│   ├── capstone_planning/
│   │   ├── model.py                  # 관절 순서, named goal, 한계 로딩
│   │   ├── mplib_planner.py          # MPlib 래퍼 (FCL 충돌 월드)
│   │   ├── mplib_plan_node.py        # MPlib 계획 노드
│   │   ├── moveit_plan_node.py       # MoveIt 계획 서비스 클라이언트
│   │   ├── workcell.py               # 두 플래너 공통 작업대 충돌 형상
│   │   └── trajectory.py             # 궤적 생성·검증 유틸
│   ├── config/ur3_planning_limits.yaml
│   └── description/                  # ur3_calibrated.urdf, xacro
├── capstone_manipulation/       # 궤적 검증·실행
│   └── capstone_manipulation/
│       ├── trajectory_executor.py    # 유일한 FollowJointTrajectory 전송 지점
│       └── ur3_pose_control.py       # 실장비 검증된 키보드 Home/S 제어 (보존)
├── capstone_mujoco/             # MuJoCo ros2_control 백엔드
│   ├── description/                  # ur3_model.xml, ur3_scene.xml, assets
│   └── config/controllers.yaml
├── capstone_moveit_support/     # MoveIt의 별도 실행 경로 차단 플러그인
├── capstone_key_control/        # 단일 터미널 키 세션 (C++)
└── capstone_perception/         # D435i 이미지 뷰어 (좌표 발행 미구현)
```

---

## 1. 시스템 요구사항

| | 개발·실행 PC |
|---|---|
| **OS** | Ubuntu 22.04 |
| **ROS** | ROS 2 Humble |

### 검증한 버전

| 구성 | 버전 |
|---|---|
| MPlib | 0.2.1 |
| MoveIt 2 | 2.5.9 |
| `moveit_msgs` | 2.2.1 |
| `ros2_control` | 2.54.0 |
| `ros2_controllers` JTC | 2.53.1 |
| UR ROS 2 driver / MoveIt config | 2.13.2 |
| `mujoco_ros2_control` | 0.0.3 |
| NumPy | `<2` (MPlib 0.2.1 요구) |

---

## 2. 설치 및 빌드

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-mujoco-ros2-control \
  ros-humble-ur-robot-driver \
  ros-humble-realsense2-camera \
  python3-opencv
python3 -m pip install --user "numpy<2" "mplib==0.2.1"

cd ~/capstone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

<details><summary>관리자 권한 없이 MuJoCo를 쓰는 경우</summary>

현재 개발 PC에서는 MuJoCo deb를 `~/.local/ros-humble-mujoco`에 풀어 사용했습니다.
`ur3_mujoco.launch.py`가 이 경로를 자동 감지하므로 통합 런치에는 추가 설정이 필요 없습니다.
`ros2 pkg prefix mujoco_ros2_control`처럼 런치 밖에서 패키지를 직접 조회할 때만
아래를 현재 터미널에 추가합니다.

```bash
export AMENT_PREFIX_PATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble/lib:$HOME/.local/ros-humble-mujoco/opt/ros/humble/opt/mujoco_vendor/lib:$LD_LIBRARY_PATH
export PATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble/lib/mujoco_ros2_control:$PATH
export PYTHONPATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
```

시스템 설치가 있으면 `/opt/ros/humble` 패키지를 우선 사용합니다.

</details>

---

## 3. 설정

### 3.1 실행 게이트

| 인자 | 기본값 | 의미 |
|---|---|---|
| `plan_only` | `true` | 계획만 수행하고 궤적을 발행·출력 |
| `execute` | `false` | `trajectory_executor` 비활성 |

실제 움직임은 `plan_only:=false execute:=true`를 **동시에** 지정해야 발생합니다.
검증기는 관절 순서, 위치 한계, 속도 `0.5 rad/s`, 가속도 `0.5 rad/s²`, 시작점 오차
`0.05 rad`, 유한값, 시간 단조 증가를 검사하고 하나라도 어긋나면 전송하지 않습니다.

### 3.2 기구학 보정

루트의 `ur3_calibration.yaml`은 실장비에서 추출한 값이며
`ur3_calibrated.urdf`의 6개 관절 원점과 MJCF 모델에 함께 반영됩니다.

### 3.3 작업대 배치

| 항목 | `world` 기준 값 |
|---|---|
| 작업대 상판 중심 | `[-0.015, -0.015, 0.635] m` |
| 작업대 상판 크기 | `[0.90, 0.90, 0.15] m` |
| 작업대 상면 | `z = 0.710 m` |
| UR3 `base_link` | `[0, 0, 0.715] m` |

`5 mm` 간격은 UR3 base collision mesh가 상판과 초기 접촉하는 것을 막습니다.
같은 상자가 MPlib FCL 월드와 MoveIt PlanningScene에 동일 좌표로 등록됩니다.

---

## 4. 실행

### 4.1 MuJoCo 시뮬레이션

**A. 장면과 모델만 확인** (플래너 없음)

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=none \
  plan_only:=true execute:=false \
  initial_pose:=scan headless:=false show_ui:=true
```

`initial_pose:=scan`은 팔이 접힌 자세라 형상 확인이 쉽고, `home`은 `[0,-90,0,-90,0,0]°`이라
화면에서는 팔이 수직에 가깝게 보입니다. `show_profiler:=true`, `show_sensor:=true`로
solver 그래프와 관절·`tool0` 센서 그래프를 켤 수 있습니다.

**B. MPlib 계획만**

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=mplib named_goal:=scan \
  plan_only:=true execute:=false
```

**C. MoveIt 2 계획만**

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=moveit named_goal:=scan \
  plan_only:=true execute:=false
```

**D. MuJoCo에서 실제 궤적 실행**

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=mplib named_goal:=scan \
  plan_only:=false execute:=true headless:=false
```

`planner:=moveit`으로 바꿔도 실행기와 컨트롤러는 그대로입니다.

**E. 임의 관절각 / TCP pose**

통합 런치는 named goal만 받습니다. 임의 목표는 백엔드를 먼저 띄운 뒤 플래너 노드를 직접 실행합니다.

```bash
# 터미널 1: MuJoCo와 비활성 실행기
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=none plan_only:=true execute:=false

# 터미널 2: MoveIt 계획 서버 (MPlib은 이 터미널이 필요 없음)
ros2 launch capstone_planning ur3_move_group.launch.py use_sim_time:=true

# 터미널 3: [x,y,z,qx,qy,qz,qw] 목표, 계획만 발행
ros2 run capstone_planning moveit_plan --ros-args \
  -p use_sim_time:=true \
  -p goal_mode:=pose \
  -p goal_pose_xyzw:="[0.33,0.11,0.40,0.66,-0.65,0.27,-0.27]"
```

MPlib은 `mplib_plan`을 같은 `goal_mode:=pose`, `goal_pose_xyzw` 형식으로 실행합니다.

### 4.2 실제 UR3

> **미적용.** 새 MPlib/MoveIt 계획–실행 경로는 아직 실제 UR3에서 실행하지 않았습니다.
> 실행 절차는 실장비 검증을 마친 뒤 추가합니다.

### 4.3 모델 재생성

URDF를 수정했을 때만 MJCF를 다시 만듭니다.

```bash
source /opt/ros/humble/setup.bash
source ~/capstone_ws/install/setup.bash

ros2 run capstone_mujoco generate_ur3_mjcf \
  --urdf ~/capstone_ws/src/capstone_planning/description/ur3_calibrated.urdf \
  --inputs ~/capstone_ws/src/capstone_mujoco/config/ur3_mujoco_inputs.xml \
  --scene ~/capstone_ws/src/capstone_mujoco/description/ur3_scene.xml \
  --collision-meshes /opt/ros/humble/share/ur_description/meshes/ur3/collision \
  --output ~/capstone_ws/src/capstone_mujoco/description
```

---

## 5. 플래너

### Named goal

관절 순서는 `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`입니다.

| 이름 | 관절각 (deg) | 비고 |
|---|---|---|
| `home` | `[0, -90, 0, -90, 0, 0]` | 기준 자세 |
| `scan` | `[0, -90, 90, -135, -90, 0]` | 접힌 관측 자세 |
| `low_pick` | `[1.06, -64.67, 84.98, -155.31, -90.84, 0.66]` | `tool0` ≈ `[0.420, 0.120, 1.015] m`, 상판 위 `0.305 m` |

### MPlib과 MoveIt 2 비교

| 항목 | MPlib 0.2.1 | MoveIt 2 (2.5.9) |
|---|---|---|
| 실행 형태 | 계획 노드 내부 라이브러리 호출 | 별도 `move_group` 노드 필요 |
| 인터페이스 | `plan_qpos` / `plan_pose` 직접 호출 | `/plan_kinematic_path` 서비스 |
| 충돌 환경 | FCL planning world | `world` 프레임 `PlanningScene` |
| 경로 탐색 | 내장 OMPL RRT 계열 + 표본 재검사 | OMPL RRTConnect |
| 목표 형식 | 관절각 / TCP pose | 관절 제약 / TCP position·orientation 제약 |
| 추가 터미널 | 불필요 | TCP pose 계획 시 필요 |
| 실행 경로 | 공통 `trajectory_executor` | 공통 `trajectory_executor` (실행 액션 차단) |

---

## 6. 문제 해결

**`mujoco_ros2_control` 패키지를 찾지 못함**
- 통합 런치가 아닌 직접 실행이라면 [2. 설치 및 빌드](#2-설치-및-빌드)의 `<details>` 환경변수를 적용
- `source install/setup.bash`를 다시 실행

**컨트롤러가 스폰되지 않음**
- `ros2 control list_controllers`로 `joint_trajectory_controller` 상태 확인
- `capstone_mujoco/config/controllers.yaml`의 관절 이름이 모델과 일치하는지 확인

**계획은 성공하는데 로봇이 움직이지 않음**
- 정상 동작입니다. `plan_only:=false execute:=true`를 동시에 지정해야 전송됩니다
- 검증기 거부 사유는 `trajectory_executor` 로그에 출력됩니다

---

## 라이선스

MIT License — [LICENSE](LICENSE) 참조.

---

## 연락처

**유한민 (ryoohanmin)**
ryoohanmin@gmail.com
