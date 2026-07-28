# hanmin_UR3_capstone2026

**MPlib과 MoveIt 2를 하나의 계획–실행 경로 위에서 비교하는 UR3 매니퓰레이션 스택 (ROS 2 Humble).**

두 플래너가 동일한 보정 UR3 모델, 동일한 관절 한계, 동일한 궤적 검증기와 실행기를 공유합니다.
계획 결과만 바꿔 끼우므로 플래너 간 차이가 그대로 드러납니다. 기본 실행은
`plan_only:=true`, `execute:=false`이며 이 상태에서는 궤적을 검사·출력만 하고 컨트롤러로
보내지 않습니다. 모바일 베이스는 아직 이 범위에 통합하지 않았습니다.

[English README](README.md)

## Demo

### Real UR3 — Home 자세

<!-- V1: 기존 리포의 Home 자세 영상 URL을 이 줄에 단독으로 붙입니다 -->

### Real UR3 — S/scan 자세

<!-- V2: 기존 리포의 S/scan 자세 영상 URL -->

### 캡스톤 목표 시나리오

<!-- V3: 기존 리포의 캡스톤 예상 시나리오 영상 URL -->

### MuJoCo — MPlib 계획·실행

<!-- V4: MuJoCo에서 MPlib으로 Home→Scan 계획 후 실행하는 영상 URL -->

### MuJoCo — MoveIt 2 계획·실행

<!-- V5: 같은 목표를 MoveIt 2로 실행한 영상 URL -->

---

## 기준 자료

이 저장소는 논문이 아니라 캡스톤 구현물입니다. 구조와 참고한 구현은 다음과 같습니다.

- 입력 경계와 저장소 구성: [iCIRLab/icir_phri_panda_husky](https://github.com/iCIRLab/icir_phri_panda_husky)
- 계획 라이브러리: [haosulab/MPlib](https://github.com/haosulab/MPlib),
  [MPlib Getting Started](https://motion-planning-lib.readthedocs.io/latest/tutorials/getting_started.html)
- 시뮬레이션 백엔드: [ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)
- 실장비 드라이버: [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- MuJoCo + ROS 2 워크플로 참고:
  [Piper 예제](https://discourse.openrobotics.org/t/build-a-mujoco-ros2-robotic-arm-workflow-for-embodied-ai/55012),
  [yanyuze1/agilex_arm_mujoco](https://github.com/yanyuze1/agilex_arm_mujoco),
  [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)

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

## 키보드 전용 버전과의 차이

| 항목 | 이전 (`ur3_control`) | 현재 |
|---|---|---|
| 목표 생성 | 키 입력 → 고정 관절각 | named goal / 임의 관절각 / TCP pose |
| 경로 생성 | 없음 (컨트롤러 spline 보간) | MPlib 또는 MoveIt 2 |
| 충돌 검사 | 없음 | FCL planning world / MoveIt PlanningScene |
| 궤적 검증 | 없음 | 관절 순서·위치·속도·가속도·시작오차·단조시간 |
| 실행 대상 | 실제 UR3 전용 | MuJoCo 또는 실제 UR3 |
| 안전 기본값 | 즉시 실행 | 계획 전용 (`plan_only:=true`) |

기존에 실장비에서 검증한 키보드 Home/S 제어는 `capstone_manipulation`의
`ur3_pose_control`로 그대로 보존했습니다.

---

## 하드웨어

| 구성 | 모델 | 인터페이스 |
|---|---|---|
| 로봇 팔 | Universal Robots UR3 (CB3) | `ur_robot_driver` 2.13.2, `scaled_joint_trajectory_controller` |
| 카메라 | Intel RealSense D435i | `realsense2_camera`, compressed image |
| 모바일 베이스 | MD400T | 시리얼 — **보존만, 미통합** |

---

## 패키지 구조

```text
src/
├── capstone_bringup/            # 통합 런치 (MuJoCo·실장비·D435i)
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
├── capstone_perception/         # D435i compressed image viewer
├── md_motor_driver_ros2/        # 보존된 모바일 베이스 하위 시스템
└── serial-ros2/                 # 보존된 모바일 베이스 의존성
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
| `confirm_real_hardware` | `false` | `backend:=real` 사용 시 명시적 확인 필요 |

실제 움직임은 `plan_only:=false execute:=true`를 **동시에** 지정해야 발생합니다.
검증기는 관절 순서, 위치 한계, 속도 `0.5 rad/s`, 가속도 `0.5 rad/s²`, 시작점 오차
`0.05 rad`, 유한값, 시간 단조 증가를 검사하고 하나라도 어긋나면 전송하지 않습니다.

### 3.2 기구학 보정

루트의 `ur3_calibration.yaml`은 실장비에서 추출한 값이며
`ur3_calibrated.urdf`의 6개 관절 원점과 MJCF 모델에 함께 반영됩니다.
`kinematics_params_file` 인자로 다른 파일을 지정할 수 있습니다.

### 3.3 작업대 배치

| 항목 | `world` 기준 값 |
|---|---|
| 작업대 상판 중심 | `[-0.015, -0.015, 0.635] m` |
| 작업대 상판 크기 | `[0.90, 0.90, 0.15] m` |
| 작업대 상면 | `z = 0.710 m` |
| UR3 `base_link` | `[0, 0, 0.715] m` |

`5 mm` 간격은 UR3 base collision mesh가 상판과 초기 접촉하는 것을 막습니다.
같은 상자가 MPlib FCL 월드와 MoveIt PlanningScene에 동일 좌표로 등록됩니다.
수정 시 함께 고쳐야 하는 파일은 [docs/ur3_table_key_control.md](docs/ur3_table_key_control.md)에 있습니다.

### 3.4 실장비 네트워크

```text
[UR3 CB3]  ────LAN────  [ROS 2 PC]
192.168.56.1             192.168.56.2
 (robot_ip)              (reverse_ip)
```

티치펜던트의 External Control 프로그램이 `reverse_ip`를 가리켜야 합니다.

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

`initial_pose:=scan`은 팔이 접힌 자세라 형상 확인이 쉽고, `home`은 실장비 시험 기준인
`[0,-90,0,-90,0,0]°`이라 화면에서는 팔이 수직에 가깝게 보입니다.
`show_profiler:=true`, `show_sensor:=true`로 solver 그래프와 관절·`tool0` 센서 그래프를 켤 수 있습니다.

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

> ⚠️ 새 MPlib/MoveIt 실행 경로는 **아직 실제 UR3에서 실행하지 않았습니다.**
> 아래를 모두 직접 확인한 뒤에만 `backend:=real`을 사용하십시오.
>
> - 사람과 장애물이 로봇 작업 반경 밖에 있음
> - 비상정지 버튼을 즉시 누를 수 있음
> - Teach Pendant와 보호 정지가 정상임
> - 속도 슬라이더가 10–20 %임
> - External Control 프로그램이 준비됨
> - `ur3_calibration.yaml`과 UR3/PC IP가 실제 장비와 일치함

**Step 1 — 계획만 확인** (명시적 확인 플래그 필요)

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=real planner:=mplib named_goal:=home \
  plan_only:=true execute:=false \
  confirm_real_hardware:=true
```

**Step 2 — 실제 이동** (위 조건 재확인 후)

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=real planner:=mplib named_goal:=home \
  plan_only:=false execute:=true \
  confirm_real_hardware:=true
```

**Step 3 — 기존 검증된 직접 자세 제어** (플래너 경로와 별개, 위 영상 V1–V3의 구성)

```bash
ros2 launch capstone_bringup ur3_driver.launch.py
ros2 run capstone_manipulation ur3_pose_control   # h: Home, s: scan, f: 취소, q: 종료
```

### 4.3 D435i

```bash
ros2 launch capstone_bringup d435i_camera_view.launch.py
```

MuJoCo 장면의 고정 카메라는 `/workcell_overview/color`, `/workcell_overview/depth`로 발행됩니다.

### 4.4 모델 재생성

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

## 5. 플래너 선택과 제약

### Named goal

관절 순서는 `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`입니다.

| 이름 | 관절각 (deg) | 비고 |
|---|---|---|
| `home` | `[0, -90, 0, -90, 0, 0]` | 실장비 시험 기준 자세 |
| `scan` | `[0, -90, 90, -135, -90, 0]` | 접힌 관측 자세 |
| `low_pick` | `[1.06, -64.67, 84.98, -155.31, -90.84, 0.66]` | `tool0` ≈ `[0.420, 0.120, 1.015] m`, 상판 위 `0.305 m` |

### 주요 런치 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `backend` | `mujoco` | `mujoco` / `real` |
| `planner` | `mplib` | `mplib` / `moveit` / `none` |
| `named_goal` | `scan` | `home` / `scan` / `low_pick` |
| `planning_time` | `5.0` | 계획 제한 시간 (s) |
| `plan_only` | `true` | 계획만 수행 |
| `execute` | `false` | 실행기 활성화 |
| `confirm_real_hardware` | `false` | `backend:=real` 필수 확인 |
| `initial_pose` | `home` | MuJoCo 초기 관절 상태 |
| `headless` | `true` | MuJoCo 창 표시 여부 |
| `show_ui` / `show_right_ui` | `true` / `false` | 좌·우 패널 |
| `show_profiler` / `show_sensor` | `false` / `false` | 성능·센서 그래프 |
| `window_width` / `window_height` | `1100` / `620` | 창 크기 |
| `render_fps` / `render_vsync` | `60` / `false` | 렌더링 |
| `render_device` | `default` | `default` / `nvidia` |
| `launch_rviz` | `false` | RViz 동시 실행 |
| `robot_ip` / `reverse_ip` | `192.168.56.1` / `192.168.56.2` | 실장비 네트워크 |
| `kinematics_params_file` | `ur3_calibration.yaml` | 보정 파일 |

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

MoveIt은 실행 액션 대신 계획 서비스만 노출하도록 `capstone_moveit_support`가 컨트롤러
경계를 막고, 실제 컨트롤러 전송은 두 플래너가 공유하는 `trajectory_executor`
한 곳에서만 일어납니다.

### ⚠️ 알려진 제약

> **MPlib은 continuous joint를 constrained planning에 쓸 수 없습니다.**
> 원본 URDF는 그대로 두고 플래너 입력에서만 `wrist_3_joint`를 공통 `[-2π, 2π]`
> revolute 한계로 변환합니다. MoveIt에도 같은 bounded 모델을 적용해 두 플래너 조건을 맞췄습니다.

> **실제 UR3에서 새 플래너 경로는 아직 실행하지 않았습니다.**
> 영상 V1–V3은 이전 키보드 직접 제어(`ur3_pose_control`) 구성에서 촬영한 것입니다.

> **렌더링 성능은 GPU 드라이버에 좌우됩니다.**
> 현재 노트북의 Mesa 23.2는 Intel GPU PCI ID `0x7d51`을 지원하지 않아 Xorg가 `llvmpipe`로
> 시작합니다. MuJoCo 창이 전면에 있을 때 `800×480`에서 151–171 FPS였고, 다른 창에 가려지면
> Xorg가 13–18 FPS로 throttling합니다. 성능 측정은 창을 전면에 두고 해야 합니다.

> **`render_device:=nvidia`는 기본값이 아닙니다.**
> 현재 Humble 백엔드는 PRIME offload 상태에서 `Ctrl+C` 종료 시 GLdispatch 충돌이 재현됩니다.

> **모바일 베이스는 통합되지 않았습니다.**
> `md_motor_driver_ros2`와 `serial-ros2`는 보존만 되어 있고 이 계획–실행 경로에 연결되지 않았습니다.

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

**실제 UR3 연결 실패**
- 티치펜던트에서 External Control 프로그램 실행 여부 확인
- `robot_ip`, `reverse_ip`가 실제 네트워크와 일치하는지 확인
- `ping 192.168.56.1`

**D435i 스트림이 열리지 않음**
- USB 3.0 포트 연결 확인, `ros2 topic list | grep camera`

---

## 검증 기록

수치 비교와 시험 항목은 [docs/verification.md](docs/verification.md),
작업대 배치와 충돌 처리는 [docs/ur3_table_key_control.md](docs/ur3_table_key_control.md)에 있습니다.
주요 결과: MPlib이 사용하는 Pinocchio와 MuJoCo MJCF의 `tool0` FK 최대 위치 차이 `4.71e-7 m`,
회전행렬 차이 `1.09e-6`.

---

## 라이선스

MIT License — [LICENSE](LICENSE) 참조.

---

## 연락처

**류한민**
<!-- 소속 표기 -->
ryoohanmin@gmail.com
