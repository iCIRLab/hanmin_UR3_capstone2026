# hanmin_UR3_capstone2026

**A UR3 manipulation stack that compares MPlib and MoveIt 2 on one shared, plan-gated execution path (ROS 2 Humble).**

[한국어 README](README.ko.md)

## Demo

The UR3 videos below were recorded with `ur3_pose_control`, which sends goal joint angles directly without any planner.

### UR3 — Scan → Home

https://github.com/user-attachments/assets/10c69df7-1f68-4c9d-badb-e4784a0701eb

### UR3 — Home → Scan

https://github.com/user-attachments/assets/a70367e5-fc25-490e-8b5b-d2ceaaec1e94

### UR3 — Grasping with a custom-built gripper

A grasping test with a custom-built gripper attached. Gripper control is not integrated into this stack yet.

https://github.com/user-attachments/assets/3398298d-5247-47cf-bab5-a05952b1811a

### MuJoCo — MPlib planning and execution

<!-- V4: MuJoCo video, MPlib plans and executes Home→Scan -->

### MuJoCo — MoveIt 2 planning and execution

<!-- V5: MuJoCo video, the same goal executed through MoveIt 2 -->

---

## Reference

- [haosulab/MPlib](https://github.com/haosulab/MPlib)
- [MPlib Getting Started](https://motion-planning-lib.readthedocs.io/latest/tutorials/getting_started.html)

---

## System Overview

<!-- IMG1: assets/mujoco_scene.png — the full MuJoCo scene with the UR3 mounted on the workbench -->

```text
┌──────────────────────────────────────────────────────────┐
│                 hanmin_UR3_capstone2026                  │
│                                                          │
│   named_goal / joint qpos / TCP pose                     │
│          ↓                                               │
│   ┌────────────────┐        ┌────────────────────────┐   │
│   │ MPlib 0.2.1    │        │ MoveIt 2 (OMPL)        │   │
│   │ in-process     │   or   │ /plan_kinematic_path   │   │
│   │ FCL world      │        │ PlanningScene          │   │
│   └───────┬────────┘        └───────────┬────────────┘   │
│           └───────────┬─────────────────┘                │
│                       ↓                                  │
│        /capstone_planning/planned_trajectory             │
│                       ↓                                  │
│   Trajectory validator (order, limits, start error, time)│
│                       ↓                                  │
│        FollowJointTrajectory  (single executor)          │
│                       ↓                                  │
│   ┌────────────────┐        ┌────────────────────────┐   │
│   │ MuJoCo JTC     │   or   │ Real UR3 scaled JTC    │   │
│   └────────────────┘        └────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

### Key Features

- **Planners**: MPlib and MoveIt 2 share the same model, limits, validator, and executor
- **MuJoCo**: a dedicated UR3 scene running on the official `mujoco_ros2_control` `SystemInterface`
- **Execution path**: `trajectory_executor` is the only place that sends a trajectory, and it opens only when `plan_only` and `execute` are switched together
- **Calibration**: `ur3_calibration.yaml` extracted from the real UR3 is applied to both the URDF and the MJCF

---

## Hardware

| Component | Model | Status |
|---|---|---|
| Robot arm | Universal Robots UR3 (CB3) | `ur_robot_driver` 2.13.2, `scaled_joint_trajectory_controller` |
| Camera | Intel RealSense D435i | Streaming verified; pose publishing not implemented |

---

## Package Structure

```text
src/
├── capstone_bringup/            # Integrated launch (MuJoCo / real UR3)
│   └── launch/
│       ├── ur3_arm.launch.py         # Single entry point: backend + planner + executor
│       ├── ur3_driver.launch.py      # Real UR3 driver
│       └── d435i_camera_view.launch.py
├── capstone_planning/           # Shared model and limits, both planners
│   ├── capstone_planning/
│   │   ├── model.py                  # Joint order, named goals, limit loading
│   │   ├── mplib_planner.py          # MPlib wrapper (FCL collision world)
│   │   ├── mplib_plan_node.py        # MPlib planning node
│   │   ├── moveit_plan_node.py       # MoveIt planning-service client
│   │   ├── workcell.py               # Workbench collision geometry shared by both planners
│   │   └── trajectory.py             # Trajectory construction and validation
│   ├── config/ur3_planning_limits.yaml
│   └── description/                  # ur3_calibrated.urdf, xacro
├── capstone_manipulation/       # Trajectory validation and execution
│   └── capstone_manipulation/
│       ├── trajectory_executor.py    # The only FollowJointTrajectory sender
│       └── ur3_pose_control.py       # Verified keyboard Home/Scan control (preserved)
├── capstone_mujoco/             # MuJoCo ros2_control backend
│   ├── description/                  # ur3_model.xml, ur3_scene.xml, assets
│   └── config/controllers.yaml
├── capstone_moveit_support/     # Plugin that blocks MoveIt's separate execution path
├── capstone_key_control/        # Single-terminal key session (C++)
└── capstone_perception/         # D435i image viewer (pose publishing not implemented)
```

---

## 1. System Requirements

| | Development / runtime PC |
|---|---|
| **OS** | Ubuntu 22.04 |
| **ROS** | ROS 2 Humble |

### Verified Versions

| Component | Version |
|---|---|
| MPlib | 0.2.1 |
| MoveIt 2 | 2.5.9 |
| `moveit_msgs` | 2.2.1 |
| `ros2_control` | 2.54.0 |
| `ros2_controllers` JTC | 2.53.1 |
| UR ROS 2 driver / MoveIt config | 2.13.2 |
| `mujoco_ros2_control` | 0.0.3 |
| NumPy | `<2` (required by MPlib 0.2.1) |

---

## 2. Installation

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

<details><summary>Using MuJoCo without administrator rights</summary>

On the current development PC the MuJoCo deb was extracted into `~/.local/ros-humble-mujoco`.
`ur3_mujoco.launch.py` detects that path automatically, so the integrated launch needs no extra
setup. Add the following to the current terminal only when querying the package outside the
launch file, e.g. `ros2 pkg prefix mujoco_ros2_control`.

```bash
export AMENT_PREFIX_PATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble/lib:$HOME/.local/ros-humble-mujoco/opt/ros/humble/opt/mujoco_vendor/lib:$LD_LIBRARY_PATH
export PATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble/lib/mujoco_ros2_control:$PATH
export PYTHONPATH=$HOME/.local/ros-humble-mujoco/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
```

If a system installation exists, packages under `/opt/ros/humble` take priority.

</details>

---

## 3. Setup

### 3.1 Execution Gates

| Argument | Default | Meaning |
|---|---|---|
| `plan_only` | `true` | Plan only; publish and print the trajectory |
| `execute` | `false` | `trajectory_executor` stays disabled |

Motion happens only when `plan_only:=false execute:=true` are given **together**.
The validator checks joint order, position limits, velocity `0.5 rad/s`, acceleration
`0.5 rad/s²`, start-state error `0.05 rad`, finiteness, and monotonic time stamps, and sends
nothing if any check fails.

### 3.2 Kinematic Calibration

`ur3_calibration.yaml` in the repository root was extracted from the real UR3 and is applied to
the six joint origins of `ur3_calibrated.urdf` and to the MJCF model.

### 3.3 Workbench Layout

The layout mirrors the actual laboratory setup.

| Item | Value in `world` |
|---|---|
| Workbench top center | `[-0.015, -0.015, 0.635] m` |
| Workbench top size | `[0.90, 0.90, 0.15] m` |
| Workbench surface | `z = 0.710 m` |
| UR3 `base_link` | `[0, 0, 0.715] m` |

The `5 mm` gap keeps the UR3 base collision mesh from touching the surface at startup.
The same box is registered in the MPlib FCL world and the MoveIt PlanningScene at identical
coordinates.

---

## 4. Usage

### 4.1 MuJoCo Simulation

**A. Scene and model only** (no planner)

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=none \
  plan_only:=true execute:=false \
  initial_pose:=scan headless:=false show_ui:=true
```

`initial_pose:=scan` folds the arm, which makes the geometry easy to inspect; `home` is
`[0,-90,0,-90,0,0]°`, so the arm looks nearly vertical on screen. `show_profiler:=true` and
`show_sensor:=true` enable the solver graph and the joint / `tool0` sensor graphs.

**B. MPlib, planning only**

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=mplib named_goal:=scan \
  plan_only:=true execute:=false
```

**C. MoveIt 2, planning only**

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=moveit named_goal:=scan \
  plan_only:=true execute:=false
```

**D. Executing the trajectory in MuJoCo**

```bash
ros2 launch capstone_bringup ur3_arm.launch.py \
  backend:=mujoco planner:=mplib named_goal:=scan \
  plan_only:=false execute:=true headless:=false
```

Switching to `planner:=moveit` keeps the same executor and controller.

### 4.2 Real UR3

> **Not applied yet.** The new MPlib / MoveIt planning and execution path has not been run on
> the real UR3. The procedure will be added once it has been verified on hardware.

### 4.3 Regenerating the Model

Regenerate the MJCF only after editing the URDF.

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

## 5. Planners

### Named Goals

The joint order is `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`.

| Name | Joint angles (deg) | Note |
|---|---|---|
| `home` | `[0, -90, 0, -90, 0, 0]` | Reference pose |
| `scan` | `[0, -90, 90, -135, -90, 0]` | Folded observation pose |
| `low_pick` | `[1.06, -64.67, 84.98, -155.31, -90.84, 0.66]` | `tool0` ≈ `[0.420, 0.120, 1.015] m`, `0.305 m` above the surface |

### MPlib vs. MoveIt 2

| Item | MPlib 0.2.1 | MoveIt 2 (2.5.9) |
|---|---|---|
| Execution form | Library call inside the planning node | Separate `move_group` node required |
| Interface | Direct `plan_qpos` / `plan_pose` calls | `/plan_kinematic_path` service |
| Collision world | FCL planning world | `PlanningScene` in the `world` frame |
| Search | Built-in OMPL RRT family + sample re-check | OMPL RRTConnect |
| Goal form | Joint angles / TCP pose | Joint constraints / TCP position and orientation constraints |
| Extra terminal | Not needed | Needed for TCP pose planning |
| Execution path | Shared `trajectory_executor` | Shared `trajectory_executor` (execution action blocked) |

---

## 6. Troubleshooting

**`mujoco_ros2_control` package not found**
- If running outside the integrated launch, apply the environment variables in the
  `<details>` block of [2. Installation](#2-installation)
- Re-run `source install/setup.bash`

**Controller does not spawn**
- Check `joint_trajectory_controller` with `ros2 control list_controllers`
- Verify that the joint names in `capstone_mujoco/config/controllers.yaml` match the model

**Planning succeeds but the robot does not move**
- This is expected. `plan_only:=false execute:=true` must be given together
- Rejection reasons from the validator are printed in the `trajectory_executor` log

---

## License

MIT License — see [LICENSE](LICENSE).

---

## Contact

**Hanmin Yoo (ryoohanmin)**
ryoohanmin@gmail.com
