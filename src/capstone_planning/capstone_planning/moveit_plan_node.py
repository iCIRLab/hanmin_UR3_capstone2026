"""ROS 2 adapter that requests and republishes one MoveIt plan."""

from typing import Optional

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from capstone_planning.joint_state import JointStateBuffer
from capstone_planning.model import HOME_Q, NAMED_GOALS
from capstone_planning.moveit_request import (
    build_motion_plan_request,
    canonicalize_joint_trajectory,
    joint_goal_constraints,
    pose_goal_constraints,
)
from capstone_planning.trajectory import (
    TrajectoryValidationError,
    validate_joint_trajectory_message,
)
from capstone_planning.workcell import WORKCELL_BOXES, moveit_planning_scene


class MoveItPlanNode(Node):
    """Request one plan-only MoveGroup result from a fresh UR3 state."""

    def __init__(self) -> None:
        super().__init__('moveit_plan_node')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'planned_trajectory_topic',
            '/capstone_planning/planned_trajectory',
        )
        self.declare_parameter(
            'motion_plan_service',
            '/plan_kinematic_path',
        )
        self.declare_parameter(
            'apply_planning_scene_service',
            '/apply_planning_scene',
        )
        self.declare_parameter('plan_only', True)
        self.declare_parameter('goal_mode', 'named')
        self.declare_parameter('named_goal', 'home')
        self.declare_parameter('goal_qpos', list(HOME_Q))
        self.declare_parameter(
            'goal_pose_xyzw',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('planning_time', 2.0)
        self.declare_parameter('velocity_scaling', 1.0)
        self.declare_parameter('acceleration_scaling', 1.0)
        self.declare_parameter('max_joint_state_age_sec', 0.5)
        self.declare_parameter('maximum_start_error_rad', 0.05)

        self._state_buffer = JointStateBuffer()
        self._request_sent = False
        self._scene_request_sent = False
        self._scene_ready = False

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter('planned_trajectory_topic').value),
            qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_state_callback,
            qos,
        )
        self._plan_client = self.create_client(
            GetMotionPlan,
            str(self.get_parameter('motion_plan_service').value),
        )
        self._scene_client = self.create_client(
            ApplyPlanningScene,
            str(
                self.get_parameter(
                    'apply_planning_scene_service'
                ).value
            ),
        )
        self.create_timer(0.1, self._try_plan)
        self.get_logger().info(
            'MoveIt planner adapter ready. Waiting to apply the workcell '
            'scene, then for MoveGroup and JointState.'
        )

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            self._state_buffer.update(message.name, message.position)
        except ValueError as error:
            self.get_logger().error(f'Invalid JointState ignored: {error}')

    def _try_plan(self) -> None:
        if not self._scene_ready:
            self._try_apply_scene()
            return
        if self._request_sent or not self._plan_client.service_is_ready():
            return
        try:
            state = self._state_buffer.get(
                float(self.get_parameter('max_joint_state_age_sec').value)
            )
            constraints, joint_goal = self._goal_constraints()
            if joint_goal is not None and max(
                abs(actual - target)
                for actual, target in zip(state.positions, joint_goal)
            ) < 1e-4:
                self._request_sent = True
                self.get_logger().info(
                    'Goal already reached; no MoveIt request was sent.'
                )
                return

            request = GetMotionPlan.Request()
            request.motion_plan_request = build_motion_plan_request(
                state.positions,
                constraints,
                planning_time=float(
                    self.get_parameter('planning_time').value
                ),
                velocity_scaling=float(
                    self.get_parameter('velocity_scaling').value
                ),
                acceleration_scaling=float(
                    self.get_parameter('acceleration_scaling').value
                ),
            )
        except (RuntimeError, ValueError) as error:
            self.get_logger().debug(f'Waiting to plan: {error}')
            return

        self._request_sent = True
        future = self._plan_client.call_async(request)
        future.add_done_callback(self._plan_response_callback)

    def _try_apply_scene(self) -> None:
        if self._scene_request_sent or not self._scene_client.service_is_ready():
            return
        request = ApplyPlanningScene.Request()
        request.scene = moveit_planning_scene()
        self._scene_request_sent = True
        future = self._scene_client.call_async(request)
        future.add_done_callback(self._scene_response_callback)

    def _scene_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self._scene_request_sent = False
            self.get_logger().error(
                f'PlanningScene service call failed: {error}'
            )
            return
        if not response.success:
            self._scene_request_sent = False
            self.get_logger().error('MoveIt rejected the workcell scene.')
            return
        self._scene_ready = True
        self.get_logger().info(
            f'Applied {len(WORKCELL_BOXES)} workcell obstacles to MoveIt.'
        )

    def _goal_constraints(self):
        mode = str(self.get_parameter('goal_mode').value).lower()
        if mode == 'named':
            name = str(self.get_parameter('named_goal').value).lower()
            if name not in NAMED_GOALS:
                raise ValueError(
                    f'Unknown named_goal {name!r}; use one of '
                    f'{tuple(NAMED_GOALS)}'
                )
            goal = NAMED_GOALS[name]
            return joint_goal_constraints(goal), goal
        if mode == 'joint':
            values = tuple(self.get_parameter('goal_qpos').value)
            return joint_goal_constraints(values), values
        if mode == 'pose':
            values = tuple(self.get_parameter('goal_pose_xyzw').value)
            return pose_goal_constraints(values), None
        raise ValueError(
            f'Unknown goal_mode {mode!r}; use named, joint, or pose'
        )

    def _plan_response_callback(self, future) -> None:
        try:
            result = future.result().motion_plan_response
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f'MoveIt planning failed with code '
                    f'{result.error_code.val}.'
                )
                return
            trajectory = canonicalize_joint_trajectory(
                result.trajectory.joint_trajectory
            )
            validate_joint_trajectory_message(trajectory)
            snapshot = self._state_buffer.get(
                float(self.get_parameter('max_joint_state_age_sec').value)
            )
            start_error = max(
                abs(actual - planned)
                for actual, planned in zip(
                    snapshot.positions,
                    trajectory.points[0].positions,
                )
            )
            maximum_error = float(
                self.get_parameter('maximum_start_error_rad').value
            )
            if start_error > maximum_error:
                raise TrajectoryValidationError(
                    f'MoveIt first point differs from current state by '
                    f'{start_error:.6f} rad'
                )
            self._publisher.publish(trajectory)
            last = trajectory.points[-1].time_from_start
            duration = last.sec + last.nanosec / 1_000_000_000.0
            mode = (
                'PLAN-ONLY'
                if bool(self.get_parameter('plan_only').value)
                else 'EXECUTION REQUEST'
            )
            self.get_logger().info(
                f'{mode}: MoveIt published {len(trajectory.points)} points, '
                f'{duration:.3f}s (planning {result.planning_time:.3f}s).'
            )
        except (
            RuntimeError,
            ValueError,
            TrajectoryValidationError,
        ) as error:
            self.get_logger().error(f'MoveIt result rejected: {error}')
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Could not read MoveIt result: {error}')


def main(args: Optional[list[str]] = None) -> None:
    """Run the one-shot MoveIt planning adapter."""
    rclpy.init(args=args)
    node = MoveItPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
