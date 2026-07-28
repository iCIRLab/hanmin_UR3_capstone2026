"""Interactive MPlib/MoveIt planner selected by the migrated C++ key node."""

from typing import Optional

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, String
from trajectory_msgs.msg import JointTrajectory

from capstone_planning.joint_state import JointStateBuffer
from capstone_planning.model import NAMED_GOALS
from capstone_planning.moveit_request import (
    build_motion_plan_request,
    canonicalize_joint_trajectory,
    joint_goal_constraints,
)
from capstone_planning.mplib_planner import Ur3MplibPlanner
from capstone_planning.trajectory import (
    TrajectoryValidationError,
    plan_result_to_joint_trajectory,
    validate_joint_trajectory_message,
)
from capstone_planning.workcell import add_mplib_workcell
from capstone_planning.workcell import moveit_planning_scene


MPLIB_MODE = 1
MOVEIT_MODE = 2


class InteractivePlanNode(Node):
    """Route named-pose requests to MPlib or MoveIt without controller overlap."""

    def __init__(self) -> None:
        super().__init__('interactive_plan_node')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'planned_trajectory_topic',
            '/capstone_planning/planned_trajectory',
        )
        self.declare_parameter(
            'planner_mode_topic',
            '/capstone_planning/planner_mode',
        )
        self.declare_parameter(
            'goal_request_topic',
            '/capstone_planning/named_goal_request',
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
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('rrt_range', 0.1)
        self.declare_parameter('time_step', 0.05)
        self.declare_parameter('velocity_scaling', 1.0)
        self.declare_parameter('acceleration_scaling', 1.0)
        self.declare_parameter('max_joint_state_age_sec', 0.5)
        self.declare_parameter('maximum_start_error_rad', 0.05)

        self._state_buffer = JointStateBuffer()
        self._active_mode = MPLIB_MODE
        self._busy = False
        self._moveit_goal_name: Optional[str] = None
        self._moveit_inflight_name: Optional[str] = None

        self._mplib = Ur3MplibPlanner()
        add_mplib_workcell(self._mplib.planner)

        trajectory_topic = str(
            self.get_parameter('planned_trajectory_topic').value
        )
        self._publisher = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            1,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_state_callback,
            1,
        )
        self.create_subscription(
            Int16,
            str(self.get_parameter('planner_mode_topic').value),
            self._mode_callback,
            1,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('goal_request_topic').value),
            self._goal_request_callback,
            1,
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
        self._scene_request_sent = False
        self._scene_ready = False
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'Interactive planner ready: mode 1=MPlib, mode 2=MoveIt; '
            f'named goals={tuple(NAMED_GOALS)}.'
        )

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            self._state_buffer.update(message.name, message.position)
        except ValueError as error:
            self.get_logger().error(f'Invalid JointState ignored: {error}')

    def _mode_callback(self, message: Int16) -> None:
        mode = int(message.data)
        if mode not in (MPLIB_MODE, MOVEIT_MODE):
            self.get_logger().error(
                f'Invalid planner mode {mode}; use 1 or 2.'
            )
            return
        if self._busy:
            self.get_logger().warn(
                'Planner is busy; the new mode applies to the next request.'
            )
        self._active_mode = mode
        label = 'MPlib' if mode == MPLIB_MODE else 'MoveIt'
        self.get_logger().info(f'Planner mode selected: {mode} ({label}).')

    def _goal_request_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        request_mode = self._active_mode
        name = command
        if ':' in command:
            mode_text, name = command.split(':', maxsplit=1)
            try:
                request_mode = int(mode_text)
            except ValueError:
                self.get_logger().error(
                    f'Invalid planner command {command!r}.'
                )
                return
        if request_mode not in (MPLIB_MODE, MOVEIT_MODE):
            self.get_logger().error(
                f'Invalid planner mode {request_mode}; use 1 or 2.'
            )
            return
        if name not in NAMED_GOALS:
            self.get_logger().error(
                f'Unknown named goal {name!r}; use one of '
                f'{tuple(NAMED_GOALS)}.'
            )
            return
        if self._busy:
            self.get_logger().warn(
                'A planning request is active; the new request was ignored.'
            )
            return

        try:
            state = self._state_buffer.get(
                float(self.get_parameter('max_joint_state_age_sec').value)
            )
        except RuntimeError as error:
            self.get_logger().error(f'Cannot plan without fresh state: {error}')
            return

        if request_mode == MPLIB_MODE:
            self._run_mplib(name, state.positions)
            return

        self._busy = True
        self._moveit_goal_name = name
        self.get_logger().info(
            f'Queued MoveIt collision-aware plan to {name!r}.'
        )

    def _run_mplib(self, name: str, current_qpos) -> None:
        self._busy = True
        try:
            result = self._mplib.plan_joint_goal(
                current_qpos,
                NAMED_GOALS[name],
                time_step=float(self.get_parameter('time_step').value),
                rrt_range=float(self.get_parameter('rrt_range').value),
                planning_time=float(
                    self.get_parameter('planning_time').value
                ),
            )
            if not result.success:
                raise RuntimeError(result.message)
            trajectory, summary = plan_result_to_joint_trajectory(
                result,
                current_qpos,
            )
            if summary.no_op:
                self.get_logger().info(f'Goal {name!r} is already reached.')
                return
            self._publisher.publish(trajectory)
            self._log_published('MPlib', name, summary.point_count)
        except (
            RuntimeError,
            ValueError,
            TrajectoryValidationError,
        ) as error:
            self.get_logger().error(f'MPlib planning failed: {error}')
        finally:
            self._busy = False

    def _tick(self) -> None:
        if not self._scene_ready:
            self._try_apply_scene()
            return
        if self._moveit_goal_name is None:
            return
        if not self._plan_client.service_is_ready():
            return

        try:
            state = self._state_buffer.get(
                float(self.get_parameter('max_joint_state_age_sec').value)
            )
            name = self._moveit_goal_name
            request = GetMotionPlan.Request()
            request.motion_plan_request = build_motion_plan_request(
                state.positions,
                joint_goal_constraints(NAMED_GOALS[name]),
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
            self.get_logger().error(f'MoveIt request rejected: {error}')
            self._finish_moveit_request()
            return

        self._moveit_inflight_name = name
        self._moveit_goal_name = None
        future = self._plan_client.call_async(request)
        future.add_done_callback(self._moveit_result)

    def _try_apply_scene(self) -> None:
        if (
            self._scene_request_sent
            or not self._scene_client.service_is_ready()
        ):
            return
        request = ApplyPlanningScene.Request()
        request.scene = moveit_planning_scene()
        self._scene_request_sent = True
        future = self._scene_client.call_async(request)
        future.add_done_callback(self._scene_response)

    def _scene_response(self, future) -> None:
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
            'MoveIt workcell collision scene applied in the world frame.'
        )

    def _moveit_result(self, future) -> None:
        try:
            result = future.result().motion_plan_response
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(
                    f'MoveIt error code {result.error_code.val}'
                )
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
            self._log_published(
                'MoveIt',
                self._moveit_inflight_name or 'unknown',
                len(trajectory.points),
            )
        except (
            RuntimeError,
            ValueError,
            TrajectoryValidationError,
        ) as error:
            self.get_logger().error(f'MoveIt planning failed: {error}')
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Could not read MoveIt result: {error}')
        finally:
            self._finish_moveit_request()

    def _finish_moveit_request(self) -> None:
        self._moveit_goal_name = None
        self._moveit_inflight_name = None
        self._busy = False

    def _log_published(
        self,
        planner: str,
        goal_name: str,
        point_count: int,
    ) -> None:
        gate = (
            'PLAN-ONLY'
            if bool(self.get_parameter('plan_only').value)
            else 'EXECUTION REQUEST'
        )
        self.get_logger().info(
            f'{gate}: {planner} published {point_count} points for '
            f'{goal_name!r}.'
        )

    def destroy_node(self):
        self._mplib.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """Run the interactive dual-planner adapter."""
    rclpy.init(args=args)
    node = InteractivePlanNode()
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
