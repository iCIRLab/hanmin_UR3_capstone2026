"""Safety-gated FollowJointTrajectory executor shared by sim and real UR3."""

from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from capstone_planning.joint_state import JointStateBuffer
from capstone_planning.trajectory import (
    TrajectoryValidationError,
    validate_joint_trajectory_message,
)


class TrajectoryExecutor(Node):
    """Validate planned trajectories and send one monitored action goal."""

    def __init__(self) -> None:
        super().__init__('trajectory_executor')
        self.declare_parameter('enabled', False)
        self.declare_parameter(
            'controller_action',
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.declare_parameter(
            'trajectory_topic',
            '/capstone_planning/planned_trajectory',
        )
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('max_joint_state_age_sec', 0.5)
        self.declare_parameter('maximum_start_error_rad', 0.05)
        self.declare_parameter('goal_time_tolerance_sec', 2.0)

        self._state_buffer = JointStateBuffer()
        self._goal_handle = None
        self._busy = False
        self._cancel_when_accepted = False

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_state_callback,
            qos,
        )
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter('trajectory_topic').value),
            self._trajectory_callback,
            qos,
        )
        self.create_service(Trigger, '~/cancel', self._cancel_callback)

        action_name = str(self.get_parameter('controller_action').value)
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name,
        )

        if bool(self.get_parameter('enabled').value):
            self.get_logger().warn(
                f'Trajectory execution ENABLED for {action_name}.'
            )
        else:
            self.get_logger().info(
                'Trajectory execution disabled; plans will not be sent.'
            )

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            self._state_buffer.update(message.name, message.position)
        except ValueError as error:
            self.get_logger().error(f'Invalid JointState ignored: {error}')

    def _trajectory_callback(self, trajectory: JointTrajectory) -> None:
        if not bool(self.get_parameter('enabled').value):
            self.get_logger().warn(
                'Received a plan while execution is disabled; not sending it.'
            )
            return
        if self._busy:
            self.get_logger().error(
                'A trajectory is already active; cancel it before sending another.'
            )
            return

        try:
            validate_joint_trajectory_message(trajectory)
            state = self._state_buffer.get(
                float(self.get_parameter('max_joint_state_age_sec').value)
            )
            start_error = max(
                abs(actual - planned)
                for actual, planned in zip(
                    state.positions,
                    trajectory.points[0].positions,
                )
            )
            maximum_error = float(
                self.get_parameter('maximum_start_error_rad').value
            )
            if start_error > maximum_error:
                raise TrajectoryValidationError(
                    f'First point differs from current state by '
                    f'{start_error:.6f} rad'
                )
        except (RuntimeError, TrajectoryValidationError) as error:
            self.get_logger().error(f'Trajectory rejected: {error}')
            return

        if not self._action_client.server_is_ready():
            self.get_logger().error(
                f'Controller action is not ready: '
                f'{self.get_parameter("controller_action").value}'
            )
            return

        tolerance = max(
            0.0,
            float(self.get_parameter('goal_time_tolerance_sec').value),
        )
        whole_seconds = int(tolerance)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(
            sec=whole_seconds,
            nanosec=int((tolerance - whole_seconds) * 1_000_000_000),
        )

        self._busy = True
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Goal transmission failed: {error}')
            self._busy = False
            return

        if not handle.accepted:
            self.get_logger().error('Controller rejected the trajectory.')
            self._busy = False
            return

        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_callback)
        if self._cancel_when_accepted:
            self._cancel_when_accepted = False
            handle.cancel_goal_async()

    def _result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Trajectory execution succeeded.')
            elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Trajectory execution was canceled.')
            else:
                result = wrapped_result.result
                self.get_logger().error(
                    f'Trajectory failed: status={wrapped_result.status}, '
                    f'code={result.error_code}, {result.error_string}'
                )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Could not read action result: {error}')
        finally:
            self._goal_handle = None
            self._busy = False

    def _cancel_callback(self, request, response):
        del request
        if not self._busy:
            response.success = False
            response.message = 'No trajectory is active.'
            return response
        if self._goal_handle is None:
            self._cancel_when_accepted = True
            response.success = True
            response.message = 'Goal will be canceled as soon as it is accepted.'
            return response

        self._goal_handle.cancel_goal_async()
        response.success = True
        response.message = 'Cancellation requested.'
        return response


def main(args: Optional[list[str]] = None) -> None:
    """Run the trajectory executor node."""
    rclpy.init(args=args)
    node = TrajectoryExecutor()
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
