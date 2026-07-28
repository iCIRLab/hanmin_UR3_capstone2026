"""ROS 2 adapter that publishes one validated MPlib UR3 plan."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from capstone_planning.joint_state import JointStateBuffer
from capstone_planning.model import HOME_Q, NAMED_GOALS
from capstone_planning.mplib_planner import CartesianPose, Ur3MplibPlanner
from capstone_planning.trajectory import (
    TrajectoryValidationError,
    plan_result_to_joint_trajectory,
)
from capstone_planning.workcell import WORKCELL_BOXES, add_mplib_workcell


class MplibPlanNode(Node):
    """Plan once from a fresh JointState and publish a safe trajectory."""

    def __init__(self) -> None:
        super().__init__('mplib_plan_node')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'planned_trajectory_topic',
            '/capstone_planning/planned_trajectory',
        )
        self.declare_parameter('plan_only', True)
        self.declare_parameter('goal_mode', 'named')
        self.declare_parameter('named_goal', 'home')
        self.declare_parameter('goal_qpos', list(HOME_Q))
        self.declare_parameter(
            'goal_pose_xyzw',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('time_step', 0.05)
        self.declare_parameter('rrt_range', 0.1)
        self.declare_parameter('planning_time', 2.0)
        self.declare_parameter('max_joint_state_age_sec', 0.5)

        self._state_buffer = JointStateBuffer()
        self._planner = Ur3MplibPlanner()
        add_mplib_workcell(self._planner.planner)
        self._planned = False

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
        self.get_logger().info(
            f'MPlib planner ready with {len(WORKCELL_BOXES)} workcell '
            'obstacles. Waiting for a valid UR3 JointState.'
        )

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            self._state_buffer.update(message.name, message.position)
        except ValueError as error:
            self.get_logger().error(f'Invalid JointState ignored: {error}')
            return
        if self._planned:
            return

        self._planned = True
        try:
            snapshot = self._state_buffer.get(
                float(self.get_parameter('max_joint_state_age_sec').value)
            )
            result = self._make_plan(snapshot.positions)
            trajectory, summary = plan_result_to_joint_trajectory(
                result,
                snapshot.positions,
            )
            if summary.no_op:
                self.get_logger().info(
                    'Goal already reached; no trajectory was published.'
                )
                return

            self._publisher.publish(trajectory)
            mode = (
                'PLAN-ONLY'
                if bool(self.get_parameter('plan_only').value)
                else 'EXECUTION REQUEST'
            )
            self.get_logger().info(
                f'{mode}: published {summary.point_count} points, '
                f'{summary.duration:.3f}s, path length '
                f'{summary.path_length:.3f} rad.'
            )
        except (RuntimeError, ValueError, TrajectoryValidationError) as error:
            self.get_logger().error(f'MPlib planning failed: {error}')

    def _make_plan(self, current_qpos):
        options = {
            'time_step': float(self.get_parameter('time_step').value),
            'rrt_range': float(self.get_parameter('rrt_range').value),
            'planning_time': float(self.get_parameter('planning_time').value),
        }
        mode = str(self.get_parameter('goal_mode').value).lower()
        if mode == 'named':
            name = str(self.get_parameter('named_goal').value).lower()
            if name not in NAMED_GOALS:
                raise ValueError(
                    f'Unknown named_goal {name!r}; use one of '
                    f'{tuple(NAMED_GOALS)}'
                )
            result = self._planner.plan_joint_goal(
                current_qpos,
                NAMED_GOALS[name],
                **options,
            )
        elif mode == 'joint':
            result = self._planner.plan_joint_goal(
                current_qpos,
                self.get_parameter('goal_qpos').value,
                **options,
            )
        elif mode == 'pose':
            values = [
                float(value)
                for value in self.get_parameter('goal_pose_xyzw').value
            ]
            if len(values) != 7:
                raise ValueError(
                    'goal_pose_xyzw must be [x, y, z, qx, qy, qz, qw]'
                )
            pose = CartesianPose(
                position=tuple(values[:3]),
                quaternion_wxyz=(
                    values[6],
                    values[3],
                    values[4],
                    values[5],
                ),
            )
            result = self._planner.plan_pose_goal(
                current_qpos,
                pose,
                **options,
            )
        else:
            raise ValueError(
                f'Unknown goal_mode {mode!r}; use named, joint, or pose'
            )

        if not result.success:
            raise RuntimeError(result.message)
        return result

    def destroy_node(self):
        self._planner.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """Run the one-shot MPlib planning adapter."""
    rclpy.init(args=args)
    node = MplibPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Humble can raise from take_message() when launch-wide SIGINT shuts
        # down the context while this subscription is being serviced.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
