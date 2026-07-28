import select
import sys
import termios
import tty
from typing import Dict, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_ORDER = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# 실제 UR3에서 검증한 관절 자세(rad)
POSES: Dict[str, List[float]] = {
    'h': [0.0, -1.5707963268, 0.0, -1.5707963268, 0.0, 0.0],
    's': [0.0, -1.5707963268, 1.5707963268, -2.3561944902,
          -1.5707963268, 0.0],
}


class Ur3PoseControl(Node):
    def __init__(self):
        super().__init__('ur3_pose_control')

        self.declare_parameter(
            'controller_name', 'scaled_joint_trajectory_controller')
        self.declare_parameter('move_time_sec', 12.0)

        self.program_running: Optional[bool] = None
        self.speed_scaling: Optional[float] = None
        self.motion_busy = False
        self.goal_handle = None
        self.cancel_when_accepted = False
        self.terminal_settings = None

        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Bool,
            '/io_and_status_controller/robot_program_running',
            self._program_running_cb,
            state_qos,
        )
        self.create_subscription(
            Float64,
            '/speed_scaling_state_broadcaster/speed_scaling',
            self._speed_scaling_cb,
            state_qos,
        )

        controller = self.get_parameter('controller_name').value
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{controller}/follow_joint_trajectory',
        )

        if not sys.stdin.isatty():
            raise RuntimeError('키 입력이 가능한 터미널에서 직접 실행해야 합니다.')

        self.terminal_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        self.create_timer(0.05, self._poll_keyboard)

        self.get_logger().warn(
            '로봇 주변을 비우고 비상정지 버튼을 잡으십시오. '
            '키: h=Home, s=S/scan, f=정지(목표 취소), q=종료'
        )

    def _program_running_cb(self, msg: Bool):
        self.program_running = bool(msg.data)

    def _speed_scaling_cb(self, msg: Float64):
        self.speed_scaling = float(msg.data)

    def _poll_keyboard(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return

        key = sys.stdin.read(1).lower()
        if key in POSES:
            self._move_to(key)
        elif key == 'f':
            self._cancel_motion()
        elif key in ('q', '\x1b'):
            if self.motion_busy:
                self.get_logger().warn('이동 중입니다. f로 정지한 뒤 q를 누르십시오.')
            else:
                self.get_logger().info('키보드 자세 제어를 종료합니다.')
                rclpy.shutdown()

    def _ready_to_move(self) -> bool:
        if self.program_running is not True:
            self.get_logger().error(
                'UR External Control 프로그램이 실행 중이 아닙니다. '
                '티치펜던트에서 Play를 누르십시오.'
            )
            return False
        if self.speed_scaling is None or self.speed_scaling <= 0.0:
            self.get_logger().error(
                '속도 슬라이더가 0이거나 상태를 받지 못했습니다.'
            )
            return False
        if not self.action_client.server_is_ready():
            self.get_logger().error(
                'scaled_joint_trajectory_controller 액션 서버가 없습니다.'
            )
            return False
        return True

    def _build_goal(self, positions: List[float]) -> FollowJointTrajectory.Goal:
        seconds = max(1.0, float(self.get_parameter('move_time_sec').value))
        whole_seconds = int(seconds)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=whole_seconds,
            nanosec=int((seconds - whole_seconds) * 1e9),
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_ORDER
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(sec=5)
        return goal

    def _move_to(self, key: str):
        if self.motion_busy:
            self.get_logger().warn('이미 이동 중입니다. 먼저 f로 정지하십시오.')
            return
        if not self._ready_to_move():
            return

        pose_name = 'Home' if key == 'h' else 'S/scan'
        seconds = float(self.get_parameter('move_time_sec').value)
        self.get_logger().warn(f'{pose_name} 자세로 {seconds:.1f}초 동안 이동합니다.')
        self.motion_busy = True
        future = self.action_client.send_goal_async(self._build_goal(POSES[key]))
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'목표 전송 실패: {error}')
            self.motion_busy = False
            return

        if not handle.accepted:
            self.get_logger().error('컨트롤러가 자세 목표를 거부했습니다.')
            self.motion_busy = False
            return

        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)
        if self.cancel_when_accepted:
            self.cancel_when_accepted = False
            self._cancel_motion()

    def _cancel_motion(self):
        if not self.motion_busy:
            self.get_logger().info('현재 실행 중인 이동이 없습니다.')
            return
        if self.goal_handle is None:
            self.cancel_when_accepted = True
            self.get_logger().warn('목표가 접수되는 즉시 정지합니다.')
            return

        self.get_logger().warn('현재 이동 목표를 취소하고 정지합니다.')
        self.goal_handle.cancel_goal_async()

    def _result_cb(self, future):
        try:
            wrapped_result = future.result()
            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('목표 자세 이동을 완료했습니다.')
            elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('이동 목표가 취소되었습니다.')
            else:
                result = wrapped_result.result
                self.get_logger().error(
                    f'이동 실패: status={wrapped_result.status}, '
                    f'code={result.error_code}, {result.error_string}'
                )
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'이동 결과 확인 실패: {error}')
        finally:
            self.goal_handle = None
            self.motion_busy = False

    def destroy_node(self):
        if self.terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN, self.terminal_settings)
            self.terminal_settings = None
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Ur3PoseControl()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as error:
        if isinstance(error, RuntimeError):
            print(f'[ur3_pose_control] {error}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
