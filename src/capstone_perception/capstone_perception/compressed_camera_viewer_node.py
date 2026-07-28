import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CompressedCameraViewer(Node):
    """Display a compressed ROS image without overlays."""

    def __init__(self):
        super().__init__('compressed_camera_viewer')

        self.declare_parameter(
            'color_topic',
            '/sensors/d435i/color/image_raw/compressed',
        )
        self.declare_parameter('window_name', 'UR3 D435i Camera')

        self.bridge = CvBridge()
        self.window_name = self.get_parameter('window_name').value
        self.latest_frame = None

        self.create_subscription(
            CompressedImage,
            self.get_parameter('color_topic').value,
            self.on_color,
            10,
        )
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self.on_gui_tick)

    def on_color(self, msg):
        self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8',
        )

    def on_gui_tick(self):
        if self.latest_frame is None:
            return

        cv2.imshow(self.window_name, self.latest_frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = CompressedCameraViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
