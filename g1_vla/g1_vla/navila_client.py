import base64
import math
import threading
import time

import cv2
import requests
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class NaViLAClient(Node):
    def __init__(self):
        super().__init__('navila_client')

        self.declare_parameter('server_url',    'http://SERVER_IP:8000/infer')
        self.declare_parameter('linear_speed',  0.25)   # m/s  for FORWARD
        self.declare_parameter('angular_speed', 0.3)    # rad/s for turns
        self.declare_parameter('goal', (
            "Explore the environment by navigating forward and turning to discover new areas. "
            "Avoid revisiting the same space. Prefer moving forward when the path is clear. "
            "Turn to discover new directions when facing walls or dead ends. "
            "Only output stop if you are completely blocked with no possible direction to move."
        ))

        self.server_url    = self.get_parameter('server_url').value
        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal          = self.get_parameter('goal').value

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self._image_cb, 10)

        self.bridge           = CvBridge()
        self.latest_frame_jpg = None
        self.executing        = False

        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info(f'NaViLA client ready, server: {self.server_url}')

    # ── Camera ────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, jpg  = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        self.latest_frame_jpg = jpg.tobytes()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _timer_cb(self):
        if self.executing or self.latest_frame_jpg is None:
            return
        self.executing = True
        threading.Thread(target=self._step, daemon=True).start()

    def _step(self):
        try:
            self._stop()

            frame_b64 = base64.b64encode(self.latest_frame_jpg).decode()
            response  = requests.post(
                self.server_url,
                json={"frame": frame_b64, "goal": self.goal},
                timeout=120.0,
            )
            response.raise_for_status()
            action = response.json()
            self.get_logger().info(f"Action: {action}")

            self._execute(action)

        except Exception as e:
            self.get_logger().error(f'Step failed: {e}')
            self._stop()
        finally:
            self.executing = False

    # ── Action execution ──────────────────────────────────────────────────────

    def _execute(self, action: dict):
        kind = action.get("action", 0)

        if kind == 0:
            # STOP during exploration — re-query next cycle without moving
            self.get_logger().info('Model returned STOP — re-querying next cycle.')

        elif kind == 1:  # FORWARD
            distance_m = action.get("distance_cm", 25) / 100.0
            self._drive(self.linear_speed, 0.0, distance_m / self.linear_speed)

        elif kind == 2:  # TURN LEFT
            angle_rad = math.radians(action.get("angle_deg", 15))
            self._drive(0.0, self.angular_speed, angle_rad / self.angular_speed)

        elif kind == 3:  # TURN RIGHT
            angle_rad = math.radians(action.get("angle_deg", 15))
            self._drive(0.0, -self.angular_speed, angle_rad / self.angular_speed)

    def _drive(self, linear_x: float, angular_z: float, duration: float):
        self._publish(linear_x, angular_z)
        time.sleep(duration)
        self._stop()

    # ── cmd_vel helpers ───────────────────────────────────────────────────────

    def _publish(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x  = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = NaViLAClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
