import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class NavGoalServer(Node):

    def __init__(self):
        super().__init__('nav_goal_server')

        self.declare_parameter('semantic_json_path', rclpy.Parameter.Type.STRING)
        path = os.path.expanduser(
            self.get_parameter('semantic_json_path').value)

        with open(path, 'r') as f:
            self.semantic_map = json.load(f)

        self.get_logger().info(
            f'Loaded {len(self.semantic_map)} objects from {path}: '
            f'{list(self.semantic_map.keys())}')

        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/semantic_nav/status', 10)
        self.create_subscription(String, '/semantic_nav/query', self._on_query, 10)

        self.get_logger().info(
            'NavGoalServer ready — '
            'ros2 topic pub /semantic_nav/query std_msgs/String "data: \'white fridge\'" --once')

    def _publish_status(self, msg: str):
        self.get_logger().info(msg)
        self.status_pub.publish(String(data=msg))

    def _on_query(self, msg: String):
        object_name = msg.data.strip()

        if object_name not in self.semantic_map:
            self._publish_status(
                f'"{object_name}" not found. Available: {list(self.semantic_map.keys())}')
            return

        centroid = self.semantic_map[object_name]['centroid']
        ox, oy   = centroid[0], centroid[1]

        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = ox + 0.5
        pose.pose.position.y    = oy + 0.5
        pose.pose.position.z    = 0.0
        pose.pose.orientation.w = 1.0

        self.goal_pub.publish(pose)
        self._publish_status(
            f'Goal sent for "{object_name}" — '
            f'centroid ({ox:.2f}, {oy:.2f}), goal ({ox+0.5:.2f}, {oy+0.5:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
