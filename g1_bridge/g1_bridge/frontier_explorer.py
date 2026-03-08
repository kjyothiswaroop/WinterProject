import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import numpy as np
from scipy import ndimage
import tf2_ros


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # /map is latched — TRANSIENT_LOCAL ensures we get the last published map on subscribe
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_map = None
        self.navigating = False
        self.min_frontier_size = 10  # filter out noisy single-cell frontier slivers
        self.forward_bias = 2.0      # weight for heading alignment (0 = off, higher = stronger forward preference)

        self.create_timer(2.0, self.exploration_step)

    # ── Map ──────────────────────────────────────────────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    # ── Robot pose ───────────────────────────────────────────────────────────

    def get_robot_pose(self):
        """Return (x, y, yaw) in the map frame, or None on failure."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # ── Frontier detection ───────────────────────────────────────────────────

    def find_frontiers(self, map_msg: OccupancyGrid):
        w   = map_msg.info.width
        h   = map_msg.info.height
        res = map_msg.info.resolution
        ox  = map_msg.info.origin.position.x
        oy  = map_msg.info.origin.position.y

        grid = np.array(map_msg.data, dtype=np.int8).reshape(h, w)

        free    = grid == 0
        unknown = grid == -1

        # A frontier cell is free and has at least one unknown neighbour
        frontier_mask = free & ndimage.binary_dilation(unknown, np.ones((3, 3), bool))

        labeled, n_clusters = ndimage.label(frontier_mask)

        frontiers = []
        for i in range(1, n_clusters + 1):
            cluster = labeled == i
            size = int(cluster.sum())
            if size < self.min_frontier_size:
                continue

            ys, xs = np.where(cluster)
            world_x = ox + (float(np.mean(xs)) + 0.5) * res
            world_y = oy + (float(np.mean(ys)) + 0.5) * res
            frontiers.append((world_x, world_y, size))

        return frontiers

    def pick_frontier(self, frontiers, robot_x, robot_y, robot_yaw):
        # Score = (size / dist) * forward_factor
        # forward_factor rewards frontiers that lie in front of the robot's heading.
        # cos(angle_diff) in [0,1] when the frontier is ahead, so we add a
        # weighted bonus: 1 + forward_bias * max(0, cos(angle_diff)).
        def score(f):
            wx, wy, size = f
            dist = max(np.hypot(wx - robot_x, wy - robot_y), 0.1)
            angle_to_frontier = math.atan2(wy - robot_y, wx - robot_x)
            angle_diff = angle_to_frontier - robot_yaw
            # Wrap to [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            forward_factor = 1.0 + self.forward_bias * max(0.0, math.cos(angle_diff))
            return (size / dist) * forward_factor

        return max(frontiers, key=score)

    # ── Main loop ────────────────────────────────────────────────────────────

    def exploration_step(self):
        if self.navigating or self.current_map is None:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y, robot_yaw = robot_pose
        frontiers = self.find_frontiers(self.current_map)

        if not frontiers:
            self.get_logger().info('No frontiers remaining — exploration complete.')
            return

        goal_x, goal_y, size = self.pick_frontier(frontiers, robot_x, robot_y, robot_yaw)
        goal_yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)
        self.get_logger().info(
            f'Navigating to frontier ({goal_x:.2f}, {goal_y:.2f}), cluster size={size}'
        )
        self.send_nav_goal(goal_x, goal_y, goal_yaw)

    # ── Nav2 goal ────────────────────────────────────────────────────────────

    def send_nav_goal(self, x, y, yaw=0.0):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 NavigateToPose server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Orient the goal to face toward the frontier
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.navigating = True
        self.nav_client.send_goal_async(goal).add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 — will retry next cycle')
            self.navigating = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_goal_done)

    def _on_goal_done(self, future):
        self.navigating = False
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Reached frontier.')
        else:
            self.get_logger().warn(f'Navigation ended with status {status}, picking new frontier.')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
