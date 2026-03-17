import json
import os
import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import FollowWaypoints

import torch
from transformers import CLIPModel, CLIPTokenizer


class NavGoalServer(Node):

    def __init__(self):
        super().__init__('nav_goal_server')

        self.declare_parameter('semantic_json_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('clip_threshold', 0.83)

        path = os.path.expanduser(
            self.get_parameter('semantic_json_path').value)
        self.clip_threshold = self.get_parameter('clip_threshold').value

        with open(path, 'r') as f:
            self.semantic_map = json.load(f)

        self.get_logger().info(
            f'Loaded {len(self.semantic_map)} objects from {path}: '
            f'{list(self.semantic_map.keys())}')

        # Load CLIP model and tokenizer
        self.get_logger().info('Loading CLIP model...')
        model_name = 'openai/clip-vit-base-patch32'
        self.clip_model = CLIPModel.from_pretrained(model_name)
        self.tokenizer = CLIPTokenizer.from_pretrained(model_name)
        self.clip_model.eval()
        self.get_logger().info('CLIP model loaded.')

        # Precompute embeddings for all object names
        self.object_names = list(self.semantic_map.keys())
        self.object_embeddings = self._embed_texts(self.object_names)
        self.get_logger().info(
            f'Precomputed CLIP embeddings for {len(self.object_names)} objects.')

        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/semantic_nav/status', 10)
        self.tts_pub    = self.create_publisher(String, '/semantic_nav/tts', 10)
        self.create_subscription(String, '/semantic_nav/query', self._on_query, 10)

        self.waypoint_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.get_logger().info(
            'NavGoalServer ready — '
            'ros2 topic pub /semantic_nav/query std_msgs/String "data: \'white fridge\'" --once')

    @torch.no_grad()
    def _embed_texts(self, texts: list[str]) -> np.ndarray:
        inputs = self.tokenizer(texts, padding=True, return_tensors='pt')
        out = self.clip_model.text_model(**inputs)
        embeddings = self.clip_model.text_projection(out.pooler_output)
        embeddings = embeddings / embeddings.norm(dim=-1, keepdim=True)
        return embeddings.cpu().numpy()

    def _find_best_match(self, query: str) -> tuple[str | None, float]:
        query_emb = self._embed_texts([query])
        similarities = (query_emb @ self.object_embeddings.T).squeeze(0)
        best_idx = int(np.argmax(similarities))
        best_score = float(similarities[best_idx])
        return self.object_names[best_idx], best_score

    def _make_pose(self, object_name: str) -> PoseStamped:
        centroid = self.semantic_map[object_name]['centroid']
        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = centroid[0] + 1.0
        pose.pose.position.y    = centroid[1] + 1.0
        pose.pose.position.z    = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_status(self, msg: str):
        self.get_logger().info(msg)
        self.status_pub.publish(String(data=msg))

    def _split_query(self, query: str) -> list[str]:
        parts = re.split(r'\s+and\s+|\s+then\s+|\s+after that\s+', query)
        return [p.strip() for p in parts if p.strip()]

    def _on_query(self, msg: String):
        query = msg.data.strip().lower()
        parts = self._split_query(query)

        matches = []
        for part in parts:
            object_name, score = self._find_best_match(part)
            self.get_logger().info(
                f'Part: "{part}" → best match: "{object_name}" (score: {score:.3f})')

            if score < self.clip_threshold:
                self._publish_status(
                    f'"{part}" not confident enough (best: "{object_name}", '
                    f'score: {score:.3f} < {self.clip_threshold}). '
                    f'Available: {self.object_names}')
                self.tts_pub.publish(String(data=f"{part} not found"))
                continue

            matches.append((object_name, score))

        if not matches:
            return

        # Single match — publish to /goal_pose as before
        if len(matches) == 1:
            name, score = matches[0]
            centroid = self.semantic_map[name]['centroid']
            pose = self._make_pose(name)
            self.goal_pub.publish(pose)
            self.tts_pub.publish(String(data=f"Walking to {name}"))
            self._publish_status(
                f'Goal sent for "{name}" (score: {score:.3f}) — '
                f'centroid ({centroid[0]:.2f}, {centroid[1]:.2f})')
            return

        # Multiple matches — use waypoint follower
        names = [m[0] for m in matches]
        poses = [self._make_pose(name) for name in names]
        route = ' → '.join(names)

        self.tts_pub.publish(String(data=f"Following route: {route}"))
        self._publish_status(f'Sending {len(poses)} waypoints: {route}')

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self._publish_status('Waypoint follower server not available!')
            return

        future = self.waypoint_client.send_goal_async(
            goal, feedback_callback=self._waypoint_feedback)
        future.add_done_callback(
            lambda f: self._on_waypoint_response(f, names))

    def _waypoint_feedback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Navigating to waypoint {idx}')

    def _on_waypoint_response(self, future, names):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status('Waypoint goal rejected!')
            return

        self._publish_status(f'Waypoint goal accepted: {" → ".join(names)}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_waypoint_result(f, names))

    def _on_waypoint_result(self, future, names):
        result = future.result().result
        missed = [w for w in result.missed_waypoints if w.waypoint_status == 3]
        if missed:
            missed_names = [names[w.waypoint_index] for w in missed
                           if w.waypoint_index < len(names)]
            self._publish_status(f'Route done. Missed: {missed_names}')
            self.tts_pub.publish(String(data=f"Missed {', '.join(missed_names)}"))
        else:
            self._publish_status(f'Route complete: {" → ".join(names)}')
            self.tts_pub.publish(String(data="Route complete"))


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
