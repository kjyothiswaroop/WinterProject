import json
import os

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

COLOURS = [
    (0.2, 0.6, 1.0),   # blue
    (1.0, 0.8, 0.0),   # yellow
    (0.2, 0.9, 0.4),   # green
    (1.0, 0.3, 0.3),   # red
    (0.8, 0.3, 1.0),   # purple
]


class SemanticViz(Node):

    def __init__(self):
        super().__init__('semantic_viz')

        self.declare_parameter('semantic_json_path', '')
        path = os.path.expanduser(
            self.get_parameter('semantic_json_path').get_parameter_value().string_value)

        with open(path, 'r') as f:
            self.semantic_json = json.load(f)

        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'semantic_markers', qos)

        self.publish_markers()

    def create_marker(self, marker_type, position, label, marker_id, colour):
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'semantic_map'
        m.id                 = marker_id
        m.action             = Marker.ADD
        m.lifetime           = Duration(sec=0)  # 0 = lasts forever
        m.pose.orientation.w = 1.0
        m.color              = ColorRGBA(r=colour[0], g=colour[1], b=colour[2], a=0.9)

        m.pose.position.x = position[0]
        m.pose.position.y = position[1]
        m.pose.position.z = position[2]

        if marker_type == 'SPHERE':
            m.type    = Marker.SPHERE
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3

        elif marker_type == 'TEXT':
            m.type            = Marker.TEXT_VIEW_FACING
            m.text            = label
            m.scale.z         = 0.25                   # text height in metres
            m.pose.position.z = position[2] + 0.4      # float above the sphere

        return m

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id    = 0

        for i, (label, data) in enumerate(self.semantic_json.items()):
            centroid = data['centroid']
            colour   = COLOURS[i % len(COLOURS)]

            marker_array.markers.append(
                self.create_marker('SPHERE', centroid, label, marker_id, colour))
            marker_id += 1

            marker_array.markers.append(
                self.create_marker('TEXT', centroid, label, marker_id, colour))
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(self.semantic_json)} semantic markers')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticViz()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
