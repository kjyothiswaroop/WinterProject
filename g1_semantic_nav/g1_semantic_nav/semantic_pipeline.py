import json
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
import struct
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from g1_semantic_nav.preProcess.backProject import BackProject
from g1_semantic_nav.preProcess.rtabmapExtract import RtabMapExtract
from g1_semantic_nav.preProcess.sam3Client import Sam3Client

from rich.progress import (Progress, SpinnerColumn, BarColumn,
                           TextColumn, MofNCompleteColumn, TimeElapsedColumn)
from sklearn.cluster import DBSCAN


class SemanticPipeline(Node):

    def __init__(self):
        super().__init__('semantic_pipeline')

        self.declare_parameter('dbpath',     rclpy.Parameter.Type.STRING)
        self.declare_parameter('export_dir', rclpy.Parameter.Type.STRING)
        self.declare_parameter('server_url', rclpy.Parameter.Type.STRING)
        self.declare_parameter('objects',    rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('debug',              rclpy.Parameter.Type.BOOL)
        self.declare_parameter('semantic_json_path', rclpy.Parameter.Type.STRING)

        db_path    = self.get_parameter('dbpath').value
        export_dir = self.get_parameter('export_dir').value
        server_url = self.get_parameter('server_url').value
        self.objects           = self.get_parameter('objects').value
        self.debug             = self.get_parameter('debug').value
        self.semantic_json_path = os.path.expanduser(
            self.get_parameter('semantic_json_path').value)

        self.rtabmap_extract = RtabMapExtract(db_path, export_dir)
        self.sam3_client = Sam3Client(server_url)
        self.back_proj = BackProject()

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, 'semantic_cloud', qos)
        

        self.run_pipeline()

    def run_pipeline(self):
        frames = self.rtabmap_extract.run()
        all_clouds = {label: [] for label in self.objects}

        with Progress(
            SpinnerColumn(),
            TextColumn("[bold cyan]{task.description}"),
            BarColumn(),
            MofNCompleteColumn(),
            TimeElapsedColumn(),
        ) as progress:
            for label in self.objects:
                task = progress.add_task(f'{label}', total=len(frames))
                for frame in frames:
                    detections = self.sam3_client.segment(frame['rgb_path'], label)
                    for det in detections:
                        points = self.back_proj.project_to_3D(
                            det['mask'],
                            frame['depth_path'],
                            frame['pose'],
                            frame['intrinsics']
                        )
                        if points is not None:
                            all_clouds[label].append(points)

                        if self.debug:
                            self._publish_debug_image(frame['rgb_path'], det['mask'], label)

                    progress.advance(task)

                progress.print(f'[green]✓[/green] {label}: '
                               f'{len(all_clouds[label])} raw detections')

        # Step 3 Clustering and centriods
        centroids = {}
        for label, clouds in all_clouds.items():

            if not clouds:
                self.get_logger().warn(f'No detections for: {label}')
                continue
            
            merged  = np.vstack(clouds)
            db      = DBSCAN(eps=0.5, min_samples=2).fit(merged)
            labels  = db.labels_
            best_cluster, best_count = None, 0
            
            for cid in set(labels):
                if cid == -1:
                    continue
                pts = merged[labels == cid]
                if len(pts) > best_count:
                    best_count   = len(pts)
                    best_cluster = np.median(pts, axis=0)
            
            if best_cluster is not None:
                centroids[label] = best_cluster
                self.get_logger().info(
                    f'{label}: centroid={np.round(best_cluster, 2).tolist()}')

        self.save_to_json(centroids)
        # self.publish_cloud(all_clouds)

    def save_to_json(self, centroids):
        data = {
            label: {'centroid': np.round(centroid, 4).tolist()}
            for label, centroid in centroids.items()
        }
        with open(self.semantic_json_path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(
            f'Saved {len(data)} centroids to {self.semantic_json_path}')

    def _publish_debug_image(self, rgb_path, mask, label):
        pass

    def publish_tfs(self, centriods):
        tfs=[]
        time = self.get_clock().now().to_msg()
        for label, centriod in centriods.items():
            t = TransformStamped()
            t.header.stamp = time
            t.header.frame_id = 'map'
            t.child_frame_id = label.replace(' ', '_')
            t.transform.translation.x = float(centriod[0])
            t.transform.translation.y = float(centriod[1])
            t.transform.translation.z = float(centriod[2])
            t.transform.rotation.w = 1.0
            tfs.append(t)
        
        self.tf_broadcaster.sendTransform(tfs)
        self.get_logger().info(f'Published {len(tfs)} static TF frames')

    def publish_cloud(self, all_clouds, vis_stride=5):
        COLOURS = [
            (0,   150, 255),   # blue
            (255, 220,   0),   # yellow
            (0,   230, 100),   # green
            (255,  80,  80),   # red
            (200,  80, 255),   # purple
        ]

        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        all_points = []
        for i, (label, clouds) in enumerate(all_clouds.items()):
            if not clouds:
                continue
            r, g, b = COLOURS[i % len(COLOURS)]
            rgb_float = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]

            merged = np.vstack(clouds)[::vis_stride]
            for pt in merged:
                all_points.append([float(pt[0]), float(pt[1]), float(pt[2]), rgb_float])

        if not all_points:
            self.get_logger().warn('No points to publish in semantic cloud')
            return

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        cloud = point_cloud2.create_cloud(header, fields, all_points)
        self.cloud_pub.publish(cloud)
        self.get_logger().info(f'Published semantic cloud with {len(all_points)} points')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPipeline()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
