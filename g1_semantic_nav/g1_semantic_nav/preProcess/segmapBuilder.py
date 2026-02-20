import os
import json
import numpy as np
from rich.progress import (Progress, SpinnerColumn, BarColumn,
                           TextColumn, MofNCompleteColumn, TimeElapsedColumn)
from preProcess.rtabmapExtract import RtabMapExtract
from preProcess.sam3Client import Sam3Client
from preProcess.backProject import BackProject
from sklearn.cluster import DBSCAN


class SegMapBuilder():

    def __init__(self, rtabmap_db_path, export_dir, server_url, objects, output_path):
        self.db_path = os.path.expanduser(rtabmap_db_path)
        self.export_dir = os.path.expanduser(export_dir)

        self.server_url = server_url
        self.objs = objects
        self.output = os.path.expanduser(output_path)

        self.rtabmap_extract = RtabMapExtract(self.db_path, self.export_dir)
        self.sam3_client = Sam3Client(self.server_url)
        self.back_proj = BackProject()

    def build(self):
        frames = self.rtabmap_extract.run()
        all_centroids = {label: [] for label in self.objs}

        with Progress(
            SpinnerColumn(),
            TextColumn("[bold cyan]{task.description}"),
            BarColumn(),
            MofNCompleteColumn(),
            TimeElapsedColumn(),
        ) as progress:
            for label in self.objs:
                task = progress.add_task(f'{label}', total=len(frames))
                for frame in frames:
                    detections = self.sam3_client.segment(frame['rgb_path'], label)
                    for det in detections:
                        centroid = self.back_proj.project_to_3D(
                            det['mask'],
                            frame['depth_path'],
                            frame['pose'],
                            frame['intrinsics']
                        )
                        if centroid is not None:
                            all_centroids[label].append(centroid)
                    progress.advance(task)

                progress.print(f'[green]✓[/green] {label}: '
                               f'{len(all_centroids[label])} raw detections')

        return all_centroids

    def _cluster(self, centroids):
        if len(centroids) < 2:
            return []

        points = np.array(centroids)
        db     = DBSCAN(eps=0.5, min_samples=2).fit(points)
        labels = db.labels_

        best_cluster, best_count = None, 0
        for cluster_id in set(labels):
            if cluster_id == -1:
                continue
            cluster_points = points[labels == cluster_id]
            if len(cluster_points) > best_count:
                best_count   = len(cluster_points)
                best_cluster = np.mean(cluster_points, axis=0).tolist()

        return best_cluster  # single [x, y, z] or None if everything was noise

    def save(self, all_centroids):
        semantic_map = {}

        for label, centroids in all_centroids.items():
            best = self._cluster(centroids)
            if best is not None:
                semantic_map[label] = {
                    'centroid':       best,
                    'num_detections': len(centroids)
                }
                print(f'[SegMapBuilder] {label}: found at {[round(v, 2) for v in best]}')
            else:
                print(f'[SegMapBuilder] {label}: no confident detection')

        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        with open(self.output, 'w') as f:
            json.dump(semantic_map, f, indent=2)
        print(f'[SegMapBuilder] Saved semantic map to {self.output}')

    def run(self):
        all_centroids = self.build()
        self.save(all_centroids)


if __name__ == '__main__':
    builder = SegMapBuilder(
        rtabmap_db_path='~/.ros/rtabmap.db',
        export_dir='~/.ros',
        server_url='http://129.105.69.11:8000',
        objects=['white fridge', 'yellow board', 'kitchen'],
        output_path='~/.ros/semantic_map.json',
    )
    builder.run()
