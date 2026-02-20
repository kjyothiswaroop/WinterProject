import os
import glob
import subprocess
import yaml
import numpy as np
from scipy.spatial.transform import Rotation


class RtabMapExtract():

    def __init__(self, db_path, export_dir):
        self.db_path = os.path.expanduser(db_path)
        self.export_dir = os.path.expanduser(export_dir)
        os.makedirs(self.export_dir, exist_ok=True)

    def extract(self, force=False):
        rgb_dir = os.path.join(self.export_dir, 'rtabmap_rgb')
        if not force and os.path.isdir(rgb_dir) and len(os.listdir(rgb_dir)) > 0:
            print(f'[RtabMapExtract] Export already exists at {self.export_dir}, skipping.')
            return

        cmd = [
            'rtabmap-export',
            '--images',
            '--poses',
            '--poses_camera',
            '--output', self.export_dir,
            self.db_path,
        ]
        print(f'[RtabMapExtract] Running: {" ".join(cmd)}')
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(
                f'rtabmap-export failed (code {result.returncode}):\n{result.stderr}'
            )
        print('[RtabMapExtract] Export complete.')

    def parse_intrinsics(self):
        calib_dir = os.path.join(self.export_dir, 'rtabmap_calib')
        yaml_files = glob.glob(os.path.join(calib_dir, '*.yaml'))
        if not yaml_files:
            raise FileNotFoundError(f'No calibration YAML found in {calib_dir}')

        with open(yaml_files[0], 'r') as f:
            # rtabmap_calib uses OpenCV YAML format which starts with "%YAML:1.0"
            # Standard yaml.safe_load cannot parse that directive — strip it first
            content = ''.join(
                line for line in f if not line.startswith('%YAML')
            )
        data = yaml.safe_load(content)

        K = data['camera_matrix']['data']
        return {
            'fx': K[0],
            'fy': K[4],
            'cx': K[2],
            'cy': K[5],
            'width':  data['image_width'],
            'height': data['image_height'],
        }

    def parse_poses(self):
        poses_file = os.path.join(self.export_dir, 'rtabmap_camera_poses.txt')
        if not os.path.exists(poses_file):
            raise FileNotFoundError(f'Camera poses file not found: {poses_file}')

        poses = {}
        with open(poses_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                tokens = line.split()
                timestamp = tokens[0]
                x, y, z   = float(tokens[1]), float(tokens[2]), float(tokens[3])
                qx, qy, qz, qw = float(tokens[4]), float(tokens[5]), float(tokens[6]), float(tokens[7])
                # tokens[8] is the RTABMap node id — not needed

                R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3]  = [x, y, z]
                poses[timestamp] = T

        return poses

    def get_frames(self):
        poses      = self.parse_poses()
        intrinsics = self.parse_intrinsics()
        rgb_dir    = os.path.join(self.export_dir, 'rtabmap_rgb')
        depth_dir  = os.path.join(self.export_dir, 'rtabmap_depth')

        frames  = []
        missing = 0
        for timestamp, pose in poses.items():
            rgb_path   = os.path.join(rgb_dir,   f'{timestamp}.jpg')
            depth_path = os.path.join(depth_dir, f'{timestamp}.png')

            if not os.path.exists(rgb_path) or not os.path.exists(depth_path):
                missing += 1
                continue

            frames.append({
                'timestamp':  timestamp,
                'rgb_path':   rgb_path,
                'depth_path': depth_path,
                'pose':       pose,
                'intrinsics': intrinsics,
            })

        frames.sort(key=lambda f: float(f['timestamp']))

        if missing:
            print(f'[RtabMapExtract] Warning: {missing} poses had no matching images.')
        print(f'[RtabMapExtract] {len(frames)} frames ready.')
        return frames

    def run(self, force=False):
        self.extract(force=force)
        return self.get_frames()
