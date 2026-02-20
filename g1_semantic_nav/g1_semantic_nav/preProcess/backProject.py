import cv2
import numpy as np

class BackProject():

    def __init__(self):
        pass

    def project_to_3D(self, mask, depth_path, pose_4x4, intrinsics):
        """Project the mask from SAM3 to 3D."""

        depth = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH).astype(np.float32) / 1000.0

        rows, columns = np.where(mask)
        depths = depth[rows, columns]

        valid = (depths > 0.1 ) & (depths < 5.0)
        rows = rows[valid]
        columns = columns[valid]
        depths = depths[valid]

        if len(depths) < 10:
            return None

        rows    = rows[::10]
        columns = columns[::10]
        depths  = depths[::10]

        fx, fy = intrinsics['fx'], intrinsics['fy']                                                                                                                                           
        cx, cy = intrinsics['cx'], intrinsics['cy']                                                                                                                                           

        x_cam = (columns - cx) * depths / fx
        y_cam = (rows    - cy) * depths / fy
        z_cam = depths

        points = np.stack([x_cam, y_cam, z_cam, np.ones(len(depths))], axis=1)

        points_in_map = (pose_4x4 @ points.T).T[:, :3]
        return np.median(points_in_map, axis=0).tolist()  # [x, y, z]
    
