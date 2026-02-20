import requests
import numpy as np
import cv2
import base64


class Sam3Client:

    def __init__(self, server_url, score_threshold=0.5):
        self.url = f"{server_url}/segment"
        self.score_threshold = score_threshold
    
    def segment(self, image_path, prompt):
        with open(image_path, 'rb') as f:
            files = {'image': (image_path, f, 'image/jpeg')}
            data  = {'prompt': prompt}
            response = requests.post(self.url, files=files, data=data, timeout=60)

        response.raise_for_status()
        raw = response.json()

        detections = []
        for det in raw['detections']:
            if det['score'] < self.score_threshold:
                continue

            # base64 PNG → boolean numpy mask (H x W)
            png_bytes = base64.b64decode(det['mask_b64'])
            nparr     = np.frombuffer(png_bytes, np.uint8)
            mask      = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE).astype(bool)

            detections.append({
                'mask':  mask,          # H x W bool numpy array
                'box':   det['box'],    # [x1, y1, x2, y2]
                'score': det['score'],
            })

        return detections 
