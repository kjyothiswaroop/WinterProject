import base64
import sys

import cv2
import numpy as np
import requests

SERVER  = "http://129.105.69.11:8000"
IMAGE   = sys.argv[1] if len(sys.argv) > 1 else "/home/kjs/.ros/rtabmap_rgb/1771210702.367458.jpg"
PROMPT  = sys.argv[2] if len(sys.argv) > 2 else "white fridge"

# --- call the server ---
with open(IMAGE, "rb") as f:
    resp = requests.post(f"{SERVER}/segment",
                         files={"image": f},
                         data={"prompt": PROMPT},
                         timeout=60)
resp.raise_for_status()
detections = resp.json()["detections"]
print(f"Found {len(detections)} detection(s) for '{PROMPT}'")

# --- load original image ---
img = cv2.imread(IMAGE)

for i, det in enumerate(detections):
    print(f"  [{i}] score={det['score']:.3f}  box={[round(x) for x in det['box']]}")

    # decode mask
    png_bytes = base64.b64decode(det["mask_b64"])
    nparr     = np.frombuffer(png_bytes, np.uint8)
    mask      = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE).astype(bool)

    # green overlay where mask is True
    overlay        = img.copy()
    overlay[mask]  = (0, 200, 0)
    img            = cv2.addWeighted(img, 0.5, overlay, 0.5, 0)

    # draw bounding box
    x1, y1, x2, y2 = [int(v) for v in det["box"]]
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(img, f"{PROMPT} {det['score']:.2f}", (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

out = "/tmp/sam3_result.jpg"
cv2.imwrite(out, img)
print(f"Saved to {out}")
