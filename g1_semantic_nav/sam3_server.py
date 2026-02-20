"""
SAM3 FastAPI server — run this on the remote GPU server inside tmux.

    pip install fastapi uvicorn python-multipart
    tmux new -s sam3
    python sam3_server.py

The model loads once at startup (~30s). All subsequent /segment calls
run inference on the already-loaded model.
"""

import io
import base64

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse
from PIL import Image

from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor


app = FastAPI()

print("Loading SAM3 model — this takes ~30s...")
model     = build_sam3_image_model()
processor = Sam3Processor(model)
print("SAM3 ready. Listening for requests.")


@app.post("/segment")
async def segment(
    image:  UploadFile = File(...),
    prompt: str        = Form(...),
):
    # Decode uploaded image bytes → PIL RGB image
    contents  = await image.read()
    pil_image = Image.open(io.BytesIO(contents)).convert("RGB")

    # SAM3 inference
    state  = processor.set_image(pil_image)
    output = processor.set_text_prompt(state=state, prompt=prompt)

    masks  = output["masks"]   # [N, 1, H, W]  torch.bool
    boxes  = output["boxes"]   # [N, 4]
    scores = output["scores"]  # [N]

    detections = []
    for i in range(masks.shape[0]):
        # Boolean mask → uint8 → PNG bytes → base64 string
        mask_np   = masks[i, 0].cpu().numpy().astype(np.uint8) * 255
        _, encoded = cv2.imencode('.png', mask_np)
        mask_b64  = base64.b64encode(encoded).decode('utf-8')

        detections.append({
            "mask_b64": mask_b64,
            "box":      boxes[i].cpu().tolist(),
            "score":    float(scores[i].item()),
        })

    return JSONResponse({"detections": detections})


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
