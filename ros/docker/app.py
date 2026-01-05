from contextlib import asynccontextmanager
from fastapi import FastAPI, UploadFile, File, HTTPException
import uvicorn
import numpy as np
import cv2
import sys

import numpy as np
import cv2

def decode_bytes_to_cv2(image_bytes: bytes) -> np.ndarray | None:
    """Decodes raw image bytes into an OpenCV image (numpy array).

    Uses OpenCV to decode the byte stream directly into a BGR numpy array.

    Args:
        image_bytes: The raw bytes of the image (e.g., from a network stream).

    Returns:
        A numpy array representing the image in BGR format if successful,
        otherwise None.
    """
    try:
        # Convert bytes to a numpy array of uint8. This is a fast operation as
        # it creates a memory view.
        nparr = np.frombuffer(image_bytes, np.uint8)

        # Decode the image array. cv2.IMREAD_COLOR loads the image in BGR format.
        # Use cv2.IMREAD_GRAYSCALE if only grayscale is needed for SLAM.
        img_cv2 = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        return img_cv2
    except (ValueError, cv2.error) as e:
        print(f"OpenCV Decode Error: {e}")
        return None
    except Exception as e:
        print(f"Unexpected Error during image decoding: {e}")
        return None

app = FastAPI()

@app.post("/loc")
def process_image(file: UploadFile = File(...)):
    try:
        file_bytes = file.file.read()
        
        img = decode_bytes_to_cv2(file_bytes)
        
        if img is None:
            raise HTTPException(status_code=400, detail="Invalid image file")
        
        result = len(list(img.shape))
        
        # 5. 返回结果 (FastAPI 会自动转为 JSON)
        return result

    except Exception as e:
        print(f"处理出错: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# 提示：使用 fastapi dev ros/app.py 或 fastapi run ros/app.py 启动
if __name__ == "__main__":
    # 监听 8000 端口
    uvicorn.run(app, host="0.0.0.0", port=8000)