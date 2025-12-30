from contextlib import asynccontextmanager
from fastapi import FastAPI, UploadFile, File, HTTPException
import uvicorn
import numpy as np
import cv2
import sys

# 引入你的算法类
# 暂时不需要这样处理，后端另行开启
# from localization import PoseEstimator

from utils.byte2img import decode_bytes_to_cv2

# 全局变量存放算法实例
estimator = None

# @asynccontextmanager
# async def lifespan(app: FastAPI):
#     global estimator
#     print("Loading model...")
#     try:
#         # 初始化模型
#         estimator = PoseEstimator()
#         print("Model loaded successfully.")
#     except Exception as e:
#         print(f"Failed to load model: {e}")
#     yield
#     # 清理资源
#     estimator = None

app = FastAPI(lifespan=lifespan)

@app.post("/process")
def process_image(file: UploadFile = File(...)):
    global estimator
    
    if estimator is None:
        raise HTTPException(status_code=500, detail="Model not loaded")

    try:
        # 1. 读取上传文件的字节流
        # file.file 类似一个 Python 文件对象
        file_bytes = file.file.read()
        
        img = decode_bytes_to_cv2(file_bytes)
        
        if img is None:
            raise HTTPException(status_code=400, detail="Invalid image file")

        # 4. 调用算法进行推理
        result = estimator.predict(img)
        
        # 5. 返回结果 (FastAPI 会自动转为 JSON)
        return result

    except Exception as e:
        print(f"处理出错: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# 提示：使用 fastapi dev ros/app.py 或 fastapi run ros/app.py 启动
if __name__ == "__main__":
    # 监听 8000 端口
    uvicorn.run(app, host="0.0.0.0", port=8000)