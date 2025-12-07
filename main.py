from fastapi import FastAPI, File, UploadFile, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
import uvicorn
import time
import random
import json
import asyncio
from typing import List, Dict, Any

app = FastAPI(title="Edge SLAM Localization Service", description="用于接收图像并返回SLAM定位和矫正信息的API")

# --- 预留的 SLAM 接口与图像处理 ---
class SLAMProcessor:
    def __init__(self):
        """
        初始化 SLAM 系统。
        在这里加载地图、初始化算法模型或连接到后端 C++ SLAM 库。
        """
        print("Initializing SLAM system...")
        # self.slam_system = ORB_SLAM3.System(...) 
        pass

    def process_image(self, image_data: bytes) -> Dict[str, Any]:
        """
        处理图像数据并返回定位结果。
        这是连接 FastAPI 接口与实际 SLAM 算法的桥梁。
        
        Args:
            image_data: 图像的二进制数据 (JPEG/PNG 字节流)
            
        Returns:
            dict: 包含 pose, correction_matrix, confidence 的字典
        """
        # TODO: 1. 图像解码
        # 实际项目中通常使用 OpenCV 或 PIL 解码
        # import cv2
        # import numpy as np
        # nparr = np.frombuffer(image_data, np.uint8)
        # img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        # TODO: 2. 调用 SLAM 核心算法进行定位
        # pose, correction = self.slam_system.TrackMonocular(img, time.time())
        
        # --- 以下为模拟返回数据 ---
        return {
            "pose": {
                "x": random.uniform(-5.0, 5.0),
                "y": random.uniform(-5.0, 5.0),
                "z": random.uniform(0.0, 2.0),
                "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
            },
            "correction_matrix": [
                [1.0, 0.0, 0.0, 0.01],
                [0.0, 1.0, 0.0, -0.02],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ],
            "confidence": 0.98
        }

# 实例化全局处理器，确保模型只加载一次
slam_processor = SLAMProcessor()

# 定义返回的数据模型
class Pose(BaseModel):
    x: float
    y: float
    z: float
    qx: float  # 四元数
    qy: float
    qz: float
    qw: float

class LocalizationResponse(BaseModel):
    status: str
    timestamp: float
    pose: Pose
    correction_matrix: List[List[float]] # 矫正矩阵 4x4
    confidence: float
    processing_time_ms: float

@app.websocket("/ws/slam")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket 接口，用于高频/低延迟的实时SLAM数据传输。
    Unity端可以通过 WebSocket 发送图像二进制数据，服务端实时返回定位结果。
    """
    await websocket.accept()
    try:
        while True:
            # 接收二进制图像数据
            data = await websocket.receive_bytes()
            start_time = time.time()
            
            # 调用 SLAM 处理器处理图像
            # 这里传入的是原始字节流，解码工作在 process_image 内部完成
            result = slam_processor.process_image(data)
            
            process_time = (time.time() - start_time) * 1000
            
            response = {
                "status": "success",
                "timestamp": time.time(),
                "pose": result["pose"],
                "correction_matrix": result["correction_matrix"],
                "confidence": result["confidence"],
                "processing_time_ms": process_time
            }
            
            # 发送JSON结果回Unity
            await websocket.send_json(response)
            
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")
        await websocket.close()

@app.post("/slam/localize", response_model=LocalizationResponse)
async def localize_image(file: UploadFile = File(...)):
    """
    接收图像用于SLAM定位计算和矫正。
    
    - **file**: 上传的图像文件 (JPEG/PNG)
    """
    start_time = time.time()
    
    if not file.content_type.startswith("image/"):
        raise HTTPException(status_code=400, detail="File must be an image")

    # 读取图像数据
    contents = await file.read()
    
    # 调用 SLAM 处理器
    result = slam_processor.process_image(contents)
    
    process_time = (time.time() - start_time) * 1000

    return LocalizationResponse(
        status="success",
        timestamp=time.time(),
        pose=Pose(**result["pose"]),
        correction_matrix=result["correction_matrix"],
        confidence=result["confidence"],
        processing_time_ms=process_time
    )

@app.get("/")
def read_root():
    return {"message": "SLAM Localization Service is running. POST image to /slam/localize"}

if __name__ == "__main__":
    # 启动服务，监听所有IP，端口8000
    uvicorn.run(app, host="0.0.0.0", port=8000)
