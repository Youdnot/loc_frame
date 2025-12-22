from fastapi import FastAPI, File, UploadFile, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
# import uvicorn
import time
import random
import json
import asyncio
from typing import List, Dict, Any, Optional

import cv2
import numpy as np

app = FastAPI(title="Image Process Test", description="测试图像文件传输API")


# -- maneger --
class HMDClient:
    """
    封装单个 Unity 客户端的连接和状态。
    负责管理该客户端专属的后台任务（如定时发送数据）。
    """
    def __init__(self, websocket: WebSocket):
        self.websocket = websocket
        self.tasks: List[asyncio.Task] = []

    async def start_background_tasks(self):
        """启动该客户端专属的后台任务"""
        self.tasks.append(asyncio.create_task(self._task_1()))
        self.tasks.append(asyncio.create_task(self._task_2()))

    async def stop(self):
        """停止所有后台任务并清理资源"""
        for task in self.tasks:
            task.cancel()
        # 等待任务优雅结束（忽略取消异常）
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
        self.tasks.clear()

    async def _task_1(self):
        pass

    async def _task_2(self):
        pass

class ConnectionManager:
    """
    连接管理器：负责全局连接的跟踪和管理。
    这使得我们可以通过 HTTP API 与 WebSocket 连接进行交互。
    """
    def __init__(self):
        self.active_connections: List[HMDClient] = []

    async def connect(self, websocket: WebSocket) -> HMDClient:
        await websocket.accept()
        client = HMDClient(websocket)
        self.active_connections.append(client)
        return client

    def disconnect(self, client: HMDClient):
        if client in self.active_connections:
            self.active_connections.remove(client)

    async def broadcast(self, message: str):
        """示例功能：向所有客户端广播消息"""
        for client in self.active_connections:
            try:
                await client.websocket.send_text(message)
            except Exception:
                pass

# -- data class --
class ImageEncode(BaseModel):
    image_verification: str  # base64 编码的图像字符串
    height: int
    width: int
    channels: int

class ProcessResponse(BaseModel):
    status: str
    timestamp: float
    process_time: float
    width: int
    height: int
    channels: int
    mean_intensity: str
    received_bytes: int


# -- methods --
def retrivePoseFromMap():
    """
    模拟从地图中检索位姿信息的函数。
    实际实现应调用 SLAM 系统或地图数据库。
    """
    # 模拟返回随机位姿数据
    pose = {
        "x": random.uniform(-10, 10),
        "y": random.uniform(-10, 10),
        "z": random.uniform(-10, 10),
        "roll": random.uniform(-3.14, 3.14),
        "pitch": random.uniform(-3.14, 3.14),
        "yaw": random.uniform(-3.14, 3.14)
    }
    return pose

def byte2image(data: bytes) -> Optional[np.ndarray]:
    """
    将 Unity 发送的 byte[] (通常是 JPG/PNG 编码) 转换为 OpenCV 图像格式 (numpy array)。
    
    Args:
        data: 图像的二进制数据 (bytes)
        
    Returns:
        img: OpenCV 图像对象 (BGR 格式的 numpy 数组)，如果解码失败则返回 None
    """
    if not data:
        return None
        
    # 1. 将二进制数据转换为 numpy 的 uint8 数组
    nparr = np.frombuffer(data, np.uint8)
    
    # 2. 解码为图像
    # cv2.IMREAD_COLOR: 加载彩色图像 (BGR)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    return img

@app.post("/hmd/image", response_model=ProcessResponse)
async def process_image(file: UploadFile = File(...)):
    """
    处理上传的图像文件，返回定位结果。
    """
    start_time = time.time()
    
    # 读取上传的图像文件
    image_data = await file.read()
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if img is None:
        raise HTTPException(status_code=400, detail="Invalid image file")
    
    height, width, channels = img.shape
    mean_val = float(np.mean(img))

    process_time = (time.time() - start_time) * 1000
    
    return ProcessResponse(
        status="success",
        timestamp=time.time(),
        process_time=process_time,
        width=width,
        height=height,
        channels=channels,
        mean_intensity=f"{mean_val:.2f}",
        received_bytes=len(image_data)
    )

@app.get("/")
def read_root():
    return {"message": "Service is running."}


# 初始化全局管理器
manager = ConnectionManager()

# --- HTTP API 部分 (展示如何与 WebSocket 交互) ---

@app.get("/stats")
async def get_stats():
    """API 接口：获取当前在线客户端数量"""
    return {
        "active_connections": len(manager.active_connections),
        "status": "running"
    }

@app.post("/broadcast")
async def broadcast_message(msg: str):
    """API 接口：通过 HTTP 请求向所有 Unity 客户端广播消息"""
    await manager.broadcast(f"System Broadcast: {msg}")
    return {"message": "Broadcast sent"}

# --- WebSocket 部分 ---

@app.websocket("/")
async def websocket_endpoint(websocket: WebSocket):
    # 1. 委托管理器处理连接
    client = await manager.connect(websocket)
    print("client joined.")
    
    # 2. 启动该客户端的独立任务
    await client.start_background_tasks()

    try:
        while True:
            # 3. 主循环只负责接收和处理消息，保持清晰
            message = await websocket.receive()
            
            if "text" in message and message["text"] is not None:
                data = message["text"]
                print(f"string received from client -> '{data}'")
            
            if "bytes" in message and message["bytes"] is not None:
                data = message["bytes"]
                byte_str = ", ".join(map(str, list(data)))
                print(f"binary received from client -> {byte_str}")

    except WebSocketDisconnect:
        print("client left.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 4. 清理资源
        manager.disconnect(client)
        await client.stop()

if __name__ == "__main__":
    # 启动服务，监听所有IP，端口8000
    # uvicorn.run(app, host="0.0.0.0", port=8000)