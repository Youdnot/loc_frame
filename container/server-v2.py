# Maintain WebSocket connection and define API for localization data exchange 

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uuid
import json
import math
import time
from datetime import datetime
from dataclasses import dataclass, asdict

import numpy as np
import cv2

import aiohttp

# Data Models
@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass
class ServerResponse:
    timestamp: str
    position: Vector3
    rotation: Quaternion

app = FastAPI(title="My Application", description="This is a sample FastAPI application.")

class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    # async def send_personal_message(self, message: str, websocket: WebSocket):
        # await websocket.send_text(message)

    # async def broadcast(self, message: str):
    #     for connection in self.active_connections:
    #         await connection.send_text(message)

manager = ConnectionManager()

# utils

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
        # Convert bytes to a numpy array of uint8.
        # This is a fast operation as it creates a memory view.
        nparr = np.frombuffer(image_bytes, np.uint8)

        # Decode the image array.
        # cv2.IMREAD_COLOR loads the image in BGR format.
        # Use cv2.IMREAD_GRAYSCALE if only grayscale is needed for SLAM.
        img_cv2 = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        return img_cv2
    except Exception as e:
        print(f"OpenCV Decode Error: {e}")
        return None

LOCALIZER_URL = "http://pose-solver:8000/loc"

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    client_id = str(uuid.uuid4())
    await manager.connect(websocket)
    print(f"Client {client_id} connected")

    # 创建一个 HTTP Client Session，用于和 Container B 说话
    async with aiohttp.ClientSession() as session:
        try:
            while True:
                # 从外部接收数据 (假设是 bytes 格式的图片)
                # 这一步是“阻塞”的，直到客户端发来数据，但不会卡死整个服务
                image_data = await websocket.receive_bytes()
                
                # 转发给 Container B (通过 HTTP POST)
                # 这里我们构造一个 multipart 表单上传图片
                form = aiohttp.FormData()
                form.add_field('file', image_data, filename='frame.jpg', content_type='image/jpeg')
                
                async with session.post(LOCALIZER_URL, data=form) as resp:
                    if resp.status == 200:
                        pose_result = await resp.json()
                        # 把 Container B 的结果发回给外部 WebSocket
                        await websocket.send_json(pose_result)
                    else:
                        error_msg = await resp.text()
                        await websocket.send_json({"error": "Localization failed", "details": error_msg})
                    
        except WebSocketDisconnect:
            print("Client disconnected")
        except Exception as e:
            print(f"Error: {e}")
            await websocket.close()