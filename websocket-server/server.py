# Start with:
# fastapi run main.py
# to allow external connections

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uuid
import json
import math
import time
from datetime import datetime
from dataclasses import dataclass, asdict

import numpy as np
import cv2

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

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    client_id = str(uuid.uuid4())
    await manager.connect(websocket)
    print(f"Client {client_id} connected")
    try:
        while True:
            # 使用 receive() 方法可以同时处理文本和二进制消息
            message = await websocket.receive()
            
            if message["type"] == "websocket.receive":
                if "text" in message:
                    text_data = message["text"]
                    print(f"Received text: {text_data}")
                    # 回复消息给 Unity 客户端
                    await websocket.send_text(f"Server received: {text_data}")
                
                if "bytes" in message:
                    bytes_data = message["bytes"]
                    print(f"Received bytes: {len(bytes_data)}")

                    # Decode and display the image for debugging
                    img = decode_bytes_to_cv2(bytes_data)
                    # if img is not None:
                    #     cv2.imshow(f"Client {client_id}", img)
                    #     cv2.waitKey(1)
                    
                    # # 回复二进制数据给 Unity 客户端
                    # await websocket.send_bytes(bytes_data)

                    # 构造符合 Unity 端 ServerResponse 结构的测试数据
                    # 模拟一个简单的圆周运动，以便在 Unity 中观察到变化
                    t = time.time()
                    response = ServerResponse(
                        timestamp=datetime.now().isoformat(),
                        position=Vector3(
                            x=math.sin(t) * 0.5,
                            y=0.0,
                            z=math.cos(t) * 0.5
                        ),
                        rotation=Quaternion(
                            x=0.0,
                            y=0.0,
                            z=0.0,
                            w=1.0
                        )
                    )
                    
                    # 发送 JSON 文本给 Unity
                    await websocket.send_text(json.dumps(asdict(response)))
                    
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")
