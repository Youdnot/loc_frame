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

import aiohttp

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

LOCALIZER_URL = "http://127.0.0.1:8000/loc"

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