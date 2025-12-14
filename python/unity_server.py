import asyncio
import os
from typing import List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

app = FastAPI()

class UnityClient:
    """
    封装单个 Unity 客户端的连接和状态。
    负责管理该客户端专属的后台任务（如定时发送数据）。
    """
    def __init__(self, websocket: WebSocket):
        self.websocket = websocket
        self.tasks: List[asyncio.Task] = []

    async def start_background_tasks(self):
        """启动该客户端专属的后台任务"""
        self.tasks.append(asyncio.create_task(self._send_text_interval()))
        self.tasks.append(asyncio.create_task(self._send_binary_interval()))

    async def stop(self):
        """停止所有后台任务并清理资源"""
        for task in self.tasks:
            task.cancel()
        # 等待任务优雅结束（忽略取消异常）
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
        self.tasks.clear()

    async def _send_text_interval(self):
        """每 100ms 发送一次文本"""
        try:
            while True:
                await self.websocket.send_text("hello world!")
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

    async def _send_binary_interval(self):
        """每 110ms 发送一次二进制数据"""
        try:
            while True:
                data = os.urandom(8)
                await self.websocket.send_bytes(data)
                await asyncio.sleep(0.11)
        except asyncio.CancelledError:
            pass

class ConnectionManager:
    """
    连接管理器：负责全局连接的跟踪和管理。
    这使得我们可以通过 HTTP API 与 WebSocket 连接进行交互。
    """
    def __init__(self):
        self.active_connections: List[UnityClient] = []

    async def connect(self, websocket: WebSocket) -> UnityClient:
        await websocket.accept()
        client = UnityClient(websocket)
        self.active_connections.append(client)
        return client

    def disconnect(self, client: UnityClient):
        if client in self.active_connections:
            self.active_connections.remove(client)

    async def broadcast(self, message: str):
        """示例功能：向所有客户端广播消息"""
        for client in self.active_connections:
            try:
                await client.websocket.send_text(message)
            except Exception:
                pass

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