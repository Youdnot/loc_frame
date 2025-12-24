from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uuid

app = FastAPI(title="My Application", description="This is a sample FastAPI application.")

class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()

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
                    await manager.send_personal_message(f"Server received: {text_data}", websocket)
                
                if "bytes" in message:
                    bytes_data = message["bytes"]
                    print(f"Received bytes: {len(list(bytes_data))}")
                    # 回复二进制数据给 Unity 客户端
                    await websocket.send_bytes(bytes_data)
                    
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")
