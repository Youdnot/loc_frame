# Build based on [WebSockets - FastAPI](https://fastapi.tiangolo.com/advanced/websockets/#handling-disconnections-and-multiple-clients)

from typing import Union, List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File

from fastapi.responses import HTMLResponse

from pydantic import BaseModel

app = FastAPI(title="My Application", description="This is a sample FastAPI application.")