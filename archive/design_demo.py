from fastapi import FastAPI, File, UploadFile, HTTPException, BackgroundTasks
from pydantic import BaseModel
import httpx
import shutil
import os
import uuid
import time
from typing import Optional

app = FastAPI()

# 模拟配置
CALCULATION_SERVICE_URL = "http://localhost:8001/process" # 假设这是另一个服务的地址
TEMP_DIR = "temp_images"

# 确保临时目录存在
os.makedirs(TEMP_DIR, exist_ok=True)

class CalculationResult(BaseModel):
    score: float
    tags: list[str]
    processed_at: float

def cleanup_file(path: str):
    """后台任务：清理临时文件"""
    try:
        if os.path.exists(path):
            os.remove(path)
            print(f"Deleted temp file: {path}")
    except Exception as e:
        print(f"Error deleting file {path}: {e}")

async def process_image_logic(file_path: str, filename: str, content_type: str) -> dict:
    """
    核心计算逻辑：独立于 HTTP 接口。
    可以在这里调用外部服务、运行本地算法等。
    """
    # 模拟调用另一个服务进行计算
    # 这里演示如何将文件转发给另一个服务
    async with httpx.AsyncClient() as client:
        # 构造 multipart/form-data 请求转发给下游服务
        with open(file_path, "rb") as f:
            files = {'file': (filename, f, content_type)}
            # 假设下游服务也是一个接收文件的 POST 接口
            # response = await client.post(CALCULATION_SERVICE_URL, files=files)
            
            # --- 模拟下游服务返回 ---
            # response.raise_for_status()
            # result = response.json()
            
            # 模拟计算延迟
            await asyncio.sleep(1) 
            result = {
                "score": 0.98, 
                "tags": ["cat", "cute"], 
                "processed_at": time.time()
            }
            # -----------------------
    return result

@app.post("/calculate", response_model=CalculationResult)
async def calculate_image(
    background_tasks: BackgroundTasks,
    file: UploadFile = File(...)
):
    """
    接收图片 -> 缓存 -> 调用计算服务 -> 返回结果 -> 清理图片
    """
    # 1. 生成唯一文件名，防止冲突
    file_id = str(uuid.uuid4())
    file_extension = file.filename.split(".")[-1] if "." in file.filename else "jpg"
    temp_file_path = os.path.join(TEMP_DIR, f"{file_id}.{file_extension}")

    try:
        # 2. 缓存图片 (写入磁盘)
        # 如果图片很小，也可以直接在内存中处理，不写入磁盘
        with open(temp_file_path, "wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
        
        # 3. 添加后台清理任务 (例如 10秒后或请求结束后立即清理)
        # BackgroundTasks 会在响应发送后执行
        background_tasks.add_task(cleanup_file, temp_file_path)

        # 4. 调用核心计算逻辑
        result = await process_image_logic(temp_file_path, file.filename, file.content_type)

        return result

    except Exception as e:
        # 发生错误时也要确保清理（如果还没加入后台任务）
        cleanup_file(temp_file_path)
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    import asyncio
    uvicorn.run(app, host="0.0.0.0", port=8000)
