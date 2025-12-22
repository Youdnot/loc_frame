from typing import Union

from fastapi import FastAPI

app = FastAPI(title="My Application", description="This is a sample FastAPI application.")

@app.get("/")
def read_root():
    return {"message": "Service is running."}

@app.post("/upload/image/")
def upload_image(image_data: bytes):