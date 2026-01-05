# Start with:
# fastapi run main.py
# to allow external connections

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from typing import Optional
import uuid
import json
import math
import time
from datetime import datetime
from dataclasses import dataclass, asdict

import numpy as np
import cv2

# ros related
import rospy
import threading
import sys
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

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

def decode_bytes_to_cv2(image_bytes: bytes) -> Optional[np.ndarray]:
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

import asyncio

class TopicTester:
    def __init__(self, input_topic="/camera/undistorted/compressed", output_topic="/tf"):
        # Initialize ROS node (ensure this is only called once)
        try:
            rospy.init_node('websocket_topic_tester', anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass # Node already initialized
        
        self.input_topic = input_topic
        self.output_topic = output_topic
        
        # Publisher for the image
        self.image_pub = rospy.Publisher(self.input_topic, CompressedImage, queue_size=1)
        
        # Subscriber for the pose (TF)
        self.tf_sub = rospy.Subscriber(self.output_topic, TFMessage, self.tf_callback)
        
        self.last_pub_time = None
        self.lock = threading.Lock()
        
        # Store pending requests: {timestamp_sec: asyncio.Future}
        self.pending_requests = {}

    def publish_image(self, img: np.ndarray) -> asyncio.Future:
        """Generates and publishes the image, returning a Future for the result."""
        
        # Resize to 1024x768 to match video2bag2.py
        if img.shape[1] != 1024 or img.shape[0] != 768:
            resized_img = cv2.resize(img, (1024, 768))
        else:
            resized_img = img

        # Compress to JPEG
        success, encoded_img = cv2.imencode('.jpg', resized_img)
        if not success:
            rospy.logerr("Failed to encode image!")
            return None

        # Create message
        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(encoded_img).tobytes()

        # Create a Future to store the result
        loop = asyncio.get_event_loop()
        future = loop.create_future()
        
        with self.lock:
            self.last_pub_time = msg.header.stamp
            # Store the future keyed by the timestamp
            self.pending_requests[msg.header.stamp.to_sec()] = future
            
        self.image_pub.publish(msg)
        rospy.loginfo(f"Published image with timestamp: {msg.header.stamp.to_sec()}")
        
        return future

    def tf_callback(self, msg):
        """Callback for TF messages."""
        # Iterate through transforms in the message
        for transform in msg.transforms:
            ts = transform.header.stamp.to_sec()
            
            # Check if we have a pending request for this timestamp (or close enough)
            # Note: In a real system, exact timestamp matching might be tricky due to float precision
            # or if TF stamps are slightly different. For now, we assume exact match or we need a better correlation ID.
            # A simple way is to check if the TF timestamp is *after* our last publish time.
            
            matched_future = None
            matched_ts = None
            
            with self.lock:
                # Find the request that this transform answers
                # Simple logic: if transform time >= request time, it's a candidate.
                # For strict matching, we might need to pass a frame_id or seq.
                # Here we use the exact timestamp logic from the original code:
                # "Check if the transform timestamp is after the image publish time"
                
                # We need to find which pending request this corresponds to.
                # Since TF messages might come in a batch, we iterate our pending requests.
                for req_ts, future in list(self.pending_requests.items()):
                    if ts > req_ts: # The transform is newer than the image
                        matched_future = future
                        matched_ts = req_ts
                        break
            
            if matched_future and not matched_future.done():
                # Convert timestamp to datetime
                dt = datetime.fromtimestamp(ts)
                
                # Extract Position
                trans = transform.transform.translation
                pos = Vector3(x=trans.x, y=trans.y, z=trans.z)
                
                # Extract Rotation
                rot = transform.transform.rotation
                quat = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)
                
                # Create Response Object
                response = ServerResponse(
                    timestamp=dt.isoformat(),
                    position=pos,
                    rotation=quat
                )
                
                # Set the result on the future (thread-safe way for asyncio)
                loop = matched_future.get_loop()
                loop.call_soon_threadsafe(matched_future.set_result, response)
                
                # Clean up
                with self.lock:
                    if matched_ts in self.pending_requests:
                        del self.pending_requests[matched_ts]

tester = TopicTester()

app = FastAPI(title="My Application", description="This is a sample FastAPI application.")

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
                
                if "bytes" in message:
                    bytes_data = message["bytes"]
                    print(f"Received bytes: {len(bytes_data)}")

                    # Decode and display the image for debugging
                    img = decode_bytes_to_cv2(bytes_data)

                    if img is None:
                        print("Error: Failed to decode image")
                        await websocket.send_text(json.dumps({"error": "Invalid image data", "details": "OpenCV decode failed"}))
                        continue
                    
                    img = cv2.rotate(img, cv2.ROTATE_180)
                    img = cv2.flip(img, 1)

                    # 调用 ROS TopicTester 发布图片并等待结果
                    future = tester.publish_image(img)

                    if future:
                        try:
                            # 等待结果，设置超时时间防止无限等待
                            response = await asyncio.wait_for(future, timeout=5.0)
                            await websocket.send_text(json.dumps(asdict(response)))
                        except asyncio.TimeoutError:
                            print("Error: Localization timeout")
                            await websocket.send_text(json.dumps({"error": "Localization timeout"}))
                    else:
                         await websocket.send_text(json.dumps({"error": "Failed to publish image"}))
                    
    except WebSocketDisconnect:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")
