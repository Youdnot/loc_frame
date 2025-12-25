# Todos

- [ ] test image restore and manipulate
- [ ] test send back pose/info to unity
    - and use the info to action
- [ ] retrieve continuous pose of HMD

# Intro

A framework for image transision and pose retrival.

Using FastAPI, uv

## Files

- Dockerfile
    - reproduction of environment


# Workflow

This guide outlines the integration of Unity, WebSocket communication, and ROS within a containerized Python environment.

## Overview

The pipeline captures raw sensor data from an HMD, compresses it for network efficiency, transmits it to a remote server for AI/Robotics processing (ROS), and optionally returns metadata for Augmented Reality (AR) visualization.

## Architecture

| Component         | Role                    | Technology                         |
| ----------------- | ----------------------- | ---------------------------------- |
| **HMD / Unity**   | Source & Client         | C#, WebCamTexture, WebSocket-Sharp |
| **Network**       | Transport               | WebSocket (TCP) - Binary Frame     |
| **Python Server** | Middleware & Processing | websockets library, OpenCV, NumPy  |
| **ROS Container** | Robotic Integration     | ROS1/ROS2, Docker, cv_bridge       |

## Technical Implementation

### 1. Image Acquisition (Unity)

The HMD (Pico 4 Ultra Enterprise) captures frames through its onboard cameras.

- **Method**: SDK-specific APIs for camera access [相机数据 | PICO 开发者平台](https://developer-cn.picoxr.com/document/unity/p2j6gj2q/)
- **Texture Management:** Read the pixels from the GPU to a Texture2D object.
- **Optimization:** Downscale the resolution (e.g., 1280x720 to 640x480) at this stage if high precision is not required, reducing network load.

### 2. Encoding and Compression

Sending raw RGBA arrays is inefficient for wireless networks.

- **Encoding:** Convert the Texture2D to a byte array using Texture2D.EncodeToJPG(quality) or EncodeToPNG().
    
    - Note: JPG is recommended for high-frequency streaming due to smaller file sizes.
        
- **Threading:** Move the encoding process to a background thread or use AsyncGPUReadback to prevent frame drops in the Unity main UI thread.
    

### 3. WebSocket Transmission (Client)

Unity acts as a WebSocket client to establish a persistent, low-latency connection.

- **Data Type:** Send data as **Binary Messages** (byte[]) rather than Base64 strings. Base64 increases data size by approximately 33%.
    
- **Protocol:**
    
    - **Header (Optional):** Attach a small JSON header or a fixed-size byte prefix (e.g., 4 bytes for timestamp, 4 bytes for image ID).
        
    - **Payload:** The compressed image bytes.
        

### 4. Python Backend & ROS Integration

The Python server acts as the bridge between the network and the ROS ecosystem.

#### A. Data Decoding

- **Library:** Use websockets (asyncio) for the server.
    
- **Processing:** Receive the byte[] and convert it into an OpenCV-compatible format:
    
    codePython
    
    ```
    import cv2
    import numpy as np
    
    # Receive binary data from websocket
    nparr = np.frombuffer(binary_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    ```
    

#### B. ROS Publishing (Inside Container)

To make the image available to other ROS nodes:

1. Initialize a ROS node within the Python script.
    
2. Use cv_bridge to convert the OpenCV image to a sensor_msgs/Image message.
    
3. Publish to a topic (e.g., /hmd/camera_frames).
    

### 5. Feedback Loop (Optional)

If the system requires visualization (e.g., bounding boxes or SLAM points) on the HMD:

- **Inference:** Python processes the image (YOLO, MediaPipe, etc.).
    
- **Serialized Results:** Send coordinates or classification results back to Unity via the same WebSocket as a JSON string.
    
- **Unity Rendering:** Parse the JSON and use the Canvas or LineRenderer to draw overlays on the user's view.