# Loc-Frame

## Overview

Loc-Frame is a client-server framework for real-time localization using head-mounted device (HMD) imagery and IMU data. The system uses a multi-container architecture with FastAPI and WebSocket for real-time communication between Unity HMD client and ROS-based localization backend.

## Table of Contents

- [Architecture](#architecture)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Technical Implementation](#technical-implementation)
- [Development](#development)
- [Future Improvements](#future-improvements)

## Architecture

Loc-Frame follows a **three-component client-server architecture** designed for real-time performance:

### 1. Unity HMD Client
- **Role**: Data acquisition and pose application
- **Technology**: C#, Unity, NativeWebSocket
- **Functionality**:
  - Captures images from HMD cameras
  - Collects IMU data and timestamps
  - Compresses images (JPG) for efficient transmission
  - Establishes WebSocket connection with server
  - Applies pose corrections from localization backend

### 2. WebSocket Server (Container 1)
- **Role**: Real-time communication middleware
- **Technology**: Python, FastAPI, WebSocket, aiohttp
- **Functionality**:
  - Manages WebSocket connections with Unity clients
  - Receives compressed image data
  - Forwards images to ROS localization backend via HTTP API
  - Relays pose estimation results back to Unity
  - Handles client connection management

### 3. ROS Localization Backend (Container 2)
- **Role**: Pose estimation engine
- **Technology**: Python, FastAPI, OpenCV, ROS
- **Functionality**:
  - Provides HTTP API endpoint for image processing
  - Decodes and processes incoming images
  - Runs pose estimation algorithms
  - Returns pose data in JSON format

### Data Flow

```
Unity HMD → WebSocket → Container 1 → HTTP POST → Container 2 (ROS)
Container 2 → HTTP Response → Container 1 → WebSocket → Unity HMD
```

## Project Structure

```
├── Unity/              # Unity HMD client implementation
│   ├── Connection.cs   # WebSocket connection manager
│   ├── UnityClient.cs  # Main client logic
│   └── README.md       # Unity client documentation
├── container/          # WebSocket server container
│   ├── Dockerfile      # Container configuration
│   ├── server-v2.py    # Main WebSocket server
│   └── README.md       # Server documentation
├── ros/                # ROS localization backend
│   ├── app.py          # HTTP API for pose estimation
│   └── localization/   # Pose estimation algorithms
├── utils/              # Utility functions
│   └── byte2img.py     # Image decoding utilities
├── main.py             # Sample server (for reference)
└── README.md           # This file
```

## Quick Start

### Prerequisites

- Docker
- Unity 2020.3+ (for client development)
- Python 3.10+ (for development)
- ROS 2 (for localization backend development)

### Build and Run Docker Containers

#### Container 1: WebSocket Server
```shell
cd container/
docker build -t loc-frame-websocket .
docker run -p 8000:8000 --name loc-websocket loc-frame-websocket
```

#### Container 2: ROS Localization Backend
```shell
cd ros/
docker build -t loc-frame-ros .
docker run -p 8001:8000 --name loc-ros --network="host" loc-frame-ros
```

### Unity Client Setup

1. Open the Unity project in the `Unity/` directory
2. Update the WebSocket server URI in `UnityClient.cs`:
   ```csharp
   [SerializeField] private string _uri = "ws://<server-ip>:8000/ws";
   ```
3. Build and deploy to your HMD device
4. Run the application on the HMD

### Testing the System

1. Start both Docker containers
2. Run the Unity application on the HMD
3. Verify WebSocket connection in server logs
4. Monitor pose estimation results in Unity console

## Technical Implementation

### Image Processing Pipeline

1. **Image Acquisition (Unity)**
   - Capture frames from HMD cameras using SDK-specific APIs
   - Downscale resolution if high precision is not required
   - Read pixels from GPU to Texture2D object

2. **Encoding and Compression**
   - Convert Texture2D to byte array using Texture2D.EncodeToJPG()
   - Use background threading to prevent frame drops
   - JPG quality set to 90 for optimal balance of size and feature preservation

3. **WebSocket Transmission**
   - Send data as binary messages (byte[]) instead of Base64 strings
   - Optional: Add fixed-size byte prefix for timestamp and image ID
   - Asynchronous transmission to avoid blocking main thread

4. **Backend Processing**
   - Convert received bytes to OpenCV-compatible format
   - Forward to ROS localization backend via HTTP API
   - Process pose estimation results
   - Send results back to Unity client

### Performance Considerations

- **Asynchronous Processing**: Non-blocking I/O throughout the pipeline
- **Connection Management**: Efficient handling of multiple client connections
- **Error Handling**: Graceful handling of disconnections and processing errors
- **Resource Optimization**: Reuse HTTP client sessions to minimize resource consumption

## Development

### Python Development

```shell
# Install dependencies
pip install -e .

# Run WebSocket server for testing
python container/server-v2.py

# Run ROS backend for testing
python ros/app.py
```

### Unity Development

- Use Unity Editor's Play mode for testing
- Monitor WebSocket connection status in Console
- Test image capture and transmission functionality

## Future Improvements

- [ ] Add authentication for WebSocket connections
- [ ] Implement frame rate optimization for low-bandwidth scenarios
- [ ] Add support for multiple camera streams
- [ ] Implement sensor fusion between IMU and visual localization
- [ ] Add monitoring and logging dashboard
- [ ] Support for both ROS 1 and ROS 2 backends
- [ ] Container orchestration with Docker Compose
- [ ] Test image restore and manipulate
- [ ] Test send back pose/info to unity
- [ ] Retrieve continuous pose of HMD