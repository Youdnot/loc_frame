# Edge SLAM Localization Service

这是一个基于 FastAPI 构建的端侧 SLAM（Simultaneous Localization and Mapping）定位服务示例。它旨在接收客户端（如 Unity 头显端、机器人）传输的图像数据，并返回计算出的位姿（Pose）和矫正矩阵。

## 功能特性

*   **高性能 Web 框架**: 基于 FastAPI 和 Uvicorn。
*   **双模通信**:
    *   **HTTP (REST)**: 适用于单次请求、状态检查或低频定位。
    *   **WebSocket**: 适用于高频、低延迟的实时视频流定位与矫正。
*   **Docker 支持**: 提供完整的 Dockerfile 和 docker-compose 配置，便于部署。
*   **模拟算法**: 内置模拟的 SLAM 处理逻辑，返回随机位姿和固定矫正矩阵用于测试。

## 快速开始

### 本地运行

1.  **安装依赖**:
    ```bash
    pip install -r requirements.txt
    ```

2.  **启动服务**:
    ```bash
    python main.py
    ```
    服务将运行在 `http://0.0.0.0:8000`。

### Docker 运行

1.  **构建并启动**:
    ```bash
    docker-compose up --build -d
    ```

2.  **查看日志**:
    ```bash
    docker-compose logs -f
    ```

## API 文档

### 1. HTTP 定位接口

*   **URL**: `/slam/localize`
*   **Method**: `POST`
*   **Content-Type**: `multipart/form-data`
*   **参数**: `file` (图像文件)
*   **返回**: JSON 格式的定位信息 (Pose, Correction Matrix, etc.)

### 2. WebSocket 实时接口

*   **URL**: `/ws/slam`
*   **协议**: WebSocket
*   **流程**:
    1.  客户端建立连接。
    2.  客户端发送二进制图像数据 (`byte[]`)。
    3.  服务端实时返回 JSON 格式定位结果。

## 客户端示例 (Unity/C#)

本项目包含一个 Unity 客户端脚本示例 `UnityClient.cs`，展示了如何通过 WebSocket 连接服务并发送图像数据。

### 关键代码片段

```csharp
// 连接 WebSocket
await _ws.ConnectAsync(_uri, _cts.Token);

// 发送图像 (二进制)
await _ws.SendAsync(new ArraySegment<byte>(imageData), WebSocketMessageType.Binary, true, _cts.Token);

// 接收结果
var result = await _ws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
string jsonResponse = Encoding.UTF8.GetString(buffer, 0, result.Count);
```

详细代码请参考项目根目录下的 [UnityClient.cs](./UnityClient.cs)。

## 项目结构

```
.
├── Dockerfile              # Docker 构建文件
├── README.md               # 项目说明文档
├── UnityClient.cs          # Unity C# 客户端参考代码
├── docker-compose.yml      # Docker Compose 配置
├── main.py                 # FastAPI 服务主入口
├── pyproject.toml          # 项目配置
└── requirements.txt        # Python 依赖列表
```
