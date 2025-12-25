using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;

public class Connection : MonoBehaviour
{
    WebSocket websocket;

    // Server IP address
    [SerializeField] private string _uri = "ws://127.0.0.1:8000/ws";

    // Image processing texture cache
    private Texture2D _processTexture;

    // Setting target transform to apply received data
    public Transform targetTransform;

    // Image data
    byte[] imgByte;
    public int width = 1920;
    public int height = 1080;

    [System.Serializable]
    public class ServerResponse
    {
        public Vector3Data position;
        public QuaternionData rotation;
    }

    [System.Serializable]
    public class Vector3Data
    {
        public float x, y, z;
        // 转换辅助方法
        public Vector3 ToVector3() => new Vector3(x, y, z);
    }

    [System.Serializable]
    public class QuaternionData
    {
        public float x, y, z, w;
        // 转换辅助方法
        public Quaternion ToQuaternion() => new Quaternion(x, y, z, w);
    }

    // Start is called before the first frame update
    async void Start()
    {
        // websocket = new WebSocket("ws://echo.websocket.org");
        websocket = new WebSocket(_uri);

        websocket.OnOpen += () =>
        {
            Debug.Log("Connection open!");
        };

        websocket.OnError += (e) =>
        {
            Debug.Log("Error! " + e);
        };

        websocket.OnClose += (e) =>
        {
            Debug.Log("Connection closed!");
        };

        websocket.OnMessage += (bytes) =>
        {
            // Reading a plain text message
            Debug.Log("WebSocket Message Received");
            string jsonResponse = System.Text.Encoding.UTF8.GetString(bytes);
            ApplyCorrection(jsonResponse);
        };

        // Keep sending messages at every 0.3s
        // InvokeRepeating("SendWebSocketMessage", 0.0f, 0.3f);

        await websocket.Connect();
    }

    void Update()
    {
        #if !UNITY_WEBGL || UNITY_EDITOR
        websocket.DispatchMessageQueue();
        #endif

        if (Time.frameCount % 300 == 0) // Assuming 60fps, roughly every 5 seconds
        {
            if (websocket.State == WebSocketState.Open)
            {
                SendHeartbeat();
            }
        }

        if (UnityEngine.InputSystem.Keyboard.current != null && UnityEngine.InputSystem.Keyboard.current.iKey.wasPressedThisFrame)
        {
            SendImage();
        }
    }

    async void SendHeartbeat()
    {
        string timestamp = System.DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
        await websocket.SendText("Heartbeat: " + timestamp);
    }

    async void SendImage()
    {
        string filePath = System.IO.Path.Combine(Application.dataPath, "Images/data.png");
        if (System.IO.File.Exists(filePath))
        {
            byte[] imageData = System.IO.File.ReadAllBytes(filePath);
            if (websocket.State == WebSocketState.Open)
            {
                await websocket.Send(imageData);
                Debug.Log("Sent Image: " + filePath + ", size: " + imageData.Length);
            }
        }
        else
        {
            Debug.LogError("Image file not found at: " + filePath);
        }
    }

    // async void SendWebSocketMessage()
    // {
    //     if (websocket.State == WebSocketState.Open)
    //     {
    //         // Sending bytes
    //         await websocket.Send(new byte[] { 10, 20, 30 });
    //
    //         // Sending plain text
    //         await websocket.SendText("plain text message");
    //     }
    // }

    // public async void SendImage(byte[] imageData)
    // {
    //     if (websocket == null || websocket.State != WebSocketState.Open) return;
    //
    //     try
    //     {
    //         // 1. 懒加载初始化纹理 (Lazy Init)
    //         if (_processTexture == null)
    //         {
    //             _processTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
    //         }
    //
    //         // 2. 加载原始数据并压缩 (Raw -> JPG)
    //         // 注意：LoadRawTextureData 要求数组长度必须匹配 width * height * 4
    //         _processTexture.LoadRawTextureData(imageData);
    //         _processTexture.Apply();
    //
    //         // SLAM 优化：使用 90 质量。
    //         // 90 是体积与画质的最佳平衡点。低于 80 会产生边缘噪点，干扰特征点提取；
    //         // 高于 95 体积剧增但对机器视觉提升不明显。
    //         byte[] jpgBytes = _processTexture.EncodeToJPG(90);
    //
    //         // 3. 发送二进制数据
    //         await websocket.Send(jpgBytes);
    //     }
    //     catch (System.Exception e)
    //     {
    //         Debug.LogError($"SendImage Error: {e.Message}");
    //     }
    // }

    private void ApplyCorrection(string json)
    {
        if (targetTransform == null) return;

        try
        {
            // 解析 JSON
            ServerResponse data = JsonUtility.FromJson<ServerResponse>(json);

            if (data != null)
            {
                // 应用位置 (注意坐标系转换，Unity 是左手系，OpenCV/SLAM 通常是右手系)
                // 这里假设服务器已经处理好了坐标系，或者直接透传
                if (data.position != null)
                    targetTransform.position = data.position.ToVector3();

                if (data.rotation != null)
                    targetTransform.rotation = data.rotation.ToQuaternion();

                Debug.Log($"Updated Pose: {targetTransform.position}");
            }
        }
        catch (System.Exception e)
        {
            // 忽略非 JSON 消息（比如服务器发的纯文本 Log）
            // Debug.LogWarning($"JSON Parse Error: {e.Message}");
        }
    }

    private async void OnApplicationQuit()
    {
        await websocket.Close();
    }
}
