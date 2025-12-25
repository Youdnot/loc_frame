// Custom Unity WebSocket client for SLAM server communication

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;

public class LocConnection : MonoBehaviour
{   
    WebSocket websocket;

    // 服务器IP地址
    [SerializeField] private string _uri = "ws://127.0.0.1:8000/ws";

    // Image date
    byte[] imgByte;
    public int width=1920;        
    public int height=1080; 

    async void Start()
    {
        websocket = new WebSocket(_uri);

        websocket.OnOpen += () =>
        {
            Debug.Log("WebSocket Connection Opened");
        };

        websocket.OnError += (e) =>
        {
            Debug.LogError("WebSocket Error: " + e);
        };

        websocket.OnClose += (e) =>
        {
            Debug.Log("WebSocket Connection Closed");
        };

        websocket.OnMessage += (bytes) =>
        {
            Debug.Log("WebSocket Message Received");
            string jsonResponse = System.Text.Encoding.UTF8.GetString(bytes);
            ApplyCorrection(jsonResponse);
        };

        await websocket.Connect();
    }

    // 用于处理图像的纹理缓存
    private Texture2D _processTexture;

    public async void SendImage(byte[] imageData)
    {
        if (websocket == null || websocket.State != WebSocketState.Open) return;

        try
        {
            // 1. 懒加载初始化纹理 (Lazy Init)
            if (_processTexture == null)
            {
                _processTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
            }

            // 2. 加载原始数据并压缩 (Raw -> JPG)
            // 注意：LoadRawTextureData 要求数组长度必须匹配 width * height * 4
            _processTexture.LoadRawTextureData(imageData);
            _processTexture.Apply();
            
            // SLAM 优化：使用 90 质量。
            // 90 是体积与画质的最佳平衡点。低于 80 会产生边缘噪点，干扰特征点提取；
            // 高于 95 体积剧增但对机器视觉提升不明显。
            byte[] jpgBytes = _processTexture.EncodeToJPG(90);

            // 3. 发送二进制数据
            await websocket.Send(jpgBytes);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"SendImage Error: {e.Message}");
        }
    }

    void Update()
    {
        // Dispatch message queue to invoke callbacks on the main thread.
        // This is required for native platforms; WebGL handles this automatically.
        #if !UNITY_WEBGL || UNITY_EDITOR
            websocket.DispatchMessageQueue();
        #endif


        

    }

    private async void OnApplicationQuit()
    {
        await websocket.Close();
    }

}
