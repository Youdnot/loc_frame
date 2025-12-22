using System;
using System.Net.WebSockets;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using System.Text;

public class SlamClient : MonoBehaviour
{
    // 替换为实际的服务器IP地址
    private Uri _uri = new Uri("ws://127.0.0.1:8000/ws/slam"); 
    private ClientWebSocket _ws = new ClientWebSocket();
    private CancellationTokenSource _cts = new CancellationTokenSource();

    async void Start()
    {
        try 
        {
            await _ws.ConnectAsync(_uri, _cts.Token);
            Debug.Log("Connected to SLAM Server");
            
            // 开启接收循环
            _ = ReceiveLoop();
        }
        catch (Exception e)
        {
            Debug.LogError($"Connection failed: {e.Message}");
        }
    }

    // 示例：发送图像数据的方法
    // 应在 Update 中或协程中调用，传入编码后的图像数据 (如 JPG/PNG 字节流)
    public async void SendImage(byte[] imageData)
    {
        if (_ws.State == WebSocketState.Open)
        {
            try
            {
                // 发送二进制图像数据
                await _ws.SendAsync(new ArraySegment<byte>(imageData), WebSocketMessageType.Binary, true, _cts.Token);
            }
            catch (Exception e)
            {
                Debug.LogError($"Send failed: {e.Message}");
            }
        }
    }

    private async Task ReceiveLoop()
    {
        var buffer = new byte[1024 * 4]; // 根据预期响应大小调整缓冲区
        
        while (_ws.State == WebSocketState.Open && !_cts.IsCancellationRequested)
        {
            try
            {
                var result = await _ws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
                
                if (result.MessageType == WebSocketMessageType.Close)
                {
                    await _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Server closed", _cts.Token);
                    break;
                }

                string jsonResponse = Encoding.UTF8.GetString(buffer, 0, result.Count);
                
                // 在主线程处理数据 (Unity API 限制)
                // 注意：如果不在主线程，需使用 Dispatcher 或类似机制
                ApplyCorrection(jsonResponse);
            }
            catch (Exception e)
            {
                Debug.LogError($"Receive error: {e.Message}");
                break;
            }
        }
    }
    
    void ApplyCorrection(string json) 
    {
        // TODO: 解析 JSON 并应用位姿矫正
        // 例如使用 JsonUtility.FromJson<LocalizationResponse>(json);
        Debug.Log("Received Pose: " + json);
    }
    
    private async void OnDestroy()
    {
        _cts.Cancel();
        if (_ws != null)
        {
            if (_ws.State == WebSocketState.Open)
            {
                await _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client closing", CancellationToken.None);
            }
            _ws.Dispose();
        }
    }
}
