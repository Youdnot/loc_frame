// Custom Unity WebSocket client for SLAM server communication

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;

public class LocConnection : MonoBehaviour
{   
    WebSocket websocket;

    // 服务器IP地址
    [SerializeField] private string _uri = "ws://127.0.0.1:8000/ws/loc/";

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

    public async void SendImage(byte[] imageData)
    {

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
