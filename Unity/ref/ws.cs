// Example WebSocket connection in Unity using NativeWebSocket package
// See https://github.com/endel/NativeWebSocket
// Add comments for understanding

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using NativeWebSocket;

public class Connection : MonoBehaviour
{
  WebSocket websocket;

  // Start is called before the first frame update
  async void Start()
  {
    websocket = new WebSocket("ws://localhost:3000");
    // 订阅 OnOpen 事件：当 WebSocket 连接成功建立时触发
    // 1. "+=" 运算符：用于订阅事件。表示“当事件发生时，执行右侧的函数”。
    // 2. "() => { ... }"：Lambda 表达式（匿名函数）。
    //    - "()"：参数列表。这里为空，因为 OnOpen 事件不需要参数。
    //    - "=>"：Lambda 运算符，将参数传递给右侧的代码块。
    //    - "{ ... }"：函数体。事件触发时要执行的具体代码。
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
      Debug.Log("OnMessage!");
      Debug.Log(bytes);

      // getting the message as a string
      // var message = System.Text.Encoding.UTF8.GetString(bytes);
      // Debug.Log("OnMessage! " + message);
    };

    // Keep sending messages at every 0.3s
    InvokeRepeating("SendWebSocketMessage", 0.0f, 0.3f);

    // 发起连接
    // "await" 关键字：表示异步等待。
    // - 作用：发起连接请求，但不会卡死游戏画面（非阻塞）。
    // - 流程：Unity 会暂时跳出 Start 函数去处理下一帧画面，直到连接完成（成功或失败）后，
    //        才会回到这里继续执行后面的代码（如果有的话）。
    await websocket.Connect();
  }

  void Update()
  {
    // 预处理指令：只在 非WebGL平台 或 Unity编辑器模式 下编译这段代码
    // WebGL 平台由浏览器底层托管消息循环，不需要手动分发
    #if !UNITY_WEBGL || UNITY_EDITOR
      // 消息分发器
      // 作用：将后台网络线程收到的消息，取出并“分发”到 Unity 的主线程。
      // 必须这样做，因为 Unity 的组件（如 UI、Transform）只能在主线程操作。
      // 如果不写这行，OnMessage 等事件可能永远不会触发，或者触发时无法操作游戏对象。
      websocket.DispatchMessageQueue();
    #endif
  }

  // 发送消息的函数（被 InvokeRepeating 定时调用）
  async void SendWebSocketMessage()
  {
    // 检查连接状态：只有在连接开启（Open）时才发送，避免报错
    if (websocket.State == WebSocketState.Open)
    {
      // 发送二进制数据 (byte[])
      // 适用于传输图片、文件或自定义的高效协议
      await websocket.Send(new byte[] { 10, 20, 30 });

      // 发送纯文本消息 (string)
      // 适用于传输 JSON 字符串或普通命令
      await websocket.SendText("plain text message");
    }
  }

  // Unity 生命周期事件：当游戏/应用退出时自动调用
  private async void OnApplicationQuit()
  {
    // 优雅关闭连接
    // 告诉服务器“我要下线了”，而不是直接断网，有助于服务器清理资源
    await websocket.Close();
  }

}