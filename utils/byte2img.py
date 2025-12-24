# Transform image bytes from Unity to standard OpenCV format

import numpy as np
import cv2

def decode_bytes_to_cv2(image_bytes):
    """
    使用 OpenCV
    特点：直接解码为 Numpy 数组 (BGR格式)
    """
    try:
        # 1. 将 bytes 转换为 numpy 的 uint8 数组
        # 这一步非常快，只是内存视图的转换
        nparr = np.frombuffer(image_bytes, np.uint8)
        
        # 2. 解码图像
        # cv2.IMREAD_COLOR: 加载彩色图像
        # cv2.IMREAD_GRAYSCALE: 如果 SLAM 只需要灰度图，用这个更快
        img_cv2 = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        return img_cv2
    except Exception as e:
        print(f"OpenCV Decode Error: {e}")
        return None

# 模拟接收数据的测试代码
if __name__ == "__main__":
    # 假设这是从 WebSocket 收到的 bytes
    # dummy_bytes = b'\xff\xd8\xff...' 
    
    print("Functions defined. Import this module to use.")
