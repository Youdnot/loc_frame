import cv2
import os

def images_to_video(input_dir, output_path, fps=30):
    # 获取目录下所有图片文件
    images = [img for img in os.listdir(input_dir) if img.lower().endswith((".png", ".jpg", ".jpeg"))]
    # 按文件名排序，确保顺序正确
    images.sort()

    if not images:
        print(f"错误: 在 {input_dir} 未找到图片")
        return

    # 读取第一张图片获取尺寸
    first_image_path = os.path.join(input_dir, images[0])
    frame = cv2.imread(first_image_path)
    height, width, layers = frame.shape

    # 定义编码器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    print(f"正在处理: {input_dir} -> {output_path}")
    print(f"视频分辨率: {width}x{height}, FPS: {fps}")

    count = 0
    for image in images:
        image_path = os.path.join(input_dir, image)
        frame = cv2.imread(image_path)
        
        # 确保所有图片尺寸一致，如果不一致可能需要 resize
        if frame.shape[0] != height or frame.shape[1] != width:
            frame = cv2.resize(frame, (width, height))

        out.write(frame)
        count += 1
        if count % 10 == 0:
            print(f"已处理 {count} 帧...", end='\r')

    out.release()
    print(f"\n完成! 总帧数: {count}")

if __name__ == "__main__":
    # --- 在这里修改您的文件名 ---
    input_directory = "temp_images"   # 输入图片目录
    output_file = "output_video.mp4"  # 输出视频文件名
    
    # 检查目录是否存在
    if not os.path.exists(input_directory):
        os.makedirs(input_directory)
        print(f"创建了目录: {input_directory}，请放入图片后运行")
    else:
        images_to_video(input_directory, output_file, fps=30)