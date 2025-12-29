import cv2
import rospy
import rosbag
import numpy as np
import argparse
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def resize_and_undistort_video(input_video, output_bag):
    # Initialize ROS node
    rospy.init_node('video_to_bag', anonymous=True)

    # Open the video file
    cap = cv2.VideoCapture(input_video)
    if not cap.isOpened():
        print(f"Error: Cannot open video file {input_video}")
        return

    # Define camera intrinsics and distortion parameters
    #ego
    # intrinsics = [454.3179409196489, 445.35504621834169, 955.665085098021, 525.8623630891543]
    # distortion = [0.45996459183088175, -0.29433449292297, 0.3378144594860423, -0.0964167774458528]
    
    # intrinsics = [4.4709283171100026e+02, 4.4303948367368923e+02, 9.5257419347737459e+02, 5.4843441122562808e+02]
    # distortion = [4.7300626570950011e-01, -3.5491409990349976e-01, 4.3615617780150950e-01, -1.0955799042596553e-01]

    #third
    # intrinsics = [1506.336181640625 * 2688 / 4096, 1506.336181640625 * 2016 / 3072, 1344.0, 1008.0]
    # # intrinsics = [1506.336181640625 * 4096 / 4096, 1506.336181640625 * 3072 / 3072, 2048.0, 1536.0]
    # distortion = [0.2498999983072281, 0.013609999790787697, -0.06207999959588051, 0.012190000154078007]

    # K = np.array([[intrinsics[0], 0, intrinsics[2]], [0, intrinsics[1], intrinsics[3]], [0, 0, 1]])
    # new_K = np.array([[376.58404541015625, 0, 512.0],
    #                   [0, 376.58404541015625, 384.0],
    #                   [0, 0, 1]])
    # D = np.array(distortion)

    # Create cv_bridge object
    bridge = CvBridge()

    # Open a ROS bag file for writing
    bag = rosbag.Bag(output_bag, 'w')

    try:
        frame_id = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Resize to 1024x768
            resized_frame = cv2.resize(frame, (1024, 768))

            # Undistort and resize?
            h, w = resized_frame.shape[:2]
            print(h,w)
            # new_K = new_K 
            # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=1)
            #print(new_K)
            #undistorted_frame = cv2.fisheye.undistortImage(resized_frame, K, D, None, Knew=new_K)
            #undistorted_frame = cv2.resize(undistorted_frame, (720, 720))
            # new_K, roi = cv2.getOptimalNewCameraMatrix(K, D[:4], (w, h), 1, (w, h))
            # print(new_K)
        
            # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
            # undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            # Convert frames to ROS CompressedImage messages
            #resized_image_msg = CompressedImage()
            #resized_image_msg.header.stamp = rospy.Time.from_sec(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
            #resized_image_msg.format = "jpeg"
            #resized_image_msg.data = np.array(cv2.imencode('.jpg', resized_frame)[1]).tobytes()

            undistorted_image_msg = CompressedImage()
            undistorted_image_msg.header.stamp = rospy.Time.from_sec(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
            # undistorted_image_msg.format = "jpeg"
            undistorted_image_msg.data = np.array(cv2.imencode('.jpg', resized_frame)[1]).tobytes()

            # Write messages to bag
            #bag.write('/camera/resized/compressed', resized_image_msg, t=resized_image_msg.header.stamp)
            bag.write('/camera/undistorted/compressed', undistorted_image_msg, t=undistorted_image_msg.header.stamp)

            frame_id += 1
            if frame_id % 50 == 0:
                print(f"Processed {frame_id} frames")

    finally:
        cap.release()
        bag.close()
        print(f"Finished writing {frame_id} frames to {output_bag}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Resize and undistort video and save to ROS bag file.')
    parser.add_argument('input_video', type=str, help='Path to the input video file')
    parser.add_argument('output_bag', type=str, help='Path to the output ROS bag file')
    args = parser.parse_args()

    resize_and_undistort_video(args.input_video, args.output_bag)
