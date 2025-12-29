#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import cv2
import numpy as np
import threading
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

# Configuration
INPUT_TOPIC = "/camera/undistorted/compressed" 
OUTPUT_TOPIC = "/tf"

class TopicTester:
    def __init__(self, input_topic, output_topic, callback=None):
        """
        Initialize the TopicTester.
        :param input_topic: ROS topic to publish images to.
        :param output_topic: ROS topic to subscribe to for results.
        :param callback: Function to call when a valid result is received.
        """
        self.image_pub = rospy.Publisher(input_topic, CompressedImage, queue_size=1)
        self.tf_sub = rospy.Subscriber(output_topic, TFMessage, self.tf_callback)
        self.callback = callback
        self.last_pub_time = None
        self.lock = threading.Lock()

    def publish_frame(self, img):
        """
        Resizes, compresses, and publishes a single image frame.
        :param img: cv2 image (numpy array)
        """
        if img is None:
            return

        # Resize to 1024x768 (standardize input)
        if img.shape[1] != 1024 or img.shape[0] != 768:
            img = cv2.resize(img, (1024, 768))

        # Compress to JPEG
        success, encoded_img = cv2.imencode('.jpg', img)
        if not success:
            rospy.logerr("Failed to encode image!")
            return

        # Create and fill message
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(encoded_img).tobytes()

        # Update timestamp safely
        with self.lock:
            self.last_pub_time = msg.header.stamp
            
        self.image_pub.publish(msg)
        rospy.logdebug(f"Published image at {msg.header.stamp.to_sec()}")

    def tf_callback(self, msg):
        """
        Callback for TF messages. Checks if the transform is newer than the last published image.
        """
        with self.lock:
            if self.last_pub_time is None:
                return
            current_pub_time = self.last_pub_time

        for transform in msg.transforms:
            # Check if response corresponds to the latest request (or newer)
            if transform.header.stamp > current_pub_time:
                if self.callback:
                    self.callback(transform)
                else:
                    # Default logging if no callback provided
                    rospy.loginfo(f"Received Pose: {transform.transform.translation}")

def main():
    rospy.init_node('topic_tester', anonymous=True)

    if len(sys.argv) < 2:
        print("Usage: python topic_test.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]
    img = cv2.imread(image_path)
    if img is None:
        rospy.logerr(f"Failed to load image: {image_path}")
        sys.exit(1)

    # Define a callback to handle the immediate return of information
    def on_pose_received(transform):
        # This function is called immediately when data is received
        rospy.loginfo(f"Frame ID: {transform.header.frame_id}, Translation: {transform.transform.translation}")

    # Initialize tester with callback
    tester = TopicTester(INPUT_TOPIC, OUTPUT_TOPIC, callback=on_pose_received)

    # Simulate streaming loop
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        tester.publish_frame(img)
        rate.sleep()

if __name__ == "__main__":
    main()