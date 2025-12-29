#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import threading
import sys
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

# ================= Configuration =================
# Input Topic (Topic to publish images to)
INPUT_TOPIC = "/camera/undistorted/compressed" 
# Output Topic (Topic to subscribe for results)
OUTPUT_TOPIC = "/tf"

class TopicTester:
    def __init__(self, image_path):
        # Initialize ROS node
        rospy.init_node('topic_tester', anonymous=True)
        
        self.image_path = image_path
        self.img = cv2.imread(image_path)
        if self.img is None:
            rospy.logerr(f"Failed to load image from: {image_path}")
            sys.exit(1)
        
        # Publisher for the image
        self.image_pub = rospy.Publisher(INPUT_TOPIC, CompressedImage, queue_size=1)
        
        # Subscriber for the pose (TF)
        self.tf_sub = rospy.Subscriber(OUTPUT_TOPIC, TFMessage, self.tf_callback)
        
        self.last_pub_time = None
        self.lock = threading.Lock()

    def publish_image(self):
        """Generates and publishes the loaded image."""
        
        # Resize to 1024x768 to match video2bag2.py
        if self.img.shape[1] != 1024 or self.img.shape[0] != 768:
            resized_img = cv2.resize(self.img, (1024, 768))
        else:
            resized_img = self.img

        # Compress to JPEG
        success, encoded_img = cv2.imencode('.jpg', resized_img)
        if not success:
            rospy.logerr("Failed to encode image!")
            return

        # Create message
        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(encoded_img).tobytes()

        with self.lock:
            self.last_pub_time = msg.header.stamp
            
        self.image_pub.publish(msg)
        rospy.loginfo(f"Published image with timestamp: {msg.header.stamp.to_sec()}")

    def tf_callback(self, msg):
        """Callback for TF messages."""
        with self.lock:
            if self.last_pub_time is None:
                return
            
            current_pub_time = self.last_pub_time

        # Iterate through transforms in the message
        for transform in msg.transforms:
            # Check if the transform timestamp is after the image publish time
            if transform.header.stamp > current_pub_time:
                rospy.loginfo(f"Received Pose (TF) with timestamp: {transform.header.stamp.to_sec()}")
                rospy.loginfo(f"Frame: {transform.header.frame_id} -> {transform.child_frame_id}")
                rospy.loginfo(f"Translation: {transform.transform.translation}")
                rospy.loginfo(f"Rotation: {transform.transform.rotation}")

    def run(self):
        # Wait a bit for connections to establish
        time.sleep(1.0)
        
        rate = rospy.Rate(0.5) # Publish every 2 seconds
        while not rospy.is_shutdown():
            self.publish_image()
            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python topic_test.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]

    try:
        tester = TopicTester(image_path)
        tester.run()
    except rospy.ROSInterruptException:
        pass