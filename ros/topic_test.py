#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import threading
import sys
import os
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from datetime import datetime
from dataclasses import dataclass, asdict
import json

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass
class ServerResponse:
    timestamp: str
    position: Vector3
    rotation: Quaternion

class TopicTester:
    def __init__(self, image_folder, input_topic="/camera/undistorted/compressed", output_topic="/tf"):
        # Initialize ROS node
        rospy.init_node('topic_tester', anonymous=True)
        
        self.image_folder = image_folder
        self.input_topic = input_topic
        self.output_topic = output_topic
        
        # Get list of images
        valid_extensions = ('.jpg', '.jpeg', '.png', '.bmp')
        if not os.path.exists(image_folder):
             rospy.logerr(f"Folder not found: {image_folder}")
             sys.exit(1)

        self.image_files = sorted([
            f for f in os.listdir(image_folder) 
            if f.lower().endswith(valid_extensions)
        ])
        
        if not self.image_files:
            rospy.logerr(f"No images found in: {image_folder}")
            sys.exit(1)
            
        rospy.loginfo(f"Found {len(self.image_files)} images in {image_folder}")
        
        # Publisher for the image
        self.image_pub = rospy.Publisher(self.input_topic, CompressedImage, queue_size=1)
        
        # Subscriber for the pose (TF)
        self.tf_sub = rospy.Subscriber(self.output_topic, TFMessage, self.tf_callback)
        
        self.last_pub_time = None
        self.lock = threading.Lock()

    def publish_image(self, image_file):
        """Generates and publishes the loaded image."""
        image_path = os.path.join(self.image_folder, image_file)
        img = cv2.imread(image_path)
        
        if img is None:
            rospy.logerr(f"Failed to load image: {image_path}")
            return

        # Resize to 1024x768 to match video2bag2.py
        if img.shape[1] != 1024 or img.shape[0] != 768:
            resized_img = cv2.resize(img, (1024, 768))
        else:
            resized_img = img

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
        rospy.loginfo(f"Published image {image_file} with timestamp: {msg.header.stamp.to_sec()}")

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
                # Convert timestamp to datetime
                ts = transform.header.stamp.to_sec()
                dt = datetime.fromtimestamp(ts)
                
                # Extract Position
                trans = transform.transform.translation
                pos = Vector3(x=trans.x, y=trans.y, z=trans.z)
                
                # Extract Rotation
                rot = transform.transform.rotation
                quat = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)
                
                # Create Response Object
                response = ServerResponse(
                    timestamp=dt.isoformat(),
                    position=pos,
                    rotation=quat
                )
                
                rospy.loginfo(f"Received Pose (Structured):")
                print(json.dumps(asdict(response), indent=2))

    def run(self):
        # Wait a bit for connections to establish
        time.sleep(1.0)
        
        rate = rospy.Rate(0.5) # Publish every 2 seconds
        
        for image_file in self.image_files:
            if rospy.is_shutdown():
                break
            self.publish_image(image_file)
            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python topic_test.py <image_folder>")
        sys.exit(1)

    image_folder = sys.argv[1]

    try:
        tester = TopicTester(image_folder)
        tester.run()
    except rospy.ROSInterruptException:
        pass