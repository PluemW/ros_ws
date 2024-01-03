import rclpy
import numpy as np
import torch
import cv2
import os

from ultralytics import YOLO
from supervision import Detections, BoxAnnotator
from rclpy.node import Node
from std_msgs.msg import Int8, Float32, String
from geometry_msgs.msg import Twist
from rclpy import qos
from cv_bridge import CvBridge


class BotBucketModel(Node):
    def __init__(self):
        super().__init__("bucket_model_node")
        self.sent_detect_gripper = self.create_publisher(
            String, "bucket/detect", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        msg_detect = String()
        msg_detect.data = "ma"
        self.sent_detect_gripper.publish(msg_detect)

def main():
    rclpy.init()

    sub = BotBucketModel()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
