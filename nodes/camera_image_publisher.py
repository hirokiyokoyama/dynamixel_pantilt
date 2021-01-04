#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
import cv_bridge

class CameraImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_image_publisher')
        
        # Start publisher
        self.pub = self.create_publisher(Image, '/image', 1)
        self.cv_bridge = cv_bridge.CvBridge()

    def spin(self):
        cap = cv2.VideoCapture(0)
        
        while rclpy.ok():
            ret, img = cap.read()
            if not ret:
                self.get_logger().fatal('Cannot capture camera image.')
                break
            msg = self.cv_bridge.cv2_to_imgmsg(img)
            self.joint_states_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraImagePublisher()
        node.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
