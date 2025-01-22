import pyrealsense2 as rs
from rclpy.node import Node
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
        #TODO: review if we should change to image_rect_raw/compressed (?)
        self.path_planner = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.path_planner_callback, 10)
    
    def path_planner_callback(self, image):
        try:
            self.get_logger().info(image)
        except Exception as e:
            self.get_logger().error(f"Error in callback")


def main():
    rclpy.init(args = None)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    rclpy.shutdown()

if __name__ == "__main__":
    main()