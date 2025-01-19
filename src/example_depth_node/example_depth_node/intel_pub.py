import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

class IntelPublisher(Node):
    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)

        timer_period = 0.05
        self.br_rgb = CvBridge #bridge converting between openCV format and sensor_msgs.msg image type

        try:
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipe.start(self.cfg)
        except Exception as e:
            print(e)
            self.get_logger().error("realsense is not connected")
    
    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())

        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.get_logger().info("publishing Depth frame")


        

def main():
    rclpy.init(args = None)
    intel_publisher = IntelPublisher()
    rclpy.spin(intel_publisher)
    intel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()