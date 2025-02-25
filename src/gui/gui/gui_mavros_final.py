import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel,
                             QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QGraphicsRectItem, 
                             QSpacerItem, QSizePolicy)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QPalette, QColor, QPainter, QImage
from PyQt5.QtCore import Qt, QRectF, QRect, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # For PWM sensor data (adjust if necessary)
from cv_bridge import CvBridge  # For converting ROS2 Image messages to OpenCV format
import cv2  # For OpenCV image processing
from sensor_msgs.msg import CompressedImage

from mavros_msgs.msg import State, OpticalFlow, StatusText
from sensor_msgs.msg import Imu, BatteryState, Range
from std_msgs.msg import Float64

import transforms3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Dimensions
margin = 10
camera_width, camera_height = 900, 600
label_height = 30
info_width, info_height = 300, 450

# Colours
text_colour = "#F0F1F1"
background_colour = "#353535"
window_colour = "#242424"

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage, 
            "/camera/camera/color/image_raw/compressed",
            self.camera_sub_callback, 10
        )
        self.signal = pyqtSignal(object)

    def camera_sub_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image {e}")
            return

        self.signal.emit(cv_image)

class MavrosSubscriberNode(Node):
    def __init__(self):
        super().__init__('mavros_subscriber')
        self.create_subscription(State, '/mavros/state', self.state_callback, qos_reliable)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_best_effort)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_best_effort)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_best_effort)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self.rangefinder_callback, qos_reliable)
        self.create_subscription(OpticalFlow, '/mavros/optical_flow/raw/optical_flow', self.optflow_callback, qos_reliable)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.error_callback, qos_best_effort)
        
        self.data = {
            "armed": "Unknown",
            "mode": "Unknown",
            "battery": "Unknown",
            "roll": "Unknown",
            "pitch": "Unknown",
            "yaw": "Unknown",
            "altitude": "Unknown",
            "rangefinder": "Unknown",
            "optflow": "Unknown",
            "error": "Unknown"
        }
        self.signal = pyqtSignal(dict)
    
    def state_callback(self, msg):
        self.data["armed"] = "Armed" if msg.armed else "Disarmed"
        self.data["mode"] = msg.mode
        self.signal.emit(self.data)

    def battery_callback(self, msg):
        self.data["battery"] = f"{msg.percentage * 100:.1f}%"
        self.signal.emit(self.data)

    def imu_callback(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)
        self.data["roll"] = f"{roll:.2f}"
        self.data["pitch"] = f"{pitch:.2f}"
        self.data["yaw"] = f"{yaw*100:.2f}"
        self.signal.emit(self.data)

    def altitude_callback(self, msg):
        self.data["altitude"] = f"{msg.data:.2f} m"
        self.signal.emit(self.data)
    
    def rangefinder_callback(self, msg):
        self.data["rangefinder"] = f"{msg.range:.2f} m"
        self.signal.emit(self.data)
    
    def optflow_callback(self, msg):
        self.data["optflow"] = f"Qual:{msg.quality:.2f}, Opt_x:{msg.flow_rate.x:.2f}, Opt_y:{msg.flow_rate.y:.2f}, "
        self.signal.emit(self.data)

    def error_callback(self, msg):
        print(msg)
        self.data["error"] = msg.text
        self.signal.emit(self.data)

class RosThread(QThread):
    fwd_cam_received = pyqtSignal(object)
    telem_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.cam_node = CameraSubscriberNode()
        self.cam_node.signal = self.fwd_cam_received

        self.telem_node = MavrosSubscriberNode()
        self.telem_node.signal = self.telem_received

    def run(self):
        # rclpy.spin(self.cam_node)
        rclpy.spin(self.telem_node)

    def stop(self):
        rclpy.shutdown()
        self.wait()

class Button(QLabel):
    """A label that acts as an indicator, toggling between two colors when its key is pressed."""

    def __init__(self, key, color_on="#F0F1F1", color_off="#353535", parent=None):
        super().__init__("", parent)

        self.key = key  # The key that toggles this indicator
        self.trigger = False  # Initial state
        self.color_on = color_on  # Color when "on"
        self.color_off = color_off  # Color when "off"

        self.setFixedSize(100, 100)  # Set indicator size
        self.update_indicator(False)

    def toggle(self):
        """Toggles the indicator color."""
        self.trigger = not self.trigger
        self.update_indicator(self.trigger)

    def update_indicator(self, is_on):
        """Updates the indicator based on whether the key is pressed."""
        color = self.color_on if is_on else self.color_off
        self.setStyleSheet(f"background-color: {color};")


class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        self.setWindowTitle("SAFMC GUI")
        self.setGeometry(QApplication.primaryScreen().geometry())

        self.ros_thread = RosThread()

        # Forward cam (SHAWN)
        self.label_fwd_cam = QLabel("Camera", self)
        self.label_fwd_cam.setFixedHeight(label_height)
        self.label_fwd_cam.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        self.label_fwd_cam.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        self.pic_fwd_cam = QLabel(self)
        self.pic_fwd_cam.setAlignment(Qt.AlignCenter)
        self.pic_fwd_cam.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.pixmap_cam = QPixmap("dwd_cam_fake.jpg")
        self.ros_thread.fwd_cam_received.connect(self.updateCam)

        # UAV Info
        self.label_UAV, self.info_UAV = self.createUAVInfo()
        self.ros_thread.telem_received.connect(self.updateUAVInfo)

        # Overview
        self.label_overview = QLabel("Overview", self)
        self.label_overview.setFixedHeight(label_height)
        self.label_overview.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        self.label_overview.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        self.pic_drone = QLabel(self)
        self.pic_drone.setStyleSheet("background-color: #242424")
        self.pic_drone.setAlignment(Qt.AlignCenter)
        self.pic_drone.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.pixmap_drone = QPixmap("drone_00.jpg")

        # Buttons
        self.L1_label = QLabel("Toggle Axis", self)
        self.L1_label.setAlignment(Qt.AlignCenter)
        self.L2_label = QLabel("Toggle Camera", self)
        self.L2_label.setAlignment(Qt.AlignCenter)
        self.L3_label = QLabel("L3", self)
        self.L3_label.setAlignment(Qt.AlignCenter)
        self.L4_label = QLabel("Takeoff / Land", self)
        self.L4_label.setAlignment(Qt.AlignCenter)
        self.R1_label = QLabel("Toggle L/R Screw", self)
        self.R1_label.setAlignment(Qt.AlignCenter)
        self.R2_label = QLabel("Receive Payload", self)
        self.R2_label.setAlignment(Qt.AlignCenter)
        self.R3_label = QLabel("Dropoff Payload", self)
        self.R3_label.setAlignment(Qt.AlignCenter)
        self.R4_label = QLabel("Set Zero", self)
        self.R4_label.setAlignment(Qt.AlignCenter)

        self.L1_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.L2_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.L3_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.L4_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R1_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R2_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R3_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        self.R4_label.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")

        self.holdIndicators = {
            Qt.Key_Q: Button(Qt.Key_Q, "#F0F1F1", "#464646", self),
            Qt.Key_W: Button(Qt.Key_W, "#F0F1F1", "#464646", self),
            Qt.Key_E: Button(Qt.Key_E, "#F0F1F1", "#464646", self),
            Qt.Key_R: Button(Qt.Key_R, "#F0F1F1", "#464646", self),
            Qt.Key_U: Button(Qt.Key_U, "#F0F1F1", "#464646", self),
            Qt.Key_I: Button(Qt.Key_I, "#F0F1F1", "#464646", self),
            Qt.Key_O: Button(Qt.Key_O, "#F0F1F1", "#464646", self),
            Qt.Key_P: Button(Qt.Key_P, "#F0F1F1", "#464646", self),
        }

        # General
        self.setStyleSheet("background-color: #353535;")

        # start background ROS thread
        self.ros_thread.start()
        self.initUI()
    
    def createUAVInfo(self):
        label_UAV = QLabel("    UAV Info", self)
        label_UAV.setGeometry(margin, 2 * margin + label_height + camera_height, info_width, label_height)
        label_UAV.setFixedSize(info_width, label_height)
        label_UAV.setStyleSheet("color: #F0F1F1;"
                                "background-color: #242424;")
        label_UAV.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.info_UAV = QVBoxLayout()
        self.labels = {
            "armed": QLabel("Armed: Unknown"),
            "mode": QLabel("Mode: Unknown"),
            "battery": QLabel("Battery: Unknown"),
            "roll": QLabel("Roll: Unknown"),
            "pitch": QLabel("Pitch: Unknown"),
            "yaw": QLabel("Yaw: Unknown"),
            "altitude": QLabel("Altitude: Unknown"),
            "rangefinder": QLabel("Rangefinder: Unknown"),
            "optflow": QLabel("Optflow: Unknown"),
            "error": QLabel("Error: Unknown")
        }
        row, column = 0, 0
        for label in self.labels.values():
            label.setStyleSheet("background-color: #242424;"
                                "color: #F0F1F1;")
            self.info_UAV.addWidget(label, row, column)
            column += 1
            if column == 3:
                row += 1
                column = 0
        
        return label_UAV, self.info_UAV

    def updateCam(self, cv_image):  # (SHAWN)
        height, width, channel = cv_image.shape
        bytes_per_line = channel * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.pixmap_cam = QPixmap.fromImage(q_image)
        self.pic_fwd_cam.setPixmap(self.pixmap_cam)

    def updateUAVInfo(self, data):
        for key, value in data.items():
            self.labels[key].setText(f"{key.capitalize()}: {value}")
            self.labels[key].setStyleSheet("color: #F0F1F1;"
                                           "background-color: #242424;")

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        grid1 = QGridLayout()
        grid1.setSpacing(0)

        grid2 = QGridLayout()
        grid2.setSpacing(0)

        vbox = QVBoxLayout()

        spacer = QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding)

        # create label objects here
        grid1.addWidget(self.label_fwd_cam, 0, 0)
        grid1.addItem(spacer, 0, 1)
        grid1.addWidget(self.label_dwd_cam, 0, 2)

        grid1.addWidget(self.pic_fwd_cam, 1, 0)
        grid1.addItem(spacer, 1, 1)
        grid1.addWidget(self.pic_dwd_cam, 1, 2)

        # grid.addWidget(self.label_spacer, 2, 0)  # Span across columns 0-2

        grid2.addWidget(self.label_UAV, 0, 0)
        grid2.addLayout(self.info_UAV, 1, 0)

        grid1.addItem(spacer, 0, 1)
        grid1.addItem(spacer, 1, 1)

        grid2.addWidget(self.label_payload, 0, 2)
        grid2.addWidget(self.pic_drone, 1, 2)

        grid1.addItem(spacer, 0, 3)
        grid1.addItem(spacer, 1, 3)

        grid2.addWidget(self.label_ctrl, 0, 4)
        grid2.addWidget(self.pic_ctrl, 1, 4)

        vbox.addLayout(grid1)
        vbox.addLayout(grid2)

        central_widget.setLayout(vbox)

    def resizeEvent(self, event):
        self.updatePixmap()
    
    def updatePixmap(self):
        """Scale the image to fit within its QLabel while maintaining aspect ratio."""
        if not self.pixmap_cam.isNull():
            scaled_pixmap_cam = self.pixmap_cam.scaled(
                self.pic_fwd_cam.size(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            self.pic_fwd_cam.setPixmap(scaled_pixmap_cam)
        if not self.pixmap_drone.isNull():
            self.pic_drone.setAutoFillBackground
            scaled_pixmap_drone = self.pixmap_drone.scaled(
                min(self.pic_drone.width(),self.pic_drone.height()), min(self.pic_drone.width(),self.pic_drone.height()), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            self.pic_drone.setPixmap(scaled_pixmap_drone)

    def closeEvent(self, event):
        self.ros_thread.stop()
        event.accept()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()