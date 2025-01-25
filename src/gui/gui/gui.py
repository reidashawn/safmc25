import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel,
                             QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QGraphicsRectItem, 
                             QSpacerItem, QSizePolicy)
from PyQt5.QtGui import QIcon, QFont, QPixmap, QPalette, QColor, QPainter, QImage
from PyQt5.QtCore import Qt, QRectF, QRect, QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # For camera image messages
from std_msgs.msg import Float32  # For PWM sensor data (adjust if necessary)
from cv_bridge import CvBridge  # For converting ROS2 Image messages to OpenCV format
import cv2  # For OpenCV image processing


# Dimensions
margin = 10
camera_width, camera_height = 900, 600
label_height = 30
info_width, info_height = 300, 450

# Colours
text_colour = "#F0F1F1"
background_colour = "#353535"
window_colour = "#242424"

class GUISubscriber(Node):  # (SHAWN)
    def __init__(self, update_camera_callback):
        super().__init__('gui_subscriber')

        self.bridge = CvBridge()

        # Subscription for Camera
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera1/color/image_raw',  # Topic for Camera 1
            self.camera_callback,
            10
        )
        self.update_camera_callback = update_camera_callback

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Convert to OpenCV format
        self.update_camera_callback(cv_image)  # Send data to GUI

class ROS2Thread(QThread): # (SHAWN)
    camera_signal = pyqtSignal(object)  # Signal for Camera 1

    def run(self):
        rclpy.init()
        self.node = GUISubscriber(
            self.camera_signal.emit
        )
        rclpy.spin(self.node)
        rclpy.shutdown()

class MainWindow(QMainWindow):
    def __init__(self):

        super().__init__()
        self.setWindowTitle("SAFMC GUI")

        # Forward cam (SHAWN)
        self.label_fwd_cam, self.pic_fwd_cam = self.createCam("    Forward Cam")
        self.updateCam(self.pic_fwd_cam)

        # Downward cam
        self.label_dwd_cam, self.pic_dwd_cam = self.createCam("    Downward Cam")
        self.updateCam_fake(self.pic_dwd_cam)

        # UAV Info
        self.label_UAV, self.info_UAV = self.createUAVInfo()

        # Payload
        self.label_payload = QLabel("    Payload", self)
        self.label_payload.setGeometry(2 * margin + info_width, 2 * margin + label_height + camera_height, info_height // 6 * 10, label_height)
        self.label_payload.setFixedSize(info_height // 6 * 10, label_height)
        self.label_payload.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_payload.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        self.pic_drone = QLabel(self)
        self.pic_drone.setGeometry(margin, margin + label_height, info_height // 6 * 10, info_height)
        pixmap_drone = QPixmap("drone.jpg")        
        self.pic_drone.setPixmap(pixmap_drone.scaled(self.pic_drone.size(), aspectRatioMode=1))
        self.pic_drone.setFixedSize(info_height // 6 * 10, info_height)
        self.pic_drone.setAlignment(Qt.AlignHCenter | Qt.AlignTop)


        # Controllers
        self.label_ctrl = QLabel("    Controllers", self)
        self.label_ctrl.setGeometry(3 * margin + info_width // 6 * 16, 2 * margin + label_height + camera_height, info_height // 6 * 10, label_height)
        self.label_ctrl.setFixedSize(info_height // 6 * 10, label_height)
        self.label_ctrl.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        self.label_payload.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        # Controllers
        self.pic_ctrl = QLabel(self)
        self.pic_ctrl.setGeometry(margin, margin + label_height, info_height // 6 * 10, info_height)
        pixmap_ctrl = QPixmap("controllers.jpg")        
        self.pic_ctrl.setPixmap(pixmap_ctrl.scaled(self.pic_ctrl.size(), aspectRatioMode=1))
        self.pic_ctrl.setFixedSize(info_height // 6 * 10, info_height)
        self.pic_ctrl.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        # Misc
        self.label_spacer = QLabel(self)
        self.label_spacer.setGeometry(margin, margin + label_height + camera_height, info_width, label_height)
        self.label_spacer.setFixedHeight(margin)

        # General
        self.setStyleSheet("background-color: #353535;")
        self.initUI()
    
    def createCam(self, label):
        label_cam = QLabel(label, self)
        label_cam.setGeometry(margin, margin, camera_width, label_height)
        label_cam.setFixedHeight(label_height)
        label_cam.setStyleSheet("color: #F0F1F1;"
                                         "background-color: #242424;")
        label_cam.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        
        pic_cam = QLabel(self)
        pic_cam.setGeometry(margin, margin + label_height, camera_width, camera_height)
        
        pic_cam.setFixedSize(camera_width, camera_height)
        pic_cam.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        pic_cam.setStyleSheet("""
            border: 20px solid #242424;
        """)

        return label_cam, pic_cam
    
    def createUAVInfo(self):
        label_UAV = QLabel("    UAV Info", self)
        label_UAV.setGeometry(margin, 2 * margin + label_height + camera_height, info_width, label_height)
        label_UAV.setFixedSize(info_width, label_height)
        label_UAV.setStyleSheet("color: #F0F1F1;"
                                     "background-color: #242424;")
        label_UAV.setAlignment(Qt.AlignLeft | Qt.AlignBottom)

        info_UAV = QLabel("    Armed : ARMED\n    Battery : 96%\n    Flight Mode : GUIDED\n\n    Pitch : 0.6\n    Roll : -0.3\n    Yaw : 359\n    Altitude : 1.6m\n", self)
        info_UAV.setGeometry(margin, 2 * margin + 2 * label_height + camera_height, info_width, info_height)
        info_UAV.setFont(QFont("Arial", 10))
        info_UAV.setFixedSize(info_width, info_height)
        info_UAV.setStyleSheet("color: #F0F1F1;"
                               "background-color: #242424;")
        info_UAV.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        return label_UAV, info_UAV

    def updateCam(self, pic_cam, cv_image):  # (SHAWN)
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap_cam = QPixmap.fromImage(q_image)
        pic_cam.setPixmap(pixmap_cam)

    def updateCam_fake(self, pic_cam):
        pixmap_cam = QPixmap("dwd_cam_fake.jpg")        
        pic_cam.setPixmap(pixmap_cam.scaled(pic_cam.size(), aspectRatioMode=1))

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
        grid2.addWidget(self.info_UAV, 1, 0)

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

def main():
    app = QApplication(sys.argv)
    window = MainWindow()

    # (SHAWN)
    ros2_thread = ROS2Thread()
    ros2_thread.camera_signal.connect(window.update_camera)
    ros2_thread.start()

    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()