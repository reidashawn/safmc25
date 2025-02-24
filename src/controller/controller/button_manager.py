import rclpy.node
from rclpy.node import Node
from std_msgs.msg import Int32  # Import Int32 message type


class ButtonManager(Node):
    def __init__(self):
        super().__init__('button_manager')

        self.but1_subscriber = self.create_subscription(Int32, 'controller/but1', 10)
        self.but2_subscriber = self.create_subscription(Int32, 'controller/but2', 10)
        self.but3_subscriber = self.create_subscription(Int32, 'controller/but3', 10)
        self.but4_subscriber = self.create_subscription(Int32, 'controller/but4', 10)

        