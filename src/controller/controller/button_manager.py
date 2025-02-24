import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


TAKEOFF_HEIGHT = 1.0
MAV_CMD_NAV_TAKEOFF = 22 # From Mavlink MAV_CMD: https://mavlink.io/en/messages/common.html#mav_commands

class ButtonManager(Node):
    def __init__(self):
        super().__init__('button_manager')

        self.but1_subscriber = self.create_subscription(Int32, 'controller/but1', 10)
        self.but2_subscriber = self.create_subscription(Int32, 'controller/but2', 10)
        self.but3_subscriber = self.create_subscription(Int32, 'controller/but3', 10)
        self.but4_subscriber = self.create_subscription(Int32, 'controller/but4', 10)

    
        self.clients = {
            'mode': self.create_client(SetMode, '/mavros/set_mode'),
            'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
            'takeoff': self.create_client(CommandLong, '/mavros/cmd/command')
        }

        for service_name, client in self.clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Waiting for {service_name} service')

        def change_mode(self, mode: str):
            mode_upper = mode.upper()
            if mode_upper not in VALID_MODES:
                self.get_logger().error(f"{mode.upper()} is not a valid mode")
                return
            mode_req = SetMode.Request(custom_mode=mode_upper)
            future = self.clients['mode'].call_async(mode_req)
            self.get_logger().info(f"Change mode to {mode_upper} result: {future.result()}")

        def arm_drone(self):
            arm_req = CommandBool.Request(value=True)
            future = self.clients['arm'].call_async(arm_req)
            self.get_logger().info(f"Arm drone result: {future.result()}")

        def takeoff(self):
            takeoff_req = CommandLong.Request(
                broadcast=False, 
                command=MAV_CMD_NAV_TAKEOFF, 
                confirmation=0, 
                param1=0.0, 
                param2=0.0, 
                param3=0.0, 
                param4=0.0, 
                param5=0.0, 
                param6=0.0, 
                param7=TAKEOFF_HEIGHT
                )
            future = self.clients['takeoff'].call_async(takeoff_req)
            self.get_logger().info(f"Takeoff command result: {future.result()}")

        