import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.msg import State
from helpers.button import Button
from time import sleep



VALID_MODES = [
    'GUIDED',
    'LAND',
    'STABILIZE',
    'LOITER'
]

TAKEOFF_HEIGHT = 1.0
MAV_CMD_NAV_TAKEOFF = 22 # From Mavlink MAV_CMD: https://mavlink.io/en/messages/common.html#mav_commands

class ButtonManagerNode(Node):
    def __init__(self):
        super().__init__('button_manager')

        self.is_armed = False

        self.button1 = Button(topic='controller/but1', short_callback=self.arm_takeoff_callback)
        # self.button2 = Button(topic='controller/but2', short_callback=self.takeoff)
        self.button3 = Button(topic='controller/but3', short_callback=self.land_callback)
        self.button4 = Button(topic='controller/but4', short_callback=self.set_guided_callback)

        # RIGHT CONTROLLER BUTTONS
        self.but1_subscriber = self.create_subscription(Int32, 'controller/but1', self.button1.data_callback, 10)
        # self.but2_subscriber = self.create_subscription(Int32, 'controller/but2', self.button2.data_callback, 10)
        self.but3_subscriber = self.create_subscription(Int32, 'controller/but3', self.button3.data_callback, 10)
        self.but4_subscriber = self.create_subscription(Int32, 'controller/but4', self.button4.data_callback, 10)
        # MAVROS STATE SUBSCRIPTION
        self.is_armed_subscriber = self.create_subscription(State, '/mavros/state', self.is_armed_callback, 10)

    
        self.mavros_clients = {
            'mode': self.create_client(SetMode, '/mavros/set_mode'),
            'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
            'takeoff': self.create_client(CommandLong, '/mavros/cmd/command')
        }

        for service_name, client in self.mavros_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Waiting for {service_name} service')
        
        self.get_logger().info('All services online')

    def arm_takeoff_callback(self):
        self.get_logger().info("arm and takeoff")
        self.arm_drone()
            
    def land_callback(self):
        self._change_mode('LAND')
    
    def set_guided_callback(self):
        self._change_mode('GUIDED')

    def is_armed_callback(self, msg: State):
        self.is_armed = msg.armed


    def _change_mode(self, mode: str):
        mode_upper = mode.upper()
        if mode_upper not in VALID_MODES:
            self.get_logger().error(f"{mode.upper()} is not a valid mode")
            return
        mode_req = SetMode.Request(custom_mode=mode_upper)
        future = self.mavros_clients['mode'].call_async(mode_req)
        self.get_logger().info(f"Change mode to {mode_upper} result: {future.result()}")

    def arm_drone(self):
        arm_req = CommandBool.Request(value=True)
        future = self.mavros_clients['arm'].call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

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
        future = self.mavros_clients['takeoff'].call_async(takeoff_req)
        self.get_logger().info(f"Takeoff command result: {future.result()}")

    

def main(args=None):
    rclpy.init(args=args)
    button_manager = ButtonManagerNode()
    rclpy.spin(button_manager)
    button_manager.destroy_node()

if __name__ == '__main__':
    main()

        