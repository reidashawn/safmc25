import termios
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavros_msgs.srv import CommandBool, SetMode, CommandTOLLocal
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, Vector3
import threading
import sys
import tty

class MavControl(Node):
    '''
    Object to interact with MavROS via keyboard controls
    '''
    def __init__(self):
        super().__init__('keyboard_controller_node')
        self.state = State()

        # subscibers
        self.mav_state = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # publishers

        # clients for various services
        # self.clients = {
        #     'mode': self.create_client(SetMode, '/mavros/set_mode'),
        #     'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
        #     'takeoff': self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local'),
        #     # 'land': self.create_client(SetMode, '/mavros/set_mode')
        # }

        # for service_name, client in self.clients.items():
        #     while not client.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().warn(f'Waiting for {service_name} service')
        
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'waiting for mode service')

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'waiting for arm service')

        self.takeoff_client = self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'waiting for takeoff service')

       

        self.get_logger().info('Keyboard control is online')
        
    
    def state_callback(self, data):
        self.state = data

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)

        finally: 
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key


    def send_command(self, service_name, command):
        if service_name not in self.clients:
            self.get_logger().error(f'{service_name} cannot be found')
            return

    def change_mode(self, mode: str):
        mode_req = SetMode.Request()
        mode_req.custom_mode = mode
        future = self.mode_client.call_async(mode_req)
        self.get_logger().info(f"Change Mode result: {future.result()}")
    
    def arm_drone(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arm_client.call_async(arm_req)
        self.get_logger().info(f"Arm Drone result: {future.result()}")
    
    def takeoff(self):
        '''
        hover to a height of 0.3m above the ground at 0.5m/s
        '''
        hover_pos = Vector3(x=0.0, y=0.0, z=0.3)

        takeoff_req = CommandTOLLocal.Request(min_pitch=0.0, offset=0.0, rate=0.5, yaw=0.0, position=hover_pos)

        future = self.takeoff_client.call_async(takeoff_req)
        self.get_logger().info(f"Takeoff command result: {future.result()}")

    

    def keyboard_loop(self):
        while rclpy.ok():
            key = self.get_key()
            self.get_logger().info(f"{key} pressed")

            if key == "q":
                break
            elif key == "g":
                self.change_mode("GUIDED")
            elif key == "l":
                self.change_mode("LAND")
            elif key == "s":
                self.change_mode("STABILIZE")
            elif key == "m":
                self.arm_drone()
            elif key == "t":
                self.takeoff()


    
def main():
    rclpy.init()
    node = MavControl()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    print('starting thread')
    thread.start()

    print('running keyboard loop')

    node.keyboard_loop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

