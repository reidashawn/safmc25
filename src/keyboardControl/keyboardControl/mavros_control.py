
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist, Vector3
import threading
import sys
import tty
from pynput import keyboard


HORIZONTAL_VELOCITY = 0.1
TAKEOFF_HEIGHT = 1.0


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

        self.takeoff_client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'waiting for takeoff service')

        self.cmd_vel_publisher = self.create_publisher(Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10)

        self.listener_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
        self.listener_thread.start()


        self.get_logger().info('Keyboard control is online')
        
    
    def state_callback(self, data):
        self.state = data


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
        hover to a height of 1.0m above the ground at 0.5m/s
        '''
        

        takeoff_req = CommandLong.Request(broadcast=False, command=22, confirmation=0, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=TAKEOFF_HEIGHT)

        future = self.takeoff_client.call_async(takeoff_req)
        self.get_logger().info(f"Takeoff command result: {future.result()}")


    def horizontal_movement(self, key: str):
        directions = {
            # forward
            "w": Twist(linear=Vector3(x=HORIZONTAL_VELOCITY, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            # backward
            "s": Twist(linear=Vector3(x=-HORIZONTAL_VELOCITY, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            # left
            "a": Twist(linear=Vector3(x=0.0, y=-HORIZONTAL_VELOCITY, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            # right
            "d": Twist(linear=Vector3(x=0.0, y=HORIZONTAL_VELOCITY, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        }
        if key in directions:
            self.cmd_vel_publisher.publish(directions[key])
            self.get_logger().info(f"{key} Horizontal command issued")

       
        

    def on_press(self, key):
        try:
            key_char = key.char  # Detect character keys
        except AttributeError:
            key_char = None  # Handle special keys

        if key_char in ["w", "a", "s", "d"]:
            self.horizontal_movement(key_char)
        elif key_char == "g":
            self.change_mode("GUIDED")
        elif key_char == "l":
            self.change_mode("LAND")
        elif key_char == "h":
            self.change_mode("STABILIZE")
        elif key_char == "m":
            self.arm_drone()
        elif key_char == "t":
            self.takeoff()
        elif key == keyboard.Key.esc:  # Stop listener on ESC
            self.get_logger().info("Shutting down keyboard listener...")
            return False

    def start_keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()


    
def main():
    rclpy.init()
    node = MavControl()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    print('starting thread')
    thread.start()

    print('running keyboard loop')

    try:
        thread.join()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

