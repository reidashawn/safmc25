import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOLLocal, 
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist

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
        self.clients = {
            'mode': self.create_client(SetMode, 'mavros/set_mode'),
            'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
            'takeoff': self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local'),
            # 'land': self.create_client(SetMode, '/mavros/set_mode')
        }

        for service_name, client in self.clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Waiting for {service_name} service')

        self.get_logger().info('Keyboard control is online')
        
    
    def state_callback(self, data):
        self.state = data

    def send_command(self, service_name, command):
        pass
    

        

