import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32  # Import Int32 message type
from controller.helpers.serial_helper import SerialHelper
from mavros_msgs.srv import CommandBool, SetMode, CommandLong

VALID_MODES = [
    'GUIDED',
    'LAND',
    'STABILIZE',
    'LOITER'
]

TAKEOFF_HEIGHT = 1.0
MAV_CMD_NAV_TAKEOFF = 22 # From Mavlink MAV_CMD: https://mavlink.io/en/messages/common.html#mav_commands

class ControllerPubNode(Node):
    def __init__(self):
        super().__init__('controller_pub')
        
        # Declare parameters for serial port and baud rate with default values
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Get the parameters set at runtime (or use the defaults)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Initialize the SerialHelper with the parameters
        self.ser = SerialHelper(serial_port, baud_rate)

        # Clients
        self.clients = {
            'mode': self.create_client(SetMode, '/mavros/set_mode'),
            'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
            'takeoff': self.create_client(CommandLong, '/mavros/cmd/command')
        }

        for service_name, client in self.clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Waiting for {service_name} service')
        
        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.pot_publisher = self.create_publisher(Int32, 'controller/pot', 10)
        self.but_publisher = self.create_publisher(Int32, 'controller/but1', 10)

        # TODO: Subscribe to get state of drone for deconfliction

        # Start a timer to update IMU data every 50ms (20 Hz)
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        try:
            # Read and process serial data
            line = self.ser.read_from_serial()
            # TODO: Send data from ESP in dict format and convert it from its string representation
            if line:
                # print(line)
                split = line.split(',')
                if len(split) != 8:
                    return
                
                # Convert data to float (accelerometer and gyroscope data)
                accel_data = list(map(float, split[:3]))  # accelx, accely, accelz
                gyro_data = list(map(lambda x: float(x) * (3.141592653589793 / 180), split[3:6]))  # Convert to radians/sec
                pot_value = int(split[6])  # Potentiometer value (7th number)
                but_value = int(split[7])  # Button state (8th number)

                # Create and populate IMU message
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = accel_data[0]
                imu_msg.linear_acceleration.y = accel_data[1]
                imu_msg.linear_acceleration.z = accel_data[2]
                imu_msg.angular_velocity.x = gyro_data[0]
                imu_msg.angular_velocity.y = gyro_data[1]
                imu_msg.angular_velocity.z = gyro_data[2]
                self.imu_publisher.publish(imu_msg)

                # Create and publish Potentiometer message
                pot_msg = Int32()
                pot_msg.data = pot_value
                self.pot_publisher.publish(pot_msg)
                
                # Create and publish Button message
                but_msg = Int32()
                but_msg.data = but_value
                self.but_publisher.publish(but_msg)

        except Exception as e:
            self.get_logger().error(f"Error in update: {e}")

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



def main(args=None):
    rclpy.init(args=args)
    controller_pub = ControllerPubNode()
    rclpy.spin(controller_pub)
    controller_pub.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
