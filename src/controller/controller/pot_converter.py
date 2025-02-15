import rclpy
import csv
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from std_msgs.msg import Int32
from controller.helpers.filters import LowPassFilter
from controller.helpers.joystick import Joystick

class PotConverter(Node):
    def __init__(self):
        super().__init__('pot_converter')

        # Declare parameters
        self.declare_parameter('alpha', 0.5, 
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Low-pass filter coefficient"))
        self.declare_parameter('joy_min_input', 5)
        self.declare_parameter('joy_zero', 25)
        self.declare_parameter('joy_max_input', 70)
        self.declare_parameter('joy_min_output', 0.0)
        self.declare_parameter('joy_max_output', 0.5)

        # Get initial parameter values
        self.alpha = self.get_parameter('alpha').value
        joy_min_input = self.get_parameter('joy_min_input').value
        joy_zero = self.get_parameter('joy_zero').value
        joy_max_input = self.get_parameter('joy_max_input').value
        joy_min_output = self.get_parameter('joy_min_output').value
        joy_max_output = self.get_parameter('joy_max_output').value

        # Initialize objects
        self.low_pass_filter = LowPassFilter(self.alpha)
        self.joystick = Joystick(joy_zero, 10, joy_max_input, joy_min_input, joy_max_output)

        # Subscribe to pot topic
        self.subscription = self.create_subscription(Int32, 'controller/pot', self.pot_callback, 10)
        self.pub = self.create_publisher(Int32, '/cmd_vel_vert', 10)

        # Load calibration data
        self.calibration_file = 'pot_calibration.csv'
        self.calibration_data = self.load_calibration()

        # Add parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

    def load_calibration(self):
        calibration_data = {}
        try:
            with open(self.calibration_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header row
                for row in reader:
                    y_value = int(float(row[0]))
                    x_value = float(row[1])
                    calibration_data[y_value] = x_value
        except Exception as e:
            self.get_logger().error(f"Error loading calibration file: {e}")
        return calibration_data

    def pot_callback(self, msg):
        if 0 <= msg.data <= 1023:
            filtered_reading = self.low_pass_filter.update(msg.data)
            corresponding_value = self.calibration_data[int(filtered_reading)]
            cmd_vel_vert = self.joystick.get_output(corresponding_value)

            # Publish vertical command velocity
            cmd_msg = Int32()
            cmd_msg.data = int(cmd_vel_vert)  # Example scaling
            # print(f"reading: {filtered_reading}, cali: {corresponding_value}, vel: {cmd_vel_vert}")
            self.pub.publish(cmd_msg)

    def param_callback(self, params):
        for param in params:
            if param.name == 'alpha':
                self.alpha = param.value
                self.low_pass_filter = LowPassFilter(self.alpha)
            elif param.name in ['joy_min_input', 'joy_zero', 'joy_max_input', 'joy_min_output', 'joy_max_output']:
                joy_min_input = self.get_parameter('joy_min_input').get_parameter_value().integer_value
                joy_zero = self.get_parameter('joy_zero').get_parameter_value().integer_value
                joy_max_input = self.get_parameter('joy_max_input').get_parameter_value().integer_value
                joy_min_output = self.get_parameter('joy_min_output').get_parameter_value().double_value
                joy_max_output = self.get_parameter('joy_max_output').get_parameter_value().double_value
                self.joystick = Joystick(joy_min_input, joy_zero, joy_max_input, joy_min_output, joy_max_output)

        return rclpy.parameter.SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    pot_converter = PotConverter()
    rclpy.spin(pot_converter)
    pot_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
