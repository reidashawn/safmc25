import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from interfaces.srv import ToggleStepper
import threading


class StepperClient(Node):
    def __init__(self):
        super().__init__('stepper_client')
        self.client = self.create_client(ToggleStepper, '/toggle_stepper')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

        self.stepper_speed = {1: 0, 2: 0}
        self.lock = threading.Lock()
        self.running = True
        self.interval = 1

        # Start a thread for reading key inputs
        self.input_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.input_thread.start()

        # Create a timer that sends requests every 0.1 seconds
        self.timer = self.create_timer(0.1, self.send_periodic_request)

    def key_listener(self):
        """Reads key inputs from stdin and updates stepper speeds accordingly."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)  # Set terminal to raw mode
            while self.running:
                key = sys.stdin.read(1)  # Read one character at a time

                if key == "u":
                    self.update_speed(1, self.interval)
                elif key == "j":
                    self.update_speed(1, -self.interval)
                elif key == "i":
                    self.update_speed(2, self.interval)
                elif key == "k":
                    self.update_speed(2, -self.interval)
                elif key == "q":
                    self.running = False
                    break  # Exit the loop
                elif key in ["\x03", "\x04"]:  # Handle Ctrl+C and Ctrl+D
                    self.running = False
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Restore terminal settings

    def update_speed(self, stepper, speed):
        """Updates the stepper motor speed."""
        with self.lock:
            self.stepper_speed[stepper] += speed

    def send_request(self, stepper):
        """Sends a service request to update the stepper motor speed."""
        request = ToggleStepper.Request()
        request.stepper_id = stepper
        request.speed = float(self.stepper_speed[stepper])

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def send_periodic_request(self):
        """Sends a request every 0.1 seconds to update stepper speeds."""
        with self.lock:
            for stepper in self.stepper_speed:
                self.send_request(stepper)

    def response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    stepper_client = StepperClient()

    try:
        rclpy.spin(stepper_client)
    except KeyboardInterrupt:
        stepper_client.get_logger().info('Shutting down StepperClient node.')
    finally:
        stepper_client.running = False
        stepper_client.input_thread.join()
        stepper_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
