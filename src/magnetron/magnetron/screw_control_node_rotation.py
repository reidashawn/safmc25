import rclpy
from rclpy.node import Node
from interfaces.srv import ToggleStepper  # Replace with your actual service definition
import threading
import sys
import tty
import termios

class StepperClient(Node):
    def __init__(self):
        super().__init__('stepper_client')
        self.client = self.create_client(ToggleStepper, '/rotate_stepper')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

        self.lock = threading.Lock()
        self.running = True
        self.interval = 1

        # Start keyboard listener thread
        self.input_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.input_thread.start()

    def key_listener(self):
        """Reads key inputs and updates stepper rotation."""
        while self.running:
            key = self.read_key()
            if key:
                self.handle_key_press(key)


    def read_key(self):
        """Reads a single key press from stdin."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def handle_key_press(self, key):
        """Handles key press events."""
        key_map = {"u": "u", "j": "j", "i": "i", "k": "k"}

        if key in key_map:
            print(f"{key} pressed")
            stepper_id = 1 if key in ["u", "j"] else 2
            direction = self.interval if key in ["u", "i"] else -self.interval
            self.update_rotation(stepper_id, direction)

        if key == "q":
            self.running = False

    def update_rotation(self, stepper, rotation):
        """Sends a request to update stepper rotation."""
        with self.lock:
            self.send_request(stepper, rotation)

    def send_request(self, stepper, rotation):
        request = ToggleStepper.Request()
        request.stepper_id = stepper
        request.speed = float(rotation)

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

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
