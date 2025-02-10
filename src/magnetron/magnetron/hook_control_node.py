import rclpy
from rclpy.node import Node
from interfaces.srv import TogglePin  # Replace with your actual service definition
import threading
import sys
import termios
import tty

class PinClient(Node):
    def __init__(self):
        super().__init__('pin_client')
        self.client = self.create_client(TogglePin, 'toggle_pin')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        
        self.pin_angles = {17: 0, 18: 0}  # Track angles for both pins
        self.lock = threading.Lock()
        self.running = True

        # Start a thread for reading key inputs
        self.input_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.input_thread.start()

        # Create a timer that sends requests every 0.9 seconds
        self.timer = self.create_timer(0.9, self.send_periodic_request)
    
    def key_listener(self):
        """Reads keyboard input and updates pin angles accordingly."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                key = sys.stdin.read(1)
                if key == 'u':
                    self.update_angle(17, 5)
                elif key == 'j':
                    self.update_angle(17, -5)
                elif key == 'i':
                    self.update_angle(18, 5)
                elif key == 'k':
                    self.update_angle(18, -5)
                elif key == 'q':
                    self.running = False
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def update_angle(self, pin, delta):
        """Updates the angle of the specified pin and sends a request."""
        with self.lock:
            self.pin_angles[pin] = max(0, min(180, self.pin_angles[pin] + delta))
            self.send_request(pin, self.pin_angles[pin])
    
    def send_request(self, pin, angle):
        request = TogglePin.Request()
        request.pin = pin
        request.angle = angle
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def send_periodic_request(self):
        """Sends a request every 0.9 seconds regardless of keypress."""
        with self.lock:
            for pin, angle in self.pin_angles.items():
                self.send_request(pin, angle)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully toggled pin')
            else:
                self.get_logger().warn('Failed to toggle pin')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    pin_client = PinClient()
    
    try:
        rclpy.spin(pin_client)
    except KeyboardInterrupt:
        pin_client.get_logger().info('Shutting down PinClient node.')
    finally:
        pin_client.running = False
        pin_client.input_thread.join()
        pin_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
