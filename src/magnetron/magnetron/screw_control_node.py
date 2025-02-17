import rclpy
from rclpy.node import Node
from interfaces.srv import ToggleStepper  # Replace with your actual service definition
import threading
import sys
import termios
import tty
import keyboard

class StepperClient(Node):
    def __init__(self):
        super().__init__('stepper_client')
        self.client = self.create_client(ToggleStepper, 'toggle_stepper')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        
        self.stepper_speed = {1: 0, 2: 0}  # Track angles for both pins
        self.lock = threading.Lock()
        self.running = True
        self.interval = 1

        # Start a thread for reading key inputs
        self.input_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.input_thread.start()

        # Create a timer that sends requests every 0.9 seconds
        self.timer = self.create_timer(0.1, self.send_periodic_request)
    


    def key_listener(self):
        """Reads multiple keypresses and updates stepper speed accordingly."""
        self.running = True
        while self.running:
            if keyboard.is_pressed("u"):
                self.update_speed(1, self.interval)
            elif keyboard.is_pressed("j"):
                self.update_speed(1, -self.interval)
            else:
                self.update_speed(1, 0)
            if keyboard.is_pressed("i"):
                self.update_speed(2, self.interval)
            elif keyboard.is_pressed("k"):
                self.update_speed(2, -self.interval)
            else:
                self.update_speed(2, 0)
            if keyboard.is_pressed("q"):
                self.running = False
                break

    
    def update_speed(self, stepper, speed):
        """Updates the speed of the specified pin and sends a request."""
        with self.lock:
            self.stepper_speed[stepper] = speed
            self.send_request(stepper)
    
    def send_request(self, stepper):
        request = ToggleStepper.Request()
        request.stepperID = stepper
        request.speed = self.stepper_speed[stepper]
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def send_periodic_request(self):
        """Sends a request every 0.9 seconds regardless of keypress."""
        with self.lock:
            for pin, angle in self.stepper_speed.items():
                self.send_request(pin, angle)

    def response_callback(self, future):
        try:
            response = future.result()
            # if response.success:
                # self.get_logger().info('Successfully toggled pin')
            # else:
                # self.get_logger().warn('Failed to toggle pin')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    pin_client = StepperClient()
    
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
