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
        self.client = self.create_client(ToggleStepper, 'rotate_stepper')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        
        self.lock = threading.Lock()
        self.running = True
        self.interval = 1
        self.past_state = {"u": 0, "j": 0, "i": 0, "k": 0}

        # Start a thread for reading key inputs
        self.input_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.input_thread.start()
    


    def key_listener(self):
        """Reads multiple keypresses and updates stepper speed accordingly."""
        self.running = True
        while self.running:

            if keyboard.is_pressed("u") and self.past_state["u"] == 0:
                self.update_rotation(1, self.interval)
            elif not keyboard.is_pressed("u"):
                self.past_state["u"] = 0

            if keyboard.is_pressed("j") and self.past_state["j"] == 0:
                self.update_rotation(1, -self.interval)
            elif not keyboard.is_pressed("j"):
                self.past_state["j"] = 0

            if keyboard.is_pressed("i") and self.past_state["i"] == 0:
                self.update_rotation(2, self.interval)
            elif not keyboard.is_pressed("i"):
                self.past_state["i"] = 0

            if keyboard.is_pressed("k") and self.past_state["k"] == 0:
                self.update_rotation(2, -self.interval)
            elif not keyboard.is_pressed("k"):
                self.past_state["k"] = 0

            if keyboard.is_pressed("q"):
                self.running = False
                break

    
    def update_rotation(self, stepper, rotation):
        """Updates the speed of the specified pin and sends a request."""
        with self.lock:
            self.send_request(stepper, rotation)
    
    def send_request(self, stepper, rotation):
        request = ToggleStepper.Request()
        request.stepper_id = stepper
        request.speed = rotation
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)
    
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
