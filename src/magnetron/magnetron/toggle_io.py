#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from interfaces.srv import TogglePin
import RPi.GPIO as GPIO

class GPIOToggleServer(Node):
    def __init__(self):
        super().__init__('gpio_toggle_server')
        self.srv = self.create_service(TogglePin, 'toggle_gpio', self.toggle_gpio_callback)

        # Dictionary to track pin states
        self.pin_states = {}

        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)
        self.get_logger().info('GPIO Toggle Server ready.')

    def toggle_gpio_callback(self, request, response):
        pin = request.pin
        state = request.state

        try:
            # Setup the pin if not already done
            if pin not in self.pin_states:
                GPIO.setup(pin, GPIO.OUT)
                self.pin_states[pin] = False  # Default state is OFF

            # Set the GPIO pin state
            GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
            self.pin_states[pin] = state

            response.success = True
            response.message = f'Pin {pin} set to {"HIGH" if state else "LOW"}.'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Failed to toggle pin {pin}: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()
        self.get_logger().info('GPIO cleaned up and node shut down.')

def main(args=None):
    rclpy.init(args=args)
    node = GPIOToggleServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
