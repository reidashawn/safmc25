import rclpy
from rclpy.node import Node
from interfaces.srv import TogglePin 
import pigpio
import time

class Pin:
    """
    Represents a PWM-controlled pin with angle-based control.
    """
    pulse_min = 500   # Pulse width (µs) corresponding to MIN_ANGLE
    pulse_max = 2500  # Pulse width (µs) corresponding to MAX_ANGLE

    def __init__(self, pi, pin, default_angle, angle=None, angle_range=180):
        self.pi = pi
        self.pin = pin
        pi.set_mode(pin, pigpio.OUTPUT)  # Set pin as an output
        
        self.angle_range = angle_range
        self.default_angle = default_angle
        self.angle = angle if angle is not None else default_angle  # Use provided angle or default
        self.set_angle(self.angle)
        
    def angle_to_duty(self, angle):
        """
        Converts an angle value to the corresponding duty cycle.
        """
        normalized = angle / self.angle_range
        return int(Pin.pulse_min + normalized * (Pin.pulse_max - Pin.pulse_min))
    
    def set_angle(self, angle):
        """
        Sets the desired angle and updates the last interaction time.
        """
        self.angle = angle
        pulse = self.angle_to_duty(angle)
        self.pi.set_servo_pulsewidth(self.pin, pulse)
        print(f'setting {self.pin} to {pulse}')
        time.sleep(0.1)
    
    def stop_pwm(self):
        """
        Stops PWM on the pin.
        """
        self.pi.set_servo_pulsewidth(self.pin, 0)

class PinController(Node):
    """
    ROS2 Node for controlling multiple PWM pins via a service.
    """
    def __init__(self):
        super().__init__('pin_controller')
        self.srv = self.create_service(TogglePin, 'set_servo', self.handle_toggle_pin)
        self.pins = {}  # Dictionary to track active pins
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon. Is it running?")
            self.destroy()
            rclpy.shutdown()
            return
        
        self.get_logger().info("PinController node is ready.")

    def handle_toggle_pin(self, request, response):
        """
        Handles service requests to toggle a PWM pin.
        """
        pin = request.pin
        angle = request.angle
        
        if pin in self.pins:
            self.pins[pin].set_angle(angle)  # Update existing pin instance
        else:
            self.pins[pin] = Pin(self.pi, pin, default_angle=10, angle=angle)  # Create new pin instance
        
        response.success = True
        return response
    
    def destroy(self):
        """
        Cleans up resources before shutting down the node.
        """
        self.get_logger().info("Cleaning up GPIO and shutting down.")
        for pin in self.pins.values():
            pin.stop_pwm()  # Stop PWM before exiting
        self.pi.stop()
        self.destroy_node()

def main(args=None):
    """
    Initializes the ROS2 node and starts the service.
    """
    rclpy.init(args=args)
    pin_controller = PinController()
    
    if not pin_controller.pi.connected:
        return  # Exit if pigpio is not connected
    
    try:
        rclpy.spin(pin_controller)  # Keep node running
    except KeyboardInterrupt:
        pin_controller.get_logger().info("Shutting down PinController node.")
    finally:
        pin_controller.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
