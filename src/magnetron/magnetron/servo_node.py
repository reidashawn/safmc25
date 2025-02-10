import rclpy
from rclpy.node import Node
from interfaces.srv import TogglePin  # Replace with your actual service definition
import RPi.GPIO as GPIO
import time
import threading

class Pin:
    """
    Represents a PWM-controlled pin with angle-based control.
    """
    max_interval = 0.5  # Maximum interval before resetting to default angle

    def __init__(self, pin, freq, default_angle, angle=None, angle_range=180, min_duty=1, max_duty=99):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)  # Set GPIO mode to BCM
        GPIO.setup(pin, GPIO.OUT)  # Set pin as an output
        
        self.freq = freq
        self.pwm = GPIO.PWM(pin, freq)  # Initialize PWM
        self.angle_range = angle_range
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.default_angle = default_angle
        self.angle = angle if angle is not None else default_angle  # Use provided angle or default
        self.last_time = time.time()
        
        self.thread = threading.Thread(target=self.toggle_pwm, daemon=True)  # Daemon thread to toggle PWM
        self.thread.start()
    
    def toggle_pwm(self):
        """
        Toggles PWM output based on the current angle.
        If no angle update occurs within max_interval, resets to default.
        """
        try:
            while True:
                if (time.time() - self.last_time) > self.max_interval and self.angle != self.default_angle:
                    self.angle = self.default_angle  # Reset angle if no update occurs
                
                self.set_pwm(self.angle_to_duty())
                time.sleep(0.2)  # Keep PWM active for 0.4s
                self.set_pwm(0)  # Turn off PWM
                time.sleep(.8)  # Wait before next cycle
        finally:
            self.set_pwm(self.angle_to_duty(self.default_angle))  # Reset to default angle on exit
            self.pwm.stop()
            GPIO.cleanup()
    
    def set_pwm(self, duty_cycle):
        """
        Updates the PWM duty cycle on the pin.
        """
        self.pwm.ChangeDutyCycle(duty_cycle)
    
    def angle_to_duty(self, angle=None):
        """
        Converts an angle value to the corresponding duty cycle.
        """
        angle = self.angle if angle is None else angle
        angle = max(0, min(angle, self.angle_range))  # Clamp angle within valid range
        duty = (angle / self.angle_range) * (self.max_duty - self.min_duty) + self.min_duty
        return duty
    
    def set_angle(self, angle):
        """
        Sets the desired angle and updates the last interaction time.
        """
        self.last_time = time.time()
        self.angle = angle

class PinController(Node):
    """
    ROS2 Node for controlling multiple PWM pins via a service.
    """
    def __init__(self):
        super().__init__('pin_controller')
        self.srv = self.create_service(TogglePin, 'toggle_pin', self.handle_toggle_pin)
        self.pins = {}  # Dictionary to track active pins
        self.pwm_frequency = 50  # Default PWM frequency (Hz)
        self.get_logger().info("PinController node is ready.")

    def handle_toggle_pin(self, request, response):
        """
        Handles service requests to toggle a PWM pin.
        """
        pin = request.pin
        angle = request.angle
        self.get_logger().info(f"Received request to set pin {pin} to angle {angle}")
        
        if pin in self.pins:
            self.pins[pin].set_angle(angle)  # Update existing pin instance
        else:
            self.pins[pin] = Pin(pin, self.pwm_frequency, default_angle=0, angle=angle)  # Create new pin instance
        
        response.success = True
        return response

def main(args=None):
    """
    Initializes the ROS2 node and starts the service.
    """
    rclpy.init(args=args)
    pin_controller = PinController()
    
    try:
        rclpy.spin(pin_controller)  # Keep node running
    except KeyboardInterrupt:
        pin_controller.get_logger().info("Shutting down PinController node.")
    finally:
        pin_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
