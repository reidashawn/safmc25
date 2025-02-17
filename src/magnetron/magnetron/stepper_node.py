import rclpy
from rclpy.node import Node
from interfaces.srv import ToggleStepper  # Replace with your actual service definition
import RPi.GPIO as GPIO
import time
import threading


IN1, IN2, IN3, IN4 = 17, 18, 27, 22  # GPIO Pins
SEQUENCE = [[1, 0, 0, 1], [1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0],
            [0, 1, 1, 0], [0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1]]

GPIO.setmode(GPIO.BCM)
for pin in (IN1, IN2, IN3, IN4):
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

def stepper_step(steps, delay=0.01):
    for _ in range(steps):
        for pattern in SEQUENCE:
            for pin, val in zip((IN1, IN2, IN3, IN4), pattern):
                GPIO.output(pin, val)
            time.sleep(delay)

stepper_step(512)  # Rotate 1 full revolution (depends on stepper)

GPIO.cleanup()


class Stepper:
    """
    Represents a stepper motor with speed control.
    """
    max_interval = 0.5  # Maximum interval before stopping the stepper
    sequence = [
        [1, 0, 0, 1],
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1]
    ]

    def __init__(self, pins, delay):
        if pins:  # Check if pins list is non-empty
            self.pins = pins
            GPIO.setmode(GPIO.BCM)  # Use BCM numbering
            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
        else:
            raise ValueError("Pins list cannot be empty.")
        
        self.speed = 0
        self.delay = delay
        self.last_time = time.time()
        self.index = 0
        self.rotation_count = 0
        self.step_per_rot = 64
        self.running = True  # Control flag for the thread
        self.thread = threading.Thread(target=self.toggle_stepper, daemon=True)
        self.thread.start()

    
        def toggle_stepper(self):
            """
            Controls the stepper motor based on the current speed.
            If no speed update occurs within max_interval, stops the motor.
            """
            try:
                while self.running:
                    current_time = time.time()
                    if (current_time - self.last_time) > self.max_interval:
                        if self.speed != 0:
                            self.speed = 0  # Stop the motor if no recent updates
                            print("Interval too long; stopping stepper.")
                    
                    if self.speed != 0:
                        # Determine direction based on speed sign
                        direction = 1 if self.speed > 0 else -1
                        self.index = (self.index + direction) % len(self.sequence)
                        self.set_stepper(self.sequence[self.index])
                        time.sleep(self.delay)
                    elif self.rotation_count != 0:
                        direction = -1 if self.rotation_count > 0 else 1
                        self.index = (self.index + direction) % len(self.sequence)
                        self.set_stepper(self.sequence[self.index])
                        self.rotation_count += direction
                        
                    else:
                        self.set_stepper([0, 0, 0, 0])  # De-energize coils when stopped
                        time.sleep(0.1)  # Sleep briefly to prevent high CPU usage
            finally:
                self.set_stepper([0, 0, 0, 0])  # Ensure motor is stopped
                GPIO.cleanup()

    
    def set_stepper(self, pattern):
        """
        Sets the GPIO pins to the provided pattern.
        :param pattern: A list of 0s and 1s indicating pin states.
        """
        for pin, state in zip(self.pins, pattern):
            GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
    
    def set_speed(self, speed):
        self.last_time = time.time()
        self.speed = speed
    
    def set_rotation(self, rotation):
        self.rotation_count -= (self.step_per_rot * rotation)
    
    def stop(self):
        """
        Stops the stepper motor and cleans up GPIO resources.
        """
        self.running = False
        self.thread.join()


    

class StepperController(Node):
    """
    ROS2 Node for controlling multiple PWM pins via a service.
    """
    def __init__(self):
        super().__init__('stepper_controller')
        self.speed_srv = self.create_service(ToggleStepper, 'toggle_stepper', self.handle_toggle_stepper)
        self.rotation_srv = self.create_service(ToggleStepper, 'rotate_stepper', self.handle_rotate_stepper)
        self.stepper_pins = {1: [4, 7, 17, 22], 2: [23, 24, 25, 26]}
        self.delay = .01
        self.steppers ={stepper: Stepper(self.stepper_pins[stepper], self.delay) for stepper in self.stepper_pins}
        self.get_logger().info("StepperController node is ready.")

    def handle_toggle_stepper(self, request, response):
        """
        Handles service requests to toggle a stepper.
        """
        stepper = request.stepperID
        speed = request.speed
        # self.get_logger().info(f"Received request to set pin {pin} to angle {angle}")
        
        if stepper in self.steppers:
            self.steppers[stepper].set_speed(speed)  # Update existing pin instance
        else:
            self.steppers[stepper] = Stepper(pin, self.pwm_frequency, default_angle=0, angle=speed)  # Create new pin instance
        
        response.success = True
        return response
    
    def handle_rotate_stepper(self, request, response):
        """
        Handles service requests to toggle a stepper.
        """
        stepper = request.stepperID
        rotations = request.speed
        # self.get_logger().info(f"Received request to set pin {pin} to angle {angle}")
        
        if stepper in self.steppers:
            self.steppers[stepper].set_rotation(rotations)  # Update existing pin instance
        else:
            self.steppers[stepper] = Stepper(pin, self.delay)  # Create new pin instance
            self.steppers[stepper].set_rotation(rotations)
        
        response.success = True
        return response

    def stop(self):
        for stepper in self.steppers:
            stepper.stop()

def main(args=None):
    """
    Initializes the ROS2 node and starts the service.
    """
    rclpy.init(args=args)
    stepper_controller = StepperController()
    
    try:
        rclpy.spin(stepper_controller)  # Keep node running
    except KeyboardInterrupt:
        stepper_controller.get_logger().info("Shutting down StepperController node.")
    finally:
        stepper_controller.stop()
        stepper_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
