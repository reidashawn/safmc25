import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from interfaces.srv import SetFloat
import time
import threading

class DroneMovement(Node):
    def __init__(self):
        super().__init__('drone_movement')

        # Subscribe to pot topic
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.hor_vel_sub = self.create_subscription(Twist, '/right/cmd_vel_hor', self.hor_vel_callback, 10)
        self.yaw_vel_sub = self.create_subscription(Float32, '/right/cmd_vel_vert', self.yaw_vel_callback, 10)
        # self.vert_vel_sub = self.create_subscription(Float32, '/left/cmd_vel_vert', self.vert_vel_callback, 10)
        self.vert_srv = self.create_service(SetFloat, '/vert_vel', self.vert_vel_srv_callback)
        self.last_vert_time = time.time()
        self.last_lock_time = time.time()
        self.lock_srv = self.create_service(SetBool, '/lock_axis', self.lock_axis_callback)
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = 0
        self.angular_z = 0
        self.axis_lock = False
        self.thread = threading.Thread(target=self.thread_callback, daemon=True)  # Daemon thread to toggle PWM
        
    def publish_vel(self):
        msg = Twist()
        msg.linear.x = float(self.linear_x)
        msg.linear.y = float(self.linear_y)
        msg.linear.z = float(self.linear_z)
        msg.angular.z = float(self.angular_z)
        self.pub.publish(msg)
        return
    
    def thread_callback(self):
        if time.time() - self.last_lock_time > .5:
            self.axis_lock = False
        if time.time() - self.last_vert_time > .2:
            self.linear_z = 0
        self.publish_vel()

    def hor_vel_callback(self, data):
        if self.axis_lock:
            if data.linear.x > data.linear.y:
                self.linear_x = data.linear.x
                self.linear_y = 0
            else:
                self.linear_y = data.linear.y
                self.linear_x = 0
            self.angular_z = 0
        else:
            self.linear_y = data.linear.y
            self.linear_x = data.linear.x
        self.publish_vel()


    def yaw_vel_callback(self, data):
        if self.axis_lock:
            self.angular_z = 0
        else:
            self.angular_z = data.data
        self.publish_vel()

    def vert_vel_srv_callback(self, request, response):
        self.linear_z = request.data
        response.success = True
        self.publish_vel()
        self.last_vert_time = time.time()
        return response
    
    def lock_axis_callback(self, request, response):
        self.axis_lock = request.data
        response.success = True
        self.last_lock_time = time.time()
        return response



def main(args=None):
    rclpy.init(args=args)
    drone_movement = DroneMovement()
    rclpy.spin(drone_movement)
    drone_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
