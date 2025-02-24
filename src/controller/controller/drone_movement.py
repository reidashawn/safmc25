import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from geometry_msgs.msd import Twist
from std_msgs.msg import Float32, Int32
from std_srvs.srv import SetBool
from interfaces.srv import SetFloat
import os

class DroneMovement(Node):
    def __init__(self):
        super().__init__('drone_movement')

        # Subscribe to pot topic
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.hor_vel_sub = self.create_subscription(Twist, '/right/cmd_vel_hor', self.hor_vel_callback, 10)
        self.yaw_vel_sub = self.create_subscription(Float32, '/right/cmd_vel_vert', self.yaw_vel_callback, 10)
        # self.vert_vel_sub = self.create_subscription(Float32, '/left/cmd_vel_vert', self.vert_vel_callback, 10)
        self.vert_srv = self.create_service(SetFloat, '/vert_vel', self.vert_vel_srv_callback)
        self.lock_srv = self.create_service(SetBool, '/lock_axis', self.lock_axis_callback)
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = 0
        self.angular_z = 0
        self.axis_lock = False
        
    def publish_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.linear.z = self.linear_z
        msg.angular.z = self.angular_z
        self.pub.publish_message(msg)
        return
    
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
        self.linear_z = request.float 
        response.success = True
        self.publish_vel()
        return response
    
    def lock_axis_callback(self, request, response):
        self.axis_lock = request.data
        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    drone_movement = DroneMovement()
    rclpy.spin(drone_movement)
    drone_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
