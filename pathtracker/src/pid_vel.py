#!/usr/bin/env python

from tabnanny import filename_only
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

set_vel = 0.5

kp = 1
ki = 0.01
kd = 0.5

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.previous_err = 0
        self.total_err = 0
        self.subscription = self.create_subscription(Odometry,'odom',self.cont_sig,10)

    def cont_sig(self, msg):
        self.vel = msg.twist.twist.linear.x
        sig = self.pid_vel()
        self.publisher_.publish(sig)

    def pid_vel(self):
        vel = self.vel
        err = (set_vel-vel)
        final_signal = Twist()
        diff_err = err - self.previous_err

        sig = kp*err + ki*self.total_err + kd*diff_err

        self.total_err = self.total_err + err
        self.previous_err = err

        final_signal.linear.x = float(sig)

        return final_signal

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
