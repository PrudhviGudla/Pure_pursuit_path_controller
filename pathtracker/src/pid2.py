#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

set_vel = 0.5

kp = 1
ki = 0.01
kd = 0.5

class pid():     
    def __init__(self):
        rospy.init_node('pid2', anonymous=True)
        self.publisher_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.previous_err = 0
        self.total_err = 0
        self.subscription = rospy.Subscriber('odom', Odometry, self.cont_sig)
        rospy.spin()
        
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

if __name__ == '__main__':
    pid()
    
