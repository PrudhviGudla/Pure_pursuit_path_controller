#!/usr/bin/env python

from cmath import sqrt
from tabnanny import filename_only
from typing import final

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
import pandas as pd
import math
import rospy

look_ahead_dist = 2
track_width = 0.16

class purepursuit():

    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        self.publisher_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.i = 0
        self.subscription = rospy.Subscriber('odom', Odometry, self.cont_sig)
        rospy.spin()

    def cont_sig(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.current_vel = 0.5
        orientation = msg.pose.pose.orientation
        (roll,pitch,self.yaw) = self.euler_from_quaternion(orientation)
        sig = self.pure_pursuit()
        self.publisher_.publish(sig)
    
    def pure_pursuit(self):
        final_signal = Twist()
        min_ind = self.closest_point()
        x , y =self.look_ahead_point(min_ind)
        curv = self.get_curvature(self.yaw,self.x_pos,self.y_pos,x,y)
        left = (self.current_vel*(2+curv*track_width))/2
        right = (self.current_vel*(2-curv*track_width))/2
        vel = (left+right)/2
        omega = (left-right)/track_width
        final_signal.angular.z = float(omega)
        final_signal.linear.x = float(self.current_vel)
        return final_signal
        
    
    def closest_point(self):
        self.cx = [1,3,6,9]
        self.cy = [0,4,0,4]
        dist = []
        for i in range(len(self.cx)):
            val = math.sqrt((pow(self.x_pos-self.cx[i],2)+pow(self.y_pos-self.cy[i],2)))
            dist.append(val)
        min_ind = pd.Series(dist).idxmin()
        return min_ind
    
    def look_ahead_point(self,ind):
        E = np.array([self.cx[ind],self.cy[ind]])
        L = np.array([self.cx[ind+1],self.cy[ind+1]])
        C = np.array([self.x_pos,self.y_pos])
        r = look_ahead_dist
        t = 0
        d = L-E
        f = E-C
        k = f*d
        f = f*f
        d = d*d
        a = math.sqrt(d[0]+d[1])
        b = 2*(math.sqrt(abs(k[0])+abs(k[1])))
        c = math.sqrt(f[0]+f[1]) - (r*r)
        discriminant = b*b - 4*a*c
        if(discriminant<0):
            t = 0
        else:
            discriminant = math.sqrt(discriminant)
            t1 = (-b-discriminant)/(2*a)
            t2 = (-b+discriminant)/(2*a)
            if(t1>=0 and t1<=1):
                t = t1
            if(t2>=0 and t2<=1):
                t = t2
        P = E + t*d
        return P[0] , P[1]

    def euler_from_quaternion(self,quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def get_curvature(self,robot_angle,rx,ry,lx,ly):
        a = -1*math.tan(robot_angle)
        b = 1
        c = math.tan(robot_angle)*rx-ry
        val = math.sin(robot_angle)*(lx-rx)-math.cos(robot_angle)*(ly-ry)
        if val==0:
            side = 0
        elif val<0:
            side = -1
        else:
            side = 1
        x = abs(a*lx+b*ly+c)/math.sqrt((a*a+b*b))
        curv = (2*x)/(look_ahead_dist*look_ahead_dist)
        curv = side*curv
        return curv


if __name__ == '__main__':
    purepursuit()
