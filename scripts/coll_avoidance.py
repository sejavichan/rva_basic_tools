#!/usr/bin/env python

# A basic tool that downsamples a radar scan by taking one point for each n
import math

import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

class Coll_avoidance:
    def __init__(self):
        rospy.Subscriber('/cmd_vel_coll', Twist, self.vel)
        rospy.Subscriber('/down/marker', Marker, self.callback)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel=None
        self.coll_distance=1
        self.marker=None

    def callback(self, mark):
        self.marker = mark.points

    def vel(self, vel):
        self.vel=vel

    def peligro(self):
        enc=False
        i = 0
        x = 0
        y = 0
        while(not(enc) and i<len(self.marker) and len(self.marker)!=0):
            dist=math.sqrt(pow(self.marker[i].x,2)+pow(self.marker[i].y,2))
            rospy.loginfo("Distancia de colision: %f", dist)
            if(dist<self.coll_distance):
                enc=True
                x=self.marker[i].x
                y=self.marker[i].y
            else:
                i+=1
        return enc, x, y

    def publish(self):
        if(self.vel != None and self.marker != None):
            # Twist is a datatype for velocity
            move_cmd = Twist()
            lin_vel = self.vel.linear.x
            angular = self.vel.angular.z
            rospy.loginfo('No entiendo Vel_lineal: %f Vel_ang: %f', lin_vel, angular)
            pel, x, y=self.peligro()
            if(pel):
                rospy.loginfo('PELIGRO!!!!!!!')
            else:
                rospy.loginfo('NO PELIGRO')
            while(pel):
                rospy.loginfo('ENTRAAA????')
                pel, x, y = self.peligro()
                rospy.loginfo('LLEGAAAA????')
                dist=math.sqrt(pow(x,2)+pow(y,2))
                if(dist<0.4):
                    lin_vel = 0
                else:
                    lin_vel = dist*0.2
                alpha=math.atan2(y,x)
                angular = (alpha*0.2)+(math.pi/2)
                # let's go forward at 0.2 m/s
                move_cmd.linear.x = lin_vel
                # let's turn at 0 radians/s
                move_cmd.angular.z = angular
                rospy.loginfo('Vel_lineal: %f Vel_ang: %f',lin_vel, angular)
                self.cmd_vel.publish(move_cmd)
            # let's go forward at 0.2 m/s
            move_cmd.linear.x = lin_vel
            # let's turn at 0 radians/s
            move_cmd.angular.z = angular
            rospy.loginfo('ERROR?? Vel_lineal: %f Vel_ang: %f', lin_vel, angular)
            self.cmd_vel.publish(move_cmd)

if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('coll_avoidance', anonymous=False)
        r = rospy.Rate(10);
        # Instantiate downsampler
        ca = Coll_avoidance()
        while not rospy.is_shutdown():
          ca.publish()
          # wait for 0.1 seconds (10 HZ) and publish again
          r.sleep()
    except:
        rospy.loginfo("Error coll_avoidance.")
