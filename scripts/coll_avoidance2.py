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

class Coll_avoidance2:
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
        i = 0
        enc=False
        puntos=self.marker
        while(i<len(puntos) and not enc):
            if(math.fabs(puntos[i].x)<self.coll_distance and math.fabs(puntos[i].y)<0.3):
                enc=True
                #x=puntos[i].x
                #y=puntos[i].y
            else:
                i+=1
        return enc #,x,y

    def peligro_lat(self):
        i = 0
        enc=False
        puntos=self.marker
        while(i<len(puntos) and not enc):
            if(math.fabs(puntos[i].x)<self.coll_distance and math.fabs(puntos[i].y)>0.3 and math.fabs(puntos[i].y)<1):
                enc=True
                rospy.loginfo('X: %f, Y: %f',puntos[i].x, puntos[i].y)
                #x=puntos[i].x
                #y=puntos[i].y
            else:
                i+=1
        return enc #,x,y


    def publish(self):
        if(self.vel != None and self.marker != None):
            move_cmd = Twist()
            if(len(self.marker)==0):
                lin_vel = self.vel.linear.x
                angular = self.vel.angular.z
            else:
                if(self.peligro()):
                    rospy.loginfo('PELIGRO!!!!!!!')
                    angular=0.2
                    lin_vel=0
                elif(self.peligro_lat()):
                    rospy.loginfo('PELIGRO LATTTT!!!!!!!')
                    lin_vel=0.2
                    angular=0
                else:
                    rospy.loginfo('SEGUROO!!!!')
                    lin_vel = self.vel.linear.x
                    angular = self.vel.angular.z
            # let's go forward at 0.2 m/s
            move_cmd.linear.x = lin_vel
            # let's turn at 0 radians/s
            move_cmd.angular.z = angular
            self.cmd_vel.publish(move_cmd)



if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('coll_avoidance', anonymous=False)
        r = rospy.Rate(10);
        # Instantiate downsampler
        ca = Coll_avoidance2()
        while not rospy.is_shutdown():
          ca.publish()
          # wait for 0.1 seconds (10 HZ) and publish again
          r.sleep()
    except:
        rospy.loginfo("Error coll_avoidance.")
