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
        self.marker = mark

    def vel(self, vel):
        self.vel=vel

    def peligro(self):
        enc=False
        i=0
        while(not(enc) and i<len(self.marker.points)):
            dist=math.sqrt(pow(self.marker.points[i].x,2)+pow(self.marker.points[i].y,2))
            rospy.loginfo("Distancia de colision: %f", dist)
            if(dist<self.coll_distance):
                enc=True
            i+=1
        return enc

    def publish(self):

        if(self.vel != None and self.marker != None):
            lin_vel = self.vel.linear.x
            angular = self.vel.angular.z
            if (self.peligro()):
                rospy.loginfo('PELIGRO!!!!!!!')
                lin_vel = 0
                angular = 0.2
            # Twist is a datatype for velocity
            move_cmd = Twist()
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
        ca = Coll_avoidance()
        while not rospy.is_shutdown():
          ca.publish()
          # wait for 0.1 seconds (10 HZ) and publish again
          r.sleep()
    except:
        rospy.loginfo("Error coll_avoidance.")
