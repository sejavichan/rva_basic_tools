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

class Coll_Avoidance_Pot:
    def __init__(self):
        rospy.Subscriber('/cmd_vel_coll', Twist, self.vel)
        rospy.Subscriber('/down/marker', Marker, self.callback)
        self.max_linear = rospy.get_param('~max_linear_speed')
        self.min_linear = rospy.get_param('~min_linear_speed')
        self.max_angular = rospy.get_param('~max_angular_speed')
        self.min_linear = rospy.get_param('~min_linear_speed')
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel=None
        self.marker=None
        self.tolerancia_angulo = rospy.get_param('~tolerancia_angulo')
        self.tolerancia_obs = rospy.get_param('~tolerancia_obs')
        self.k = rospy.get_param('~k')
        self.tol_dist_vel = rospy.get_param('~tol_dist_vel')

    def callback(self, mark):
        self.marker = mark.points

    def vel(self, vel):
        self.vel=vel
    
    def campo_pot(self, v):
        puntos=self.marker
        v_pot=0
        for i in range(0, len(puntos)):
            dist = math.sqrt(math.pow(puntos[i].x, 2) + math.pow(puntos[i].y, 2))
            if (dist < self.tolerancia_obs):
                v_lin, v_ang = self.convert_esc(v)
                if(dist<self.tol_dist_vel):
                     rospy.loginfo('Peligro!!!!!!!')
                     v_lin = v_lin*dist
                     v_ang += v_ang/dist
                     v = self.convert_vect(v_lin,v_ang)
                punto=puntos[i]
                mod=math.sqrt(math.pow(punto.x,2)+math.pow(punto.y,2))
                punto_arr = np.array([punto.x, punto.y, 0.0])
                v_pot += self.k*(punto_arr/math.pow(mod,2))
                #v_pot += self.k * (punto_arr / mod)
        v -= v_pot
        return v


    def convert_esc(self,v):
        lin_vel = math.sqrt(math.pow(v[0], 2) + math.pow(v[1], 2))
        angular = math.atan2(v[1], v[0])
        return lin_vel, angular

    def convert_vect(self, lineal, angular):
        vx = lineal * math.cos(angular)
        vy = lineal * math.sin(angular)
        v = np.array([vx, vy, 0.0])
        return v


    def publish(self):
        move_cmd = Twist()
        if(self.vel != None and self.marker != None):
            lin_vel = self.vel.linear.x
            angular = self.vel.angular.z

            if (lin_vel > 0):
                v = self.convert_vect(lin_vel, angular)
                v = self.campo_pot(v)
                lin_vel, angular = self.convert_esc(v)
                if(lin_vel>self.max_linear):
                    lin_vel=self.max_linear
                if (lin_vel < self.min_linear):
                    lin_vel = self.min_linear
                if(math.fabs(angular)>self.tolerancia_angulo):
                    #lin_vel = 0.07
                    if(angular>0):
                        angular=self.max_angular
                    else:
                        angular=-self.max_angular
            move_cmd.linear.x = lin_vel
            move_cmd.angular.z = angular
            rospy.loginfo('v_lineal: %f v_angular: %f', lin_vel, angular)
            self.cmd_vel.publish(move_cmd)

    
    def shutdown(self):
       # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)



if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('coll_avoidance', anonymous=False)
        r = rospy.Rate(10)
        # Instantiate downsampler
        ca = Coll_Avoidance_Pot()
        while not rospy.is_shutdown():
          ca.publish()
          # wait for 0.1 seconds (10 HZ) and publish again
          r.sleep()
          
    except:
        rospy.loginfo("Error Coll_Avoidance_Pot.")
