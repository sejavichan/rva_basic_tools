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
        self.coll_distance=2
        self.marker=None

    def callback(self, mark):
        self.marker = mark.points

    def vel(self, vel):
        self.vel=vel

    def peligro(self,v_li,w):
        i = 0
        enc=False
        puntos=self.marker
        vx=v_li*math.cos(w)
        vy=v_li*math.sin(w)
        v=np.array([vx,vy])        
        while(i<len(puntos) and not enc):
            dist=math.sqrt(math.pow(puntos[i].x,2)+math.pow(puntos[i].y,2))
            rospy.loginfo('LLEGO??')
            if(math.fabs(dist)<self.coll_distance):
                
                v-=self.campo_pot(v,puntos[i])
                enc=True
            else:
                i+=1
        return enc,v
    
    def campo_pot(self,v, punto):
        k=0.4
        mod=math.sqrt(math.pow(punto.x,2)+math.pow(punto.y,2))
        v_pot=k*(punto/mod)
        return v_pot
        


    def publish(self):
        if(self.vel != None and self.marker != None):
            move_cmd = Twist()
            if(len(self.marker)==0):
                lin_vel = self.vel.linear.x
                angular = self.vel.angular.z
            else:
                enc,v=self.peligro(self.vel.linear.x,self.vel.linear.y)
                if(enc):
                    rospy.loginfo('PELIGRO!!!!!!!')
                    lin_vel=math.sqrt(math.pow(v.x,2)+math.pow(v.y,2))
                    angular=math.atan2(v.y,v.x)
                else:
                    rospy.loginfo('SEGUROO!!!!')
                    lin_vel = self.vel.linear.x
                    angular = self.vel.angular.z
            # let's go forward at 0.2 m/s
            move_cmd.linear.x = lin_vel
            # let's turn at 0 radians/s
            move_cmd.angular.z = angular
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
        r = rospy.Rate(10);
        # Instantiate downsampler
        ca = Coll_avoidance2()
        while not rospy.is_shutdown():
          ca.publish()
          # wait for 0.1 seconds (10 HZ) and publish again
          r.sleep()
          
    except:
        rospy.loginfo("Error coll_avoidance_pot.")
