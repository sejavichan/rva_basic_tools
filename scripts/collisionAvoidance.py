#!/usr/bin/env python
import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path


class CollisionAvoidance():

    def __init__(self):
        self.listener = tf.TransformListener()
        
        # Velocidad max lineal en metros (TODO: coger de parametro)
        self.max_linear_speed = 0.2
        # Velocidad max de giro en metros
        self.max_angular_speed = 0.25
        
        # Subscribers: scan to see the obstacles; then cmd_vel_follower to get the commands from the follower
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/cmd_vel_follower", Twist, self.cmd_vel_callback) # Get the desired commands from the follower

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Publish the safe commands to the robot

        # Setup the status flags
        self.cmd_vel_data = None
        self.laser_init = False

    #Cuando se reciban datos del laser del robot se guardaran en los atributos de clase
    def laser_callback(self, laser_data):
        self.laser_data = laser_data
        self.laser_init = True

    def cmd_vel_callback(self, cmd_vel_data):
        #----- Eliminar para que los puntos sean acumulables
        self.cmd_vel_data = cmd_vel_data

    
    def command(self):
        rospy.loginfo("Command")

        angular = 0.0
        linear = 0.0
        if self.laser_init and self.cmd_vel_data is not None:
            rospy.loginfo_once("Collision Avoidance: Laser and commands received")
        
            #Establecer limite de velocidad (TODO: complete)
            if linear > self.max_linear_speed:
                linear = self.max_linear_speed


            rospy.loginfo(angular)
        
        self.publish(linear,angular)

    def publish(self,lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel_pub.publish(move_cmd)
        
    def shutdown(self):
        rospy.loginfo("Stop Collision Avoidance")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        rospy.init_node('collision_avoidance', anonymous=False)

        ca_instance = CollisionAvoidance()

        rospy.on_shutdown(ca_instance.shutdown)

        #La frecuencia de actualizacion del bucle es de 10 Hz
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            ca_instance.command()
            r.sleep()

    except:
        rospy.loginfo("Collision avoidance node terminated.")
