#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

if __name__ == '__main__':
    
    path_publisher = rospy.Publisher('/publish_path', Path, queue_size=10)
    
    rospy.init_node('path_publisher', anonymous=False)
    
    #arg = str(sys.argv[1])
    
    while not rospy.is_shutdown():
		path = Path()
        arg = input('Escribe un nuevo path. Ejemplo "2,-1;1,-2": \n')
        
        for punto in arg.split(";"):
            x = float(punto.split(",")[0])
            y = float(punto.split(",")[1])
            
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
        
            path.poses.append(pose)
            
        path_publisher.publish(path)
        print("Path enviado!\n")