#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

if __name__ == '__main__':
    
    path_publisher = rospy.Publisher('/path', Path, queue_size=10)
    rospy.init_node('path_publisher', anonymous=False)  
    seq = 0
    goals = sorted(rospy.get_param('/path').items())
    
    while not rospy.is_shutdown():
        path = Path()
        path.header.frame_id = rospy.get_param("~frame", default="map")
        path.header.stamp = rospy.Time.now()
        path.header.seq = seq
        seq_goals = 0
        for i in goals:
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.header.seq = seq_goals
            seq_goals += 1
            pose.pose.position.x = i[1]['x']
            pose.pose.position.y = i[1]['y']
            pose.pose.position.z = 0
            
            path.poses.append(pose)
            rospy.loginfo("Added point %f, %f", pose.pose.position.x, pose.pose.position.y)
        path_publisher.publish(path)
        rospy.loginfo("Path enviado!\n")
            
        seq = seq + 1

        for i in range(5):
            rospy.sleep(1)