#!/usr/bin/env python

# A basic tool that removes the laser points that are closer than a given distance
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

class ScanFilter:
    def __init__(self):
        self.min_dist = rospy.get_param("~min_dist", default=0.3)
        rospy.Subscriber("/base_scan", LaserScan, self.callback)
        self.scan_pub = rospy.Publisher("~scan_filtered", LaserScan, queue_size=10)

    def callback(self, data):
        data.ranges = list(data.ranges)
        for i in range(0,len(data.ranges)):

            if data.ranges[i] < self.min_dist:
                data.ranges[i] = np.Infinity
        self.scan_pub.publish(data)
    
 
if __name__ == '__main__':
    try:
	    # initiliaze
        rospy.init_node('ScanFilter', anonymous=False)

        # Instantiate downsampler
        ds=ScanFilter()

	    # Log initialization and spin
        rospy.loginfo("Scan Filter starting. min_dist = %f", ds.min_dist)
        rospy.spin()
    except:
        rospy.loginfo("ScanFilter node terminated.")
