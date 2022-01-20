#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import median

# Range Filter
def range_filter(scan, min, max):
    index = []
    for i, elem in enumerate(scan) :
        if elem < min:
            scan[i] = None
        elif elem > max:
            scan[i] = None
        else :
            index.append(i)
    return scan, index

# Median Deviation
def median_deviation(scan, window):
    result = []
    windows_num = len(scan)//window 
    mod = len(scan)%window
    
    for i in range(windows_num):
        temp = scan[window * i:window * (i+1)]
        med = median(temp)
        
        for j, elem in enumerate(temp):
            if (med - elem)/med*100 > 20:
                temp[j] = 10
        
        result.extend(temp)
    
    # Check if part of the scan is left excluded
    if (mod != 0):
        temp = scan[-mod:]
        med = median(temp)
        for j, elem in enumerate(temp):
            if (med - elem)/med*100 > 20:
                temp[j] = 10
        result.extend(temp)
    
    return result

def callback(msg):
        rate = rospy.Rate(15)
        scan = list(msg.ranges)
       
        res, index = range_filter(scan, min = msg.range_min, max = 1)
        not_none = []
        for i in index:
            not_none.append(res[i])
        
        not_none = median_deviation(not_none,10)

        for i in index:
            scan[i] = not_none.pop(0) 
        
        msg.ranges = tuple(scan)
        pub.publish(msg)  

rospy.init_node('noise_filtering')
sub = rospy.Subscriber('/scan_multi', LaserScan, callback)
pub = rospy.Publisher('/filtered_scans', LaserScan, queue_size=10)
 
move = Twist()
rospy.spin()
