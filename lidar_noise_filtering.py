#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import median

# Crops all the values that are below_min/above_max
# and replaces them with the defined min_value/max_value
def range_filter(scan, min, max):
    for i in range(len(scan)):
        if scan[i] < min:
            scan[i] = min
        elif scan[i] > max:
            scan[i] = max
    return scan

# Replaces every value of the scan by calculating the median of the neighboring values
# The number of neighboring values is defined by window.
def serial_median(scan, window):
    for i in range(len(scan) - window):
        scan[i] = median(scan[i:i+window])
    return scan

# Returns the median of the current and the previous scans.
# Previous scans are saved in a queue. 
def temporal_median(scans, scan_length):
    medians = []
    for i in range(scan_length):
        medians.append(median(scan[i] for scan in scans))
    return medians

# Method to update queue and keep specified window size, 
def queue_upd(scan, window):
        queue.append(scan)
        if len(queue) == window:
                queue.pop(0)

def callback(msg):
        rate = rospy.Rate(15)

# -----------------------------Range Filter (Min/Max)-----------------------------
       
        msg.ranges = range_filter(msg.ranges, 0.06, 4.5) #0.06 as min seems good

# -----------------------------Serial Median--------------------------------------
       
        msg.ranges = serial_median(msg.ranges, 5) #window needs to be defined

# -----------------------------Temporal Median (Optimal)-----------------------------
      
        queue_upd(msg.ranges, 5) #15scans in 1sec. Keeping the 5 last of them seems good
        msg.ranges = temporal_median(queue, len(scan))

queue = []  
rospy.init_node('noise_filtering')
sub = rospy.Subscriber('/scan_1', LaserScan, callback) #/scan_multi ?
# Scan needs to be published
 
move = Twist()
rospy.spin()