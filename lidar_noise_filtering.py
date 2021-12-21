#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from statistics import median

# Crops all the values that are below_min/above_max
# and replaces them with the defined min_value/max_value
# Range Filter
def range_filter(scan, min, max):
    temp = []
    index = []
    for i, elem in enumerate(scan) :
        if elem < min:
            temp.append(None)
        elif elem > max:
            temp.append(None)
        else :
            temp.append(elem)
            index.append(i)
    return temp, index

# Replaces every value of the scan by calculating the median of the neighboring values
# The number of neighboring values is defined by window.
# Alternative sol: Using Mean instead of Median.
def serial_median(scan, window):
    #scan = list(scan)
    for i in range(window, len(scan)):
        scan[i] = median(scan[i-window:i])
    #scan = tuple(scan)
    return scan

# Returns the median of the current and the previous scans.
# Previous scans are saved in a queue. 
def temporal_median(scans, scan_length):
    medians = []
    for i in range(scan_length):
        medians.append(median(scan[i] for scan in scans))
    #medians = tuple(medians)
    return medians

# Method to update queue and keep specified window size, 
def queue_upd(scan, window):
        queue.append(scan)
        if len(queue) == window:
                queue.pop(0)

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
                temp[j] = None
        result.extend(temp)
    if (mod != 0):
        temp = scan[-mod:]
        med = median(temp)
        for j, elem in enumerate(temp):
            if (med - elem)/med*100 > 20:
                temp[j] = None
        result.extend(temp)
    return result

# Z-score
# Take Z-score for certain areas and not the whole scan
def z_score (scan):
    temp = scan
    mn = mean(temp)
    std = stdev(temp)
    for i, elem in enumerate(temp) :
        z = (elem - mn)/std
        if abs(z)>2.5 :
            temp[i] = None
    return temp

def z_score2 (scan,window):
    result = []
    windows_num = len(scan)//window 
    mod = len(scan)%window
    
    for i in range(windows_num):
        temp = scan[window * i:window * (i+1)]
        mn = mean(temp)
        std = stdev(temp)
        for i, elem in enumerate(temp) :
            z = (elem - mn)/std
            if abs(z)>2.5 :
                temp[i] = None
        result.extend(temp)
    if (mod != 0):
        temp = scan[-mod:] #.copy()
        mn = mean(temp)
        std = stdev(temp)
        for i, elem in enumerate(temp) :
            z = (elem - mn)/std
            if abs(z)>2.5 :
                temp[i] = None
        result.extend(temp)
    return result

def callback(msg):
        rate = rospy.Rate(15)
        print(type(msg.ranges))
# -----------------------------Range Filter (Min/Max)-----------------------------
       
        #res, index = range_filter(msg.ranges, msg.range_min, 1)

# -----------------------------Serial Median--------------------------------------
       
        #msg.ranges = serial_median(msg.ranges, 5) #window needs to be defined

# -----------------------------Temporal Median (Optimal)-----------------------------
      
        queue_upd(msg.ranges, 5) #15scans in 1sec. Keeping the 5 last of them seems good
        msg.ranges = temporal_median(queue, len(scan)) #you can replace len(scan) with the actual side of the lidar reading

        pub.publish(msg)  

queue = []  
rospy.init_node('noise_filtering')
sub = rospy.Subscriber('/scan_multi', LaserScan, callback) #/scan_multi ?
pub = rospy.Publisher('/filtered_scans', LaserScan, queue_size=10)
 
move = Twist()
rospy.spin()
