# LiDAR Noise Filtering

Scripts developed in order to filter noise in [LaserScans](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), gathered by TALOS' LiDAR sensors. This is done by extracting the scan transmited by `/scan_multi`, applying the designed filters, reconstructing the LaserScan message and publishing it to `/filtered_scans` topic.

## Messages 

All messages types are [sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) with the following definition:
```
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```
## Range Filter

Common practice in the following scripts is using a `Range Filter` before applying their core filter. We apply this practice in order to focus on close proximity noise which creates most of our problems. `Range Filter` replaces values that are below min or above max with None. It returns the filtered scan and the indices of the not None values. An example is being displayed below:

<img src="Images/original_scan.png" alt="original_scan" width="480" height="480"/> <img src="Images/range_filter.png" alt="range_filter" width="480" height="480"/>

## Median Deviation

`median_deviation()` divides received scan in regions defined by window (number of rays). For each region the algorith calculates the median and then checks if any element of the region deviates from it more than 20%. In that case replace the element value with an extreme one eg. 10m. Then it returns the filtered scan.
The algorith
