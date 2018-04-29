# TimeSeriesAnalyzer
Scripts to check if IMU and image data are synchronized.

In most open source visual-inertial odometry packages/libraries, the most common failure case is the synchonization between IMU and camera data. Synchronization means that the difference in header times of succeeding IMU and images messages should be constant. Here we provides a simple script to check if the devices are synchronized.

Input: a ROS bag file recording your IMU and image data. Please change the topics of IMU and camera accordingly.
Output: a plot showing the time series of incoming data.

Example:
Time series output of EuRoC MH_01_easy
![alt text](https://github.com/jason2718/TimeSeriesAnalyzer/master/Example/EuRoC_time_series.png)

Time series output of an dataset with incorrect synchronization 
![alt text](https://github.com/jason2718/TimeSeriesAnalyzer/master/Example/Incorrect_Synchronization.png)
