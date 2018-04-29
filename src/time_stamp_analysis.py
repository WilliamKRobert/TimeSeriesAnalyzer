#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
import cv2

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LastMsgType(Enum):
    IMU = 1
    IMAGE = 2
    NOMSG = 3

class TimeSeriesAnalyzer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam0/image_raw", Image, self.imgCallback)
        self.imu_sub = rospy.Subscriber("/imu0", Imu, self.imuCallback)

        self.avg_pub = rospy.Publisher('/avg_imu_img_time_offset', Float64, queue_size=1)
        self.std_pub = rospy.Publisher('/std_imu_img_time_offset', Float64, queue_size=1)


        self.ref_time = rospy.get_rostime() 
        self.is_first = True

        self.last_msg = LastMsgType.NOMSG 

        self.num_image_msg = 0
        self.avg_imu_img_offset = 0
        self.std_imu_img_offset = 0
        self.current_offset = 0
        self.imu_timestamp = np.array([])
        self.img_timestamp = np.array([])

        self.imu_msg_time = None
        self.img_msg_time = None
        
        self.imu_line = None
        self.image_line = None

        
        plt.ion() # turn interactive on
        plt.show()
         

    def imuCallback(self, data):
        '''
        Draw a vertical line every time we receive an IMU measurement
        '''
        global axis_start, axis_end
        global last_t

        if self.is_first == True:
            self.ref_time = data.header.stamp
            self.is_first = False
    
            last_t = 0
    
            axis_start = 0 
            axis_end = 0.09
            plt.axis([axis_start, axis_end, 0, 1])
    
        data_time = data.header.stamp
        t = data_time - self.ref_time 
        self.imu_msg_time = t.to_sec()
        
        self.imu_line = plt.axvline(self.imu_msg_time, ymax=0.5, label='IMU')
        plt.legend([self.imu_line, self.image_line], ['IMU', 'Image'])
    
        if self.imu_msg_time >= 0.8 * axis_end:
            delta_t = self.imu_msg_time - last_t
        
            axis_start += delta_t 
            axis_end += delta_t 
            plt.axis([axis_start, axis_end, 0, 1])
        
        plt.draw()
        plt.pause(0.000000001)
    
        self.imu_timestamp = np.append(self.imu_timestamp, self.imu_msg_time)
    
        last_t = self.imu_msg_time 
        self.last_msg = LastMsgType.IMU
    
    
    def imgCallback(self, data):
        '''
        Draw a vertical line every time we receive an Image measurement
        '''

        if self.is_first == True:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 
        except CvBridgeError as e:
            print(e)
   
        data_time = data.header.stamp
        t = data_time - self.ref_time 
        self.img_msg_time = t.to_sec()
        
        self.image_line = plt.axvline(self.img_msg_time, ymax=0.8, color='red', label='Image')
        plt.legend([self.imu_line, self.image_line], ['IMU', 'Image'])
    
        #plt.draw()
        #plt.pause(0.000000001)

        self.img_timestamp = np.append(self.img_timestamp, self.img_msg_time) 

        self.last_msg = LastMsgType.IMAGE

    

if __name__ == '__main__':
        
    rospy.init_node('time_series_analyzer', anonymous=True)

    tsa = TimeSeriesAnalyzer()

       
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

