#!/usr/bin/env python
import sys
import math
import numpy as np  

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry



#TO DO: Tune parameters
#PARAMS 
VELOCITY = 2.0 # meters per second
MIN_ACC = 1.0

class EmergencyStop:
    def __init__(self):
        #Topics & Subs, Pubs
        self.stop_signal = 0
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback)#: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd', AckermannDriveStamped, queue_size=10)#: Publish to drive

    def callback_vel(self,data):
        #TO DO: Subscribe Current Velocity

    def callback(self, data):
        #TO DO: 1. Subscribe LiDAR data 
        #       2. Calculate minimum distance 
        #       2. Calculate Time to Collision(TTC) based on current velocity
        #       3. Publish drive.speed. (If TTC is smaller than minimum time, stop the car )
        #       
        #       Example:
        #               drive_msg = AckermannDriveStamped()
        #               drive_msg.header.stamp = rospy.Time.now()
        #               drive_msg.header.frame_id = "laser"
        #               drive_msg.drive.speed = 0
        #               self.drive_pub.publish(drive_msg)
        
    
def main(args):
    rospy.init_node("Emergengy_Stop", anonymous=False)
    ES = EmergencyStop()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)