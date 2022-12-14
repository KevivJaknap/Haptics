#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu 
from std_msgs.msg import Int16
import message_filters
import simple_pid

class Main:
    def __init__(self):
        rospy.init_node('main', anonymous=True)
        self.pub = rospy.Publisher('pwm_main', Int16, queue_size=10)
        self.imu_h = message_filters.Subscriber('imu_h', Imu)
        self.imu_m = message_filters.Subscriber('imu_m', Imu)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.imu_h, self.imu_m], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.data = ""
        
    def callback(self, imu_h, imu_m):
        rospy.loginfo(f"imu_h={imu_h} imu_m={imu_m}")
        
    
    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.pub.publish(self.data)
            rate.sleep()
    
if __name__ == '__main__':
    motor1 = Main()
    Main.run()