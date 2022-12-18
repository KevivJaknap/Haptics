#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from simple_pid import PID

class Main():
    
    def __init__(self):
        self.dist = 0
        self.dist0 = 153
        self.dist_min = 80
        self.ang0 = 0
        self.ang_min = -20
        self.sub = rospy.Subscriber('act_dist', Int16, self.callback)
        self.sub_imu = rospy.Subscriber('imu', Imu, self.callback_imu)
        self.pub = rospy.Publisher('actuator', Int16, queue_size=10)
        self.pid = PID(1, 0, 0, setpoint=0)
        self.control = 0
        self.imu_angle = 0
        self.rate = rospy.Rate(10)
        
    def callback_imu(self, imu):
        self.imu_angle = imu.orientation.x
        
    def callback(self, dist):
        ang_dist = self.ang_min*(self.dist - self.dist0)/(self.dist_min - self.dist0)
        self.control = self.pid(ang_dist - self.imu_angle)
        
    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.control)
            self.rate.sleep()
        