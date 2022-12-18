#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, Int16

rospy.init_node('pubsub')
class Main:
    def __init__(self):
        
        self.sub = rospy.Subscriber('imu_machine', Imu, self.callback)
        self.pub_servo1 = rospy.Publisher('servo1', Int16, queue_size=10)
        self.pub_dc = rospy.Publisher('stepper', Int16, queue_size=10)
        self.rate = rospy.Rate(10)
        self.angle = 0
        self.s_angle = 0
    
    def callback(self, imu_h):
        #convert imu data to angle
        z = imu_h.orientation.z
        x = imu_h.orientation.x
        s_angle = (125*(x-0.72) + 90)*(5/9)
        self.s_angle = int(s_angle)           #stepper angle
        self.angle = int(125*(z-0.72) + 90)
        rospy.loginfo(f"stepper:{self.s_angle}")
        
    def run(self):
        while not rospy.is_shutdown():
            self.pub_servo1.publish(self.angle)
            self.pub_dc.publish(self.s_angle)
            self.rate.sleep()

if __name__ == "__main__":
    servo = Main()
    servo.run()