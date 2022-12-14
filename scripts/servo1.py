#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, Int16

rospy.init_node('pubsub')
class Main:
    def __init__(self):
        
        self.sub = rospy.Subscriber('imu_machine', Imu, self.callback)
        self.pub_servo1 = rospy.Publisher('servo1', Int16, queue_size=10)
        self.pub_dc = rospy.Publisher('dcmotor', Int16, queue_size=10)
        self.rate = rospy.Rate(10)
        self.angle = 0
    
    def callback(self, imu_h):
        rospy.loginfo(f"imu_machine={imu_h}")
        #convert imu data to angle
        z = imu_h.orientation.z
        y = imu_h.orientation.y
        self.dc_angle = int(125*(y-0.72) + 90)
        self.angle = int(125*(z-0.72) + 90)
        
    def run(self):
        while not rospy.is_shutdown():
            self.pub_servo1.publish(self.angle)
            self.rate.sleep()

if __name__ == "__main__":
    servo = Main()
    servo.run()