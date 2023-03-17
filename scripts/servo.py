#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu 

class Main():
    
    def __init__(self):
        
        rospy.init_node('servo_main')
        self.sub = rospy.Subscriber('imu_machine', Imu, self.callback)
        self.pub = rospy.Publisher('servo_angle', Int16, queue_size=10)
        
        self.rate = rospy.rate(10)
        self.angle = 0
    
    def callback(self, imu):
        z = imu.orientation.z
        self.angle = int(125*(z-0.72)+90)
        rospy.loginfo(f"Servo main angle: {self.angle}")
        
    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.s_angle)
            self.rate.sleep()

if __name__ == "__main__":
    servo = Main()
    servo.run()