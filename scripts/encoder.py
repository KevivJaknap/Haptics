#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64, Int16
from simple_pid import PID

rospy.init_node('encoder')

class Main():
    
    def __init__(self):
        
        self.target = 0 #initial target value
        self.angle_enc = 0 #initial angle value
        self.zero = 0 #initial zero value
        self.ninety = 32000 #initial 90 degree = 20000 lines
        
        self.sub = rospy.Subscriber('encoder', Int64, self.callback)
        self.pub = rospy.Publisher('pwm_dc', Int16, queue_size=10)
        self.pid = PID(4, 0.3, 0, setpoint=0)
        self.control = 0
        
        self.rate = rospy.Rate(10)
        self.error = 0
        
    def callback(self, encoder):
        self.angle_enc = self.extrapolate(self.zero, self.ninety, 0, 90, encoder.data)
        self.error = self.target - self.angle_enc
        if -2<=self.error<=2:
            self.error = 0
        control = int(self.pid(self.error))
        self.control = self.clip(control, -40, 40)
        if -20<=self.control<=20:
            self.control = 0
    
    def clip(self, value, min_value, max_value):
        return min(max_value, max(min_value, value))
    
    def run(self):
        self.target = int(input("Enter target angle: "))
        while not rospy.is_shutdown():
            rospy.loginfo(f"error: {int(self.error)}, control: {int(self.control)}")
            self.pub.publish(self.control)
            self.rate.sleep()

    
    def extrapolate(self, x1, x2, y1, y2, x):
        return y1 + (x - x1)*(y2 - y1)/(x2 - x1)

if __name__ == "__main__":
    main = Main()
    main.run()