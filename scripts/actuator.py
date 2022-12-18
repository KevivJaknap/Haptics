#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from simple_pid import PID

rospy.init_node('actuator')

class Main():
    
    def __init__(self):
        self.dist = 0
        self.dist0 = 153
        self.dist_min = 80
        self.ang0 = 0
        self.ang_min = -20
        self.sub = rospy.Subscriber('act_dist', Int16, self.callback)
        self.sub_imu = rospy.Subscriber('imu_machine', Imu, self.callback_imu)
        self.pub = rospy.Publisher('actuator', Int16, queue_size=10)
        self.pid = PID(1, 0, 0, setpoint=0)
        self.control = 0
        self.imu_angle = 0
        self.rate = rospy.Rate(10)
        self.ang_dist = 0
        self.error = 0
        self.output = 0
        
    def callback_imu(self, imu):
        imu_val = imu.orientation.y
        self.imu_angle = int(125*(imu_val-0.72) + 90)
        
    def callback(self, dist):
        self.dist = dist.data
        self.ang_dist = self.extrapolate(self.dist_min, self.dist0, self.ang_min, self.ang0, self.dist)
        self.error = self.ang_dist - self.imu_angle
        self.control = self.pid(self.error)
        if -10 < self.control < 10:
            self.output = 0
        elif self.control > 10:
            self.output = 1
        else:
            self.output = -1
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(f"error: {int(self.error)}, control: {int(self.control)}")
            self.pub.publish(self.output)
            self.rate.sleep()
    
    def extrapolate(self, x1, x2, y1, y2, x):
        return y1 + (x - x1)*(y2 - y1)/(x2 - x1)

if __name__ == "__main__":
    main = Main()
    main.run()
    
        