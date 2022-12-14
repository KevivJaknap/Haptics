#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Float32, Int16

rospy.init_node('flex_pubsub')

class Main():
    def __init__(self):
        self.sub = rospy.Subscriber('flex_res', Float32, self.callback)
        self.pub = rospy.Publisher('flex_angle', Int16, queue_size=10)
        self.rate = rospy.Rate(10)
        self.res = 0.0
        self.ang = 0
        self.res0 = 52000          # resistance at 0 degrees
        self.res90 = 150000        # resistance at 90 degrees
        
    def convert(self):
        ang = int(90*(self.res-self.res0)/(self.res90-self.res0))     # linear conversion
        self.ang = max(min(ang, 90), 0)
        
    def callback(self, msg):
        self.res = msg.data
        self.convert()
    
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(f"resistance={self.res}, angle={self.ang}")
            self.pub.publish(self.ang)
            self.rate.sleep()

if __name__ == "__main__":
    flex = Main()
    flex.run()