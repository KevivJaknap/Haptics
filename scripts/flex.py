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
        
    def convert(self):
        pass