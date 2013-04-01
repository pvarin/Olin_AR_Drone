#!/usr/bin/env python

import roslib
roslib.load_manifest('ARDronePTAM')

import rospy
from sensor_msgs.msg import Image

class imgEcho(object):
	def __init__(self):
		self.imgPublisher = rospy.Publisher( "camera/image_raw", Image )
		self.imgSubscriber = rospy.Subscriber( "ardrone/front/image_raw", Image, self.echoImageCallback )


	def echoImageCallback(self,data):
		self.imgPublisher.publish(data)
		pass

if __name__ == "__main__":
	rospy.init_node('test')
	imgEchoer = imgEcho()

	rospy.spin()