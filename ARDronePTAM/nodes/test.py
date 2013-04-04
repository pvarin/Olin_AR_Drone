#!/usr/bin/env python
import roslib
roslib.load_manifest('ARDronePTAM')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class imgEcho:

	def __init__(self):
		self.imgPublisher = rospy.Publisher( "camera/image_raw", Image )
		self.imgSubscriber = rospy.Subscriber( "ardrone/front/image_raw", Image, self.echoImageCallback )
		cv.NamedWindow("Image window", 1)
		self.bridge = CvBridge()

	def echoImageCallback(self,data):


		try:
			cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
		except CvBridgeError, e:
			raise e

		cv.ShowImage("Image window", cv_image)
		cv.WaitKey(3)

		#data.encoding = "MONO8"
		#self.imgPublisher.publish(data)

		try:
			self.imgPublisher.publish(self.bridge.cv_to_imgmsg(cv_image, "mono8"))
		except CvBridgeError, e:
			print e

def main(args):
  ie = imgEcho()
  rospy.init_node('test')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)