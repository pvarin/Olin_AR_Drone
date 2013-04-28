#!/usr/bin/env python
import roslib
roslib.load_manifest('ARDronePTAM')
import sys
import rospy
import cv
import pid
import std_srvs.srv
import dynamic_reconfigure.client
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
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

		#cv.ShowImage("Image window", cv_image)
		#cv.WaitKey(3)

		#data.encoding = "MONO8"
		#self.imgPublisher.publish(data)

		try:
			newimage = self.bridge.cv_to_imgmsg(cv_image, "mono8")
			newimage.header.frame_id = 'camera'
			self.imgPublisher.publish(newimage)
		except CvBridgeError, e:
			print e

class control:

	def __init__(self):
		self.velPublisher = rospy.Publisher( "cmd_vel", Twist)
		self.takeoffPublisher = rospy.Publisher( "ardrone/takeoff", Empty)
		self.landPublisher = rospy.Publisher( "ardrone/land", Empty)
		self.navdataSubscriber = rospy.Subscriber("ardrone/navdata", Navdata, self.navdata_cb)

		self.autoInit = False
		self.timer = rospy.Timer( rospy.Duration( 0.10 ), self.timer_cb, False )

		self.navdata = None
		self.zPid = pid.Pid2( 0.0005, 0.0, 0.0005)


		rospy.wait_for_service('ardrone/flattrim')
		self.flattrim = rospy.ServiceProxy( "ardrone/flattrim", std_srvs.srv.Empty )
		self.flattrim()

		self.client = dynamic_reconfigure.client.Client("ptam_visualizer", timeout=5)

	def navdata_cb(self, data):
		self.navdata = data

	def toggleAutoInit(self):
		self.autoInit = not self.autoInit
		self.startingHeight = self.navdata.altd

	def timer_cb(self, event):
		cmd_vel = Twist()

		if event.last_real == None:
			dt = 0
		else:
			dt = ( event.current_real - event.last_real ).to_sec()

		if self.autoInit:
			if abs(self.startingHeight + 800 - self.navdata.altd) > 100:
				cmd_vel.linear.z = self.zPid.update( self.startingHeight + 800, self.navdata.altd, dt )
			else:
				self.toggleAutoInit()
		
		self.velPublisher.publish( cmd_vel )


def main(args):
  rospy.init_node('find_door')
  ie = imgEcho()
  rospy.sleep(7)
  c = control()
  rospy.sleep(1)
  #c.takeoffPublisher.publish(Empty())
  #rospy.sleep(5)
  #c.toggleAutoInit()

  c.client.update_configuration({"ShowPC":True})
  #c.client.update_configuration({"ExportPC":True})

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)