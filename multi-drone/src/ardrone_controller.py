#!/usr/bin/env python

# basic ardrone controller class

# import ros libs and load manifest file for package
import roslib; roslib.load_manifest('multi-drone')
import rospy

# import relevant ros messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_srvs import srv.Empty
from ardrone_autonomy.msg import Navdat

#drone status enum
from drone_status import DroneStatus

COMMAND_PERIOD = 100 #ms

class DroneController(object):

	def __init__(self, namespace=''):

		# holds drone status
		self.status = -1

		#subscribe to navdata
		self.subNavdata = rospy.Subscriber(namespace+'/ardrone/navdata',Navdata,self.ReceiveNavdata)

		#allow takeoff, land, and e-reset
		self.pubLand    = rospy.Publisher(namespace+'/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher(namespace+'/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher(namespace+'/ardrone/reset',Empty)

 		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher(namespace+'/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Create Flat trim service proxy
		rospy.wait_for_service(namespace+'/ardrone/flattrim')
		try:
			self.srvFlatTrim = rospy.ServiceProxy(namespace+'/ardrone/flattrim', srv.Empty)
		except:
			rospy.ServiceException, e:
			print "Service called failed: %s" % e

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetFlatTrim(self):
		#call flat trim
		self.srvFlatTrim()

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)