#!/usr/bin/env python
import roslib; roslib.load_manifest('CommandVelocitySpammer')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def talker():
  pub = rospy.Publisher('cmd_vel', Twist)
  rospy.init_node('spammer')
  while not rospy.is_shutdown():
    pub.publish(Twist())
    rospy.sleep(.05)


if __name__ == '__main__':
  try:
     talker()
  except rospy.ROSInterruptException:
     pass