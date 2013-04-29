#!/usr/bin/env python
# Copyright (c) 2012, Falkor Systems, Inc.  All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  THIS SOFTWARE IS
# PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('PredatorPrey')

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
import pid
import time

class ArdroneControl:
    def __init__( self ):
        self.nav_sub = rospy.Subscriber( "ardrone/navdata", Navdata, self.callback_navdata )
        self.takeoff_sub = rospy.Subscriber( "ardrone/takeoff", Empty, self.callback_takeoff )
        self.land_sub = rospy.Subscriber( "ardrone/land", Empty, self.callback_land )
        self.reset_sub = rospy.Subscriber( "ardrone/reset", Empty, self.callback_reset )
        self.goal_vel_sub = rospy.Subscriber( "goal_vel", Twist, self.callback_goal_vel )
        self.cmd_vel_pub = rospy.Publisher( "cmd_vel", Twist )
        

        self.takenOff = False

        # gain_p, gain_i, gain_d
        self.linearxpid = pid.Pid2( 0.5, 0.0, 0.5 )
        self.linearypid = pid.Pid2( 0.5, 0.0, 0.5 )

        self.vx = self.vy = self.vz = self.ax = self.ay = self.az = 0.0
        self.last_time = None
        self.last_takeoff = None
        self.goal_vel = Twist()

        self.last_cmd_vel = Twist()
        self.cmd_filter_const = 1

    def callback_goal_vel( self, data ):
        self.goal_vel = data

    def callback_navdata( self, data ):
        self.vx = data.vx/1e3
        self.vy = data.vy/1e3
        self.vz = data.vz/1e3

    def callback_takeoff( self, foo ):
        self.takenOff = True
        self.last_takeoff = rospy.Time.now()

    def callback_land( self, foo ):
        self.takenOff = False

    def callback_reset( self, foo ):
        self.takenOff = False

        # these accelerations must take into account orientation, so we get acceleration relative
        # to base_stabilized. This does not.
#        self.ax = (data.ax*9.82)
#        self.ay = (data.ay*9.82)
#        self.az = (data.az - 1.0)*9.82

    def update( self ):
        if self.takenOff and (rospy.Time.now() - self.last_takeoff).to_sec() > 3:
            if self.last_time == None:
                self.last_time = rospy.Time.now()
                dt = 0.0
            else:
                time = rospy.Time.now()
                dt = ( time - self.last_time ).to_sec()
                self.last_time = time
                
            #self.last_cmd_vel.angular.y = 0
            #self.last_cmd_vel.angular.x = 0

            #self.last_cmd_vel.angular.z = self.goal_vel.angular.z
            #self.last_cmd_vel.linear.z  = self.goal_vel.linear.z
            #self.last_cmd_vel.linear.x  = self.linearxpid.update( self.goal_vel.linear.x, self.vx, dt )
            #rospy.logdebug("I'm here, YEEEEEEHHHHHHHHAAAAAAAAA")
            #self.last_cmd_vel.linear.y  = 0#self.linearypid.update( self.goal_vel.linear.y, self.vy, dt )

            self.last_cmd_vel.angular.z = self.filter(self.last_cmd_vel.angular.z, self.goal_vel.angular.z)
            self.last_cmd_vel.linear.z = self.filter(self.last_cmd_vel.linear.z, self.goal_vel.linear.z)
            self.last_cmd_vel.linear.x = self.filter(self.last_cmd_vel.linear.x, self.goal_vel.linear.x)
            #self.goal_vel.linear.y = self.filter(self.last_cmd_vel.linear.y, self.linearypid.update( self.goal_vel.linear.y, self.vy, dt ))


            self.cmd_vel_pub.publish( self.last_cmd_vel )
        else:
            return

    def filter(self,old,new):
        val = old*(1-self.cmd_filter_const) + new*self.cmd_filter_const
        if abs(val) < 1e-9:
            val = 0
        return val

def main():
  rospy.init_node( 'ardrone_control' )
  
  control = ArdroneControl()
  r = rospy.Rate(10)

  try:
      while not rospy.is_shutdown():
        control.update()
        r.sleep()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
