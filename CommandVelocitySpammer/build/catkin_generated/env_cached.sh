#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/patrick/ROS_ws/Olin_AR_Drone/CommandVelocitySpammer/build/catkin_generated', type 'exit' to leave"
  . "/home/patrick/ROS_ws/Olin_AR_Drone/CommandVelocitySpammer/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/patrick/ROS_ws/Olin_AR_Drone/CommandVelocitySpammer/build/catkin_generated'"
else
  . "/home/patrick/ROS_ws/Olin_AR_Drone/CommandVelocitySpammer/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
