#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/eric/groovy_workspace/Olin_AR_Drone/ethzasl_ptam/ptam_com/build/catkin_generated', type 'exit' to leave"
  . "/home/eric/groovy_workspace/Olin_AR_Drone/ethzasl_ptam/ptam_com/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/eric/groovy_workspace/Olin_AR_Drone/ethzasl_ptam/ptam_com/build/catkin_generated'"
else
  . "/home/eric/groovy_workspace/Olin_AR_Drone/ethzasl_ptam/ptam_com/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
