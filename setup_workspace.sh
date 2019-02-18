#!/bin/bash
if [[ -z $1 ]]; then
  echo "You must provide a rosinstall file!"
  exit
fi
ROSINSTALLFILE=$(cd $(dirname "$1") && pwd -P)/$(basename "$1")
echo "Initializing catkin workspace from $ROSINSTALLFILE ... (strg + c to cancel)"
sleep 5
mkdir catkin_ws
cd catkin_ws
wstool init src $ROSINSTALLFILE
echo "Building packages..."
catkin build
return_value=$?
if [ $return_value -eq 1 ]; then
  echo "Build failed"
  exit 1
fi
echo
echo "Remember to source the build files!"
