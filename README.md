# src
Usage:
1-open a terminal and cd to Simulator
2-(only after building/catkin_make) $gedit devel/setup.bash and add these 2 lines to the file with your Simulator path:
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/Simulator/src:$ROS_PACKAGE_PATH"
3-$source devel/setup.bash

Run simulator:
$roslaunch sim_pkg map_with_car.launch
(you can replace "map_with_car.launch" with different launch files in sim_pkg/launch and/or modify them to add objects in the simulator)

Run controller or function nodes:
open new terminal with "Usage"
controller: $roslaunch control car_control.launch (you can edit the launch file to add/remove functions)
any other functions: $rosrun control *.py (any python file in control/scripts or any c++ files in control/src without .cpp)
(ex: $rosrun control lane.py
ex: $rosrun control lane)