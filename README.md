# src
Usage:

1-open a terminal and cd to Simulator

2-(only after building/catkin_make) ```gedit devel/setup.bash``` and add these 2 lines to the file with your Simulator path:

```sh
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/Simulator/src:$ROS_PACKAGE_PATH"
```

3-```source devel/setup.bash```

Run simulator:

```roslaunch sim_pkg map_with_car.launch```
(you can replace "map_with_car.launch" with different launch files in sim_pkg/launch and/or modify them to add objects in the simulator)

Run controller or function nodes:

open new terminal with "Usage" (cd to Simulator and ```source devel/setup.bash```)

All controller nodes: ```roslaunch control car_control.launch``` (you can edit the launch file to add/remove ros nodes)

Any other functions: ```rosrun control *.py``` (any python file in control/scripts or any c++ files in control/src without .cpp)

(ex: ```rosrun control lane.py```
ex: ```rosrun control lane```)

Files structures in src:
control: our controller package
-launch: launch files for controller function
-models: yolo models for object detection
-msg: custom ros messages used by controller functions
-scripts: controller functions in python
-src: controller functions in c++
-srv: custom ros services used by controller functions

example: given demo code
-its src has control.py and camera.py to demo keyboard control and camera

models_pkg: sdf models used in the simulator

plugins_pkg: code to simulate the components of the car (gps, motor and servo commands...)

sim_pkg: contains simulator worlds and launch files for them

traffic_light_pkg: idk

utils: custom ros messages and services
