# Semi-autonomous Behaviour Tree-Based Framework For Sorting Electric Vehicle Batteries Waste

This package provides  all the relevant ROS code to execute and simulate the task of collecting and sorting waste from an Electric Vehicle Battery pack.


## Required
#### Catkin tools

Instructions to install Catkin Tools availabe in  its [documentation](https://catkin-tools.readthedocs.io/en/latest/installing.html)
#### Wstool
```
$ sudo apt-get install python-wstool
```
#### Rosdep
```
$ sudo apt install python-rosdep
$ sudo rosdep init
$ rosdep update
```
#### Other Deps:  
```
$ sudo apt-get install libglfw3-dev
$ sudo apt-get install libzmq3-dev libboost-dev
```
#### Intel Realsense SDK Library
Intel realsense library is required, see instructions from Intel's [repository](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)




## Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9.0


#### Behaviour Trees C++ Library
```
$ sudo apt-get install ros-melodic-behaviortree-cpp-v3
```

#### Installing repo
1. Insert COLLADA models used in this work to Gazebo models global path
```
$ echo "export GAZEBO_MODEL_PATH=~/RELIB-KMR-Bin-Picking/gazebo_models" >> ~/.bashrc
```
2. Installing ROS Control
```
$ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```
3. Clone repo and build. Note: that this will also install Moveit! to your catkin_ws therefore,  it may take several minutes to build.
```
$ cd ~/
$ git clone --recurse-submodules https://github.com/uob-erl/RELIB-KMR-Bin-Picking.git
$ cd ~/RELIB-KMR-Bin-Picking/relib_ws
$ wstool update -t src
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release 
$ catkin build -j8
```
3. Source workspace.
```
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ echo "source ~/RELIB-KMR-Bin-Picking/relib_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
## Running Simulation

It is highly recommended to install Tmux to have several terminal windows simultaneously.
```
$ sudo apt-get update
$ sudo apt-get install tmux
```

In different terminal windows run the ROS launch files using the kuka.sh script in the following order:

1. Loading enviorement, and Robot controls. Here two workstation worlds can be loaded: workstation_1 and workstation_2.

```
$ ~/RELIB-KMR-Bin-Picking/kuka.sh  spawn true eband true workstation_1
```

2. Loading Moveit! config for KUKA KMR iiwa robot without Rviz and with the OMPL planning library.
```
$ ~/RELIB-KMR-Bin-Picking/kuka.sh moveit false ompl
```

3. Run all the action nodes
```
$ ~/RELIB-KMR-Bin-Picking/kuka.sh actions
```

4. Run Object Tracking
```
$ ~/RELIB-KMR-Bin-Picking/kuka.sh tracker
```

5. To exectue the full collection task run:
```
# The order of collection can be swaped if desired
$ rosrun bt_kuka bt_collector -1 battery_module -2 plate_a -3 plate_b -4 bracket
```
6. Alternative, to collect just one type of Object run:
```
$ rosrun bt_kuka bt_def [object_name]
```