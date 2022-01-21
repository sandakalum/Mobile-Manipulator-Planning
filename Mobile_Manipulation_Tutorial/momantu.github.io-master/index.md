---
layout: default
---

# Welcome to the <u>Mo</u>bile <u>Man</u>ipulation <u>Tu</u>torial (MoManTu). 

This is the webpage for the tutorial in the (journal to be announced). Find the overview paper here (link to open access paper provided once it is accepted) and the detailed tutorial here (dito), while the code is hosted on [https://github.com/momantu](https://github.com/momantu).

**The tutorial is still under development! Please use with caution.**

The tutorial teaches how to program a mobile robot with a robot arm to do mobile manipulation. The example systems are a [Fetch](https://fetchrobotics.com/robotics-platforms/) robot and a [Clearpath Jaca](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) with a [Kinova Jaco](https://www.kinovarobotics.com/en/products/gen2-robot) attached - see the image below.

<div id="cen">
<img src="/imgs/robots.jpg" alt="MoManTu Robots" width="50%"/><br>
Figure 1: The robots used in the MoManTu: Fetch and Jackal + Kinova <br>
(top left and right) and their simulated versions below.
</div>

&nbsp;  

The tutorial is developed by the [Mobile Autonomous Robotic Systems Lab](https://robotics.shanghaitech.edu.cn/) (MARS Lab) and the [Living Machines Lab](http://lima.sist.shanghaitech.edu.cn/) (LIMA Lab) of the [ShanghaiTech Automation and Robotics Center](http://star-center.shanghaitech.edu.cn/) (STAR Center), [School of Information Science and Technology](http://sist.shanghaitech.edu.cn/) (SIST) of [ShanghaiTech University](https://www.shanghaitech.edu.cn/). 

We welcome comments, suggestions and potentially even code contributions to the tutorial. Please see [Getting Help](#getting-help) to contact us.

Below two videos show the demo on a real and a simulated Fetch robot:

<video width="100%" controls>
  <source src="/videos/real_fetch_web.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

<video width="100%" controls>
  <source src="/videos/sim_fetch_web.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

&nbsp;  

# Docker

The easiest way to get started with playing with the tutorial is to use the provided Docker image. 
    
To pull the docker image and run the demo use:

```bash
docker run --name momantu -i -t -p 6900:5900 -e RESOLUTION=1920x1080 yz16/momantu
```

Then open a VNC client (e.g. [RealVNC](https://www.realvnc.com/en/connect/download/viewer)) and connect to _127.0.0.1:6900_. The workspaces are located in the root folder and ready to use. 

## Run MoManTu

Launch the simulator and the MoManTu software by running this command in the terminal:

```bash
roslaunch fetch_sim demo.launch
```

The pose estimation module based on NOCS, which runs under a conda environment of Python 3.5.
Activate the environment and launch the object pose estimation module with:
```bash
conda activate NOCS #activate environment
source ~/nocs_ws/devel/setup.bash
roslaunch nocs_srv pose_estimation_server.launch
```

Finally, launch the flexbe_app and robot_server node to connect other modules and start the demo: 
```bash
roslaunch fetch_sim robot_service.launch
```

# Sources

The sources for the MoManTu are on GitHub: [https://github.com/momantu](https://github.com/momantu).

The MoManTu Fetch demo is found here: [https://github.com/momantu/momantu_fetch](https://github.com/momantu/momantu_fetch)

The MoManTu Jackal Kinova demo is todo.

## Online Packages:

| Role         | Package           | URL  |
|:-------------|:------------------|:------|
| Localization | AMCL | [link](http://wiki.ros.org/amcl)  |
| Local Costmap | ROS Navigation | [link](http://wiki.ros.org/navigation) |
| Path Planning | ROS Navigation | [link](http://wiki.ros.org/navigation) |
| Path Following | ROS Navigation | [link](http://wiki.ros.org/navigation) |
| Arm Control 1 | kinova_ros  | [link](https://github.com/Kinovarobotics/kinova-ros) |
| Arm Control 2 | fetch_ros  | [link](https://github.com/fetchrobotics/fetch_ros) |
| Category Detection & Pose | NOCS | [link](https://github.com/momantu/nocs_ros)  |
| Object Detection | Pose | (todo) |
| Object Place Pose | AprilTag_ROS  | [link](http://wiki.ros.org/apriltag_ros) |
| Grasp Planning | / | / |
| Arm Planning 1 & IK | MoveIt Pick and Place | [link](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html)  |
| Arm Planning 2 & IK | MoveIt directly (todo) | [link](https://moveit.ros.org/) |
| Human Robot Interaction 1 | RViz & FlexBE App| [link](http://wiki.ros.org/rviz) |
| Human Robot Interaction 2 | Speech (todo) | (todo) |
| Decision Making | FlexBE | [link](http://wiki.ros.org/flexbe)  |

## Offline Packages:

| Role         | Package           | URL  |
|:-------------|:------------------|:------|
| Simulation World Building |  Gazebo  | [link](http://gazebosim.org/) |
| SLAM 2D | Cartographer | [link](http://wiki.ros.org/cartographer) |
| SLAM 3D | Cartographer (todo)| [link](http://wiki.ros.org/cartographer) |
| Camera Calibration | camera_calibration | [link](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) |
| Camera Pose Detection | AprilTag_ROS | [link](https://github.com/AprilRobotics/apriltag_ros) |
| Hand-eye Calibration | easy_handeye | [link](https://github.com/IFL-CAMP/easy_handeye) |


# Getting Help

You are welcome to post issues on the [momantu fetch](https://github.com/momantu/momantu_fetch/issues) repo to get help. You may also send an email to [Hou Jiawei](mailto:houjw@shanghaitech.edu.cn?subject=[GitHub]%20MoManTu) or [Prof. Sören Schwertfeger](mailto:soerensch@shanghaitech.edu.cn?subject=[GitHub]%20MoManTu).

# Sensor Log

A ROS bagfile with the sensor data from a real fetch run is [avaiable](https://robotics.shanghaitech.edu.cn/static/datasets/MoManTu/fetch_real.bag) (3.4GB).

# Citing

Please cite our paper if you use the tutorial in your research.

(bibtex citation to be provided)


