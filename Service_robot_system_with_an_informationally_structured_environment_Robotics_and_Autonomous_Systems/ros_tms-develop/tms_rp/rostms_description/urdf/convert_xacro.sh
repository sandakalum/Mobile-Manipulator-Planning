# /bin/sh

cd $(dirname $0)
rosrun xacro xacro.py all_components_with_moveit.urdf.xacro > all_components_with_moveit.urdf
rosrun xacro xacro.py all_components.urdf.xacro > all_components.urdf
