#!/bin/sh

if [ $1 = 'spawn' ]
then
	NAV=$2
	PLANNER=$3
	RVIZ=$4
	WORLD=$5
	NAME='def10'
	if [ $WORLD = 'workstation_1' ]
	then
		NAME='def10'
	elif [ $WORLD = 'workstation_2' ]
	then
		NAME='def11'
	fi
		
	roslaunch kuka_kmr_iiwa_description spawn.launch navigation:=$NAV l_planner:=$PLANNER rviz:=$RVIZ world_name:=$NAME
elif [ $1 = 'moveit' ]
then
	RVIZ=$2
	PLANNER=$3
	roslaunch kuka_kmr_iiwa_config kuka_moveit.launch rviz:=$RVIZ pipeline:=$PLANNER
elif [ $1 = 'actions' ]
then
	if [ -n $2 ]
	then
		SUCTION_POINTS=$2
	else
		SUCTION_POINTS=9
	fi
	roslaunch kuka_kmr_iiwa_tasks run_actions.launch suction_n:=9
elif [ $1 = 'tracker' ]
then
	roslaunch object_tracking tracker.launch
fi