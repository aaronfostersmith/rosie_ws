#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient
roslaunch rosie_gazebo rosie_world.launch
