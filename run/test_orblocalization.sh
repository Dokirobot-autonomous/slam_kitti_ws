#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 1

#new terminal 2
xterm -e bash -c 'rosrun map_server map_server /home/ohashi/tmp/180926/map.yaml' &
sleep 1

#new terminal 3
xterm -e bash -c 'cd ~/dataset/rosbag; rosbag play -s 20 2018-09-26-10-29-52.bag' &
sleep 5

#new terminal 4
xterm -e bash -c 'rosrun rviz rviz' &
sleep 1

#new terminal 5
xterm -e bash -c 'cd ~/localization_ws; source devel/setup.bash; roslaunch orb_localizer orb_matching.launch'
sleep 1

