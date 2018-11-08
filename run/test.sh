#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 1

#new terminal 2
#xterm -e bash -c 'rosrun map_server map_server /home/ohashi/tmp/180926/map.yaml' &
#sleep 1

#new terminal 3
xterm -e bash -c 'cd ~/dataset/rosbag; rosbag play -s 30 2018-09-26-10-29-52.bag' &
sleep 1

#new terminal 4
#xterm -e bash -c 'cd ~/catkin_ws; source devel/setup.bash; roslaunch gnss_localizer nmea2tfpose.launch' &
#sleep 1

#new terminal 5
xterm -e bash -c 'rosrun rviz rviz'
sleep 1


