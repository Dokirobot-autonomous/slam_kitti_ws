#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 1

#new terminal 2
xterm -e bash -c 'rosrun map_server map_server /home/ohashi/tmp/180926/map.yaml' &
sleep 1

##new terminal 3
#xterm -e bash -c 'cd ~/localization_ws; source devel/setup.bash; roslaunch orb_localizer orb_mapping.launch' &
#sleep 1

#new terminal 4
xterm -e bash -c 'cd ~/dataset/rosbag; rosbag play 2018-09-26-10-42-06.bag'
sleep 1

##new terminal 5
#xterm -e bash -c 'rosrun rviz rviz'
#sleep 1


