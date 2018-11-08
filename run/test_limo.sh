#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 1

#new terminal 2
xterm -e bash -c 'cd ~/dataset/rosbag/Kitti; rosbag play 01.bag -r 0.1 --pause --clock' &
sleep 1

#new terminal 3
xterm -e bash -c 'cd ~/localization_kitti_ws/withous_catkin/limo; source devel_limo_release/setup.bash; roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch' &
sleep 1

#new terminal 4
xterm -e bash -c 'rosrun rviz rviz'
sleep 1

