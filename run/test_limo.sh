#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 2

#new terminal 2
xterm -e bash -c 'cd ~/dataset/rosbag/Kitti; rosbag play 01.bag -r 0.1 --pause --clock /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw' &
sleep 1

#new terminal 4
xterm -e bash -c 'rosrun rviz rviz' &
sleep 1

#new terminal 3
cd ~/slam_kitti_ws/methods/limo; source devel_limo_release/setup.bash; roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch
sleep 1


