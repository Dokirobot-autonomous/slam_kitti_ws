#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 3

#SEQ_ARRAY=(04 05 06 07 08 09 10)
SEQ_ARRAY=(00)

for SEQ in ${SEQ_ARRAY[@]}
do
  #new terminal 2
  xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d /media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/$SEQ --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 1 /sensor/camera/color_label/left/image_rect:=/sensor/camera/color_labels/left/image_rect /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw"
  sleep 1 
  cd /media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/$SEQ
#  mv ${SEQ}.bag ${SEQ}_190107.bag
  mv test.bag ${SEQ}.bag
done

