#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 3

rosparam set /ndt_mapping_tku/resolution 2.0
rosparam set /ndt_mapping_tku/leaf_size 0.6
sleep 1
#new terminal 4
xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws; source devel/setup.bash; roslaunch lidar_localizer ndt_mapping_tku.launch" &
sleep 1

#sleep 10

##new terminal 2
#xterm -e bash -c 'cd ~/dataset/rosbag/Kitti; rosbag play 01.bag -r 0.1 --pause --clock /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw'
#sleep 1

##new terminal 2
#SEQ=01
#xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d ~/dataset/rosbag/Kitti/odometry/dataset/sequence/$SEQ --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 1 /sensor/camera/color_label/left/image_rect:=/sensor/camera/color_labels/left/image_rect /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw"
#sleep 1

#new terminal 2
#SEQ=02
#xterm -e bash -c "cd ~/dataset/rosbag/Kitti/odometry/dataset/sequence/; rosbag play $SEQ.bag -r 0.1 -s 192 --pause --clock /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw"
#sleep 1

#new terminal 2
SEQ=07
xterm -e bash -c "cd ~/dataset/rosbag/Kitti/odometry/dataset/sequence/; rosbag play $SEQ.bag -r 0.1 -s 65 --pause --clock /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw"
sleep 1


##new terminal 4
#xterm -e bash -c 'rosrun rviz rviz'
#sleep 1
