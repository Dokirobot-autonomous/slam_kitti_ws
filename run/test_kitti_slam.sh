#!/bin/bash

#new terminal 1
xterm -e bash -c 'roscore' &
sleep 1

##new terminal 4
#xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws; source devel/setup.bash; roslaunch lidar_localizer ndt_mapping_tku.launch" &
#sleep 1

#new terminal 2
#xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d ~/dataset/rosbag/Kitti/odometry/dataset/sequence/0${SEQ} --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 100 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
#sleep 1

#new terminal 3
xterm -e bash -c 'cd ~/slam_kitti_ws/methods/limo; source devel_limo_release/setup.bash; roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch' &
sleep 1

##new terminal 4
#xterm -e bash -c 'cd ~/slam_kitti_ws/methods/orb_slam_ros; roslaunch orb_slam_ros orb_slam_stereo.launch' &
#sleep 1

##new terminal 4
#xterm -e bash -c '~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun slam_fusion slam_fusion' &
#sleep 1

#new terminal 2
xterm -e bash -c 'cd ~/dataset/rosbag/Kitti; rosbag play 00.bag -r 0.1 --pause --clock /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw' 
sleep 1

sleep 5

#new terminal 4
#xterm -e bash -c 'rosrun rviz rviz'
#sleep 1
