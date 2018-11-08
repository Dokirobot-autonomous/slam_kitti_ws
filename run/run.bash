#!/bin/bash

#new terminal
xterm -e bash -c 'roscore' &
sleep 1

#new terminal
xterm -e bash -c 'cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun mypkg param_optimization' &
sleep 2

exist_param_optimization=$(rosparam get /exist_param_optimization)
current_sequence=$(rosparam get /current_sequence)
start_mapping=$(rosparam get /start_mapping)

##
while [ $exist_param_optimization -eq 1 ]
do

while [ $start_mapping -eq 0 ]
do
start_mapping=$(rosparam get /start_mapping)
sleep 1
done

#new terminal 4
xterm -e bash -c 'cd ~/slam_kitti_ws/methods/Autoware/ros; source devel/setup.bash; roslaunch lidar_localizer ndt_mapping.launch' &
sleep 3

if [ $current_sequence -lt 10 ] ;then
#new terminal 2

xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d /media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/0$current_sequence --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 0.5 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
sleep 1

else
#new terminal 2
xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d /media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/$current_sequence --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 0.5 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
sleep 1

fi

start_mapping=0
rosparam set /start_mapping $start_mapping

done


##

