#!/bin/bash

FIRST_NODE_NUM=1
LAST_NODE_NUM=19

#new terminal
taskset -c 0 xterm -e bash -c 'roscore' &
sleep 1


RESOLUTION_ARRAY=(0.2 0.4 0.6 0.8 1.0)
STEP_SIZE_ARRAY=(0.05 0.1 0.15 0.2)
TRANSFORMATION_EPSILON_ARRAY=(0.01,0.02,0.03)
MAXIMUM_ITERATIONS_ARRAY=(30)
LEAF_SIZE_ARRAY=(0.2 0.4 0.6 0.8 1.0)
MINIMUM_SCAN_RANGE_ARRAY=(5)
MAXIMUM_SCAN_RANGE_ARRAY=(200)
MINIMUM_ADD_SCAN_SHIFT_ARRAY=(1)
SEQUENCE_ARRAY=(0,1,2,3,4,5,6,7,8,9,10)

NODE_NUM=$FIRST_NODE_NUM

rosparam set /first_node_num $FIRST_NODE_NUM
rosparam set /last_node_num $LAST_NODE_NUM

for RESOLUTION in $RESOLUTION_ARRAY
do 
  for STEP_SIZE in $STEP_SIZE_ARRAY
  do 
    for TRANSFORMATION_EPSILON in $TRANSFORMATION_EPSILON_ARRAY
    do 
      for MAXIMUM_ITERATIONS in $MAXIMUM_ITERATIONS_ARRAY
      do 
        for LEAF_SIZE in $LEAF_SIZE_ARRAY
        do 
          for MINIMUM_SCAN_RANGE in $MINIMUM_SCAN_RANGE_ARRAY
          do 
            for MAXIMUM_SCAN_RANGE in $MAXIMUM_SCAN_RANGE_ARRAY
            do 
              for MINIMUM_ADD_SCAN_SHIFT in $MINIMUM_ADD_SCAN_SHIFT_ARRAY
              do

                if [ $NODE_NUM < $LAST_NODE_NUM ]
                  rosparam set /ndt_mapping$NODE_NUM/resolution $RESOLUTION
                  rosparam set /ndt_mapping$NODE_NUM/step_size $STEP_SIZE
                  rosparam set /ndt_mapping$NODE_NUM/transformation_epsilon $TRANSFORMATION_EPSILON
                  rosparam set /ndt_mapping$NODE_NUM/maximu_iterations $MAXIMUM_ITERATIONS
                  rosparam set /ndt_mapping$NODE_NUM/leaf_size $LEAF_SIZE
                  rosparam set /ndt_mapping$NODE_NUM/minimum_scan_range $MINIMUM_SCAN_RANGE
                  rosparam set /ndt_mapping$NODE_NUM/maximum_scan_range $MAXIMUM_SCAN_RANGE
                  rosparam set /ndt_mapping$NODE_NUM/minimum_add_scan_shift $MINIMUM_ADD_SCAN_SHIFT
                  NODE_NUM=$(( $NODE_NUM + 1 ))
                  continue

                # after launchig all nodes
                else
                  NODE_NUM=$FIRST_NODE_NUM
                  #new terminal
                  taskset -c 0 xterm -e bash -c 'cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun mypkg param_optimization' &
                  sleep 1
                  # sequence loop
                  for SEQUENCE in $SEQUENCE_ARRAY
                  do
                    ## play NDT_Mapping nodes
                    for ((i=$FIRST_NODE_NUM; i<$LAST_NODE_NUM;i++))
                    do
                      taskset -c $i xterm -e bash -c "cd ~/slam_kitti_ws/methods/Autoware/ros; source devel/setup.bash; roslaunch lidar_localizer ndt_mapping (${i}th copy).launch"
                      sleep 1
                    done  
                    sleep 5

                    # play kitti_player
                    if [ $SEQUENCE -lt 10 ] ;then
                      taskset -c 0 xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d /media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/0$SEQUENCE --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 5 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
sleep 1
                    else
                      taskset -c 0 xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d /media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/$SEQUENCE --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 5 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
sleep 1
                    fi

                    # wait finishing error calculation
                    for ((i=$FIRST_NODE_NUM; i<$LAST_NODE_NUM;i++))
                    do
                      while [ true ]
                      do
                        tmp=$(rosparam get /done_error_calculation${i})
                        if [ $tmp -eq 1 ] ;then
                        rosparam set /done_error_calculation${i} 0
                          break
                        fi 
                        sleep 0.1
                      done                    
                    done

                  # play next sequence                  
                  done
                rosparam set /fin_all_sequence 1
                # fi of launching nodes
                fi
              # next parameters
              done
            done
          done
        done
      done
    done
  done
done









##

