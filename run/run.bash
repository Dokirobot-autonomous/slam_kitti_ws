#!/bin/bash

FIRST_SEQUENCE=0
LAST_SEQUENCE=10


#RESOLUTION_ARRAY=(1.0 1.2 1.4 1.6 1.8 2.0)
#STEP_SIZE_ARRAY=(0.1 0.05 0.15 0.2)
#TRANSFORMATION_EPSILON_ARRAY=(0.01 0.02 0.03)
MAXIMUM_ITERATIONS_ARRAY=(30)
#LEAF_SIZE_ARRAY=(0.2 0.4 0.5 0.6 0.8)
MINIMUM_SCAN_RANGE_ARRAY=(5)
MAXIMUM_SCAN_RANGE_ARRAY=(200)
MINIMUM_ADD_SCAN_SHIFT_ARRAY=(1)
#SEQUENCE_ARRAY=(0 1 2 3 4 5 6 7 8 9 10)
SEQUENCE_ARRAY=(0 10)
RESOLUTION_ARRAY=(1.0)
STEP_SIZE_ARRAY=(0.1)
TRANSFORMATION_EPSILON_ARRAY=(0.01)
#MAXIMUM_ITERATIONS_ARRAY=(30)
LEAF_SIZE_ARRAY=(0.4)
#MINIMUM_SCAN_RANGE_ARRAY=(5)
#MAXIMUM_SCAN_RANGE_ARRAY=(200)
#MINIMUM_ADD_SCAN_SHIFT_ARRAY=(1)
SEQUENCE_ARRAY=(0)

RESOLUTION=$RESOLUTION_ARRAY
STEP_SIZE=$STEP_SIZE_ARRAY
TRANSFORMATION_EPSILON=$TRANSFORMATION_EPSILON_ARRAY
MAXIMUM_ITERATIONS=$MAXIMUM_ITERATIONS_ARRAY
LEAF_SIZE=$LEAF_SIZE_ARRAY
MINIMUM_SCAN_RANGE=$MINIMUM_SCAN_RANGE_ARRAY
MAXIMUM_SCAN_RANGE=$MAXIMUM_SCAN_RANGE_ARRAY
MINIMUM_ADD_SCAN_SHIFT=$MINIMUM_ADD_SCAN_SHIFT_ARRAY
SEQUENCE=$FIRST_SEQUENCE




#new terminal
xterm -e bash -c 'roscore' &
sleep 1



#rosparam set /first_sequence $FIRST_SEQUENCE
#rosparam set /last_sequence $LAST_SEQUENCE


PARAM_NAMES="resolution,leaf_size,sequence,translation_error,rotation_error"
echo $PARAM_NAMES > ../output/errors.csv


for RESOLUTION in ${RESOLUTION_ARRAY[@]}
do
  for STEP_SIZE in ${STEP_SIZE_ARRAY[@]}
  do
    for TRANSFORMATION_EPSILON in ${TRANSFORMATION_EPSILON_ARRAY[@]}
    do
      for MAXIMUM_ITERATIONS in ${MAXIMUM_ITERATIONS_ARRAY[@]}
      do
        for LEAF_SIZE in ${LEAF_SIZE_ARRAY[@]}
        do
          for MINIMUM_SCAN_RANGE in ${MINIMUM_SCAN_RANGE_ARRAY[@]}
          do
            for MAXIMUM_SCAN_RANGE in ${MAXIMUM_SCAN_RANGE_ARRAY[@]}
            do
              for MINIMUM_ADD_SCAN_SHIFT in ${MINIMUM_ADD_SCAN_SHIFT_ARRAY[@]}
              do
#                echo "${RESOLUTION},${STEP_SIZE},${TRANSFORMATION_EPSILON},${MAXIMUM_ITERATIONS},${LEAF_SIZE},${MINIMUM_SCAN_RANGE},${MAXIMUM_SCAN_RANGE},${MINIMUM_ADD_SCAN_SHIFT}"
                # sequence loop
#                for ((SEQUENCE=$FIRST_SEQUENCE; SEQUENCE<=$LAST_SEQUENCE;SEQUENCE++))
                for SEQUENCE in ${SEQUENCE_ARRAY[@]}
                do
                  PARAM="$RESOLUTION,$LEAF_SIZE,$SEQUENCE"
                  echo "Parameters: ""${PARAM_NAMES}"
                  echo $PARAM
                  echo -n $PARAM >> ../output/errors.csv

                rosparam set /ndt_mapping_tku/resolution $RESOLUTION
                rosparam set /ndt_mapping_tku/step_size $STEP_SIZE
                rosparam set /ndt_mapping_tku/transformation_epsilon $TRANSFORMATION_EPSILON
                rosparam set /ndt_mapping_tku/maximum_iterations $MAXIMUM_ITERATIONS
                rosparam set /ndt_mapping_tku/leaf_size $LEAF_SIZE
                rosparam set /ndt_mapping_tku/minimum_scan_range $MINIMUM_SCAN_RANGE
                rosparam set /ndt_mapping_tku/maximum_scan_range $MAXIMUM_SCAN_RANGE
                rosparam set /ndt_mapping_tku/minimum_add_scan_shift $MINIMUM_ADD_SCAN_SHIFT


                  ## play NDT_Mapping nodes
                  xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws; source devel/setup.bash; roslaunch lidar_localizer ndt_mapping_tku.launch" &
                  sleep 1


                  xterm -e bash -c 'cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun mypkg param_optimization' &
                  sleep 2

                  # play kitti_player
                  if [ $SEQUENCE -lt 10 ] ;then
                    xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d ~/dataset/rosbag/Kitti/odometry/dataset/sequence/0${SEQUENCE} --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 10 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
sleep 1
                  else
                    xterm -e bash -c "cd ~/slam_kitti_ws/mypkg/catkin_ws/; source devel/setup.bash; rosrun kitti_player kitti_player -d ~/dataset/rosbag/Kitti/odometry/dataset/sequence/${SEQUENCE} --velodyne 1 --grayscale 1 --color 1 --groundtruth 1 --timestamps 1 --frequency 10 /sensor/camera/grayscale/left/image_rect:=/sensor/camera/grayscale/left/image_raw /sensor/camera/grayscale/right/image_rect:=/sensor/camera/grayscale/right/image_raw" &
sleep 1
                  fi

                  # wait finishing error calculation
                  while [ true ]
                  do
                    tmp=$(rosparam get /done_error_calculation)
                    echo "Waiting for finishing error calculation"
                    if [ $tmp -eq 1 ] ;then
                      echo "Finish error calculation"
                      rosparam set /done_error_calculation 0
                      break
                    fi
                    sleep 0.1
                  done
                  sleep 1
                  # play next sequence
                done
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
