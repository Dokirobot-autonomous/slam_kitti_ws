#!/bin/bash

cd `dirname $0`

echo "/-----　　 Install sptam　　　-----/"

# packages for using sptam
sudo apt-get install ros-kinetic-desktop
sudo apt-get install libsuitesparse-dev

mkdir lib packages

echo "\\/* Install ros-utils */\\"
mkdir lib packages
cd lib
#git submodule add https://github.com/lrse/ros-utils.git
git clone https://github.com/lrse/ros-utils.git

echo "\\/* Install g2o */\\"
#git submodule add https://github.com/RainerKuemmerle/g2o.git
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 4b9c2f5b68d14ad479457b18c5a2a0bce1541a90
mkdir build && cd build
cmake ..
make 
sudo make install

cd ../..

echo "\\/* Install DLib */\\"
#git submodule add https://github.com/dorian3d/DLib.git
git clone https://github.com/dorian3d/DLib.git
cd DLib

git checkout -b ohashi_devel 70089a38056e8aebd5a2ebacbcb67d3751433f32
mkdir build && cd build
cmake ..
cmake --build . --config Release
make
sudo make install
cd ../..

echo "\\/* Install DBow2 */\\"
#git submodule add https://github.com/dorian3d/DBoW2.git
git clone https://github.com/dorian3d/DBoW2.git
cd DBoW2
git checkout -b ohashi_devel 82401cad2cfe7aa28ee6f6afb01ce3ffa0f59b44
mkdir build && cd build
cmake ..
make
sudo make install
cd ../..

echo "\\/* Install DLoopDetector */\\"
#git submodule add https://github.com/dorian3d/DLoopDetector.git
git clone https://github.com/dorian3d/DLoopDetector.git
cd DLoopDetector
git checkout -b ohashi_devel 8e62f8ae84d583d9ab67796f779272b0850571ce
mkdir build && cd build
cmake ..
make
sudo make install
cd ../..

echo "\\/* Install OpenGV */\\"
#git submodule add https://github.com/laurentkneip/opengv.git
git clone https://github.com/laurentkneip/opengv.git
cd opengv
git checkout -b ohashi_devel 2e2d21917fd2fb75f2134e6d5be7a2536cbc7eb1
mkdir build && cd build
cmake ..
make
sudo make install
cd ../..

cd ../packages

echo -e "\/* Install S-PTAM */\\"
git clone https://github.com/Dokirobot-autonomous/sptam.git
echo "Please comment-out following sentences in sptam/CMakeLists.txt"
echo "--------------------"
echo "enable_testing()"
echo "add_subdirectory( src/tests )"
echo "--------------------"
echo "Please [ENTER] when finishing"
read tmp
cd ../..
catkin_make #build ros-util
catkin_make --pkg sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_LOOPCLOSURE=ON -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON


