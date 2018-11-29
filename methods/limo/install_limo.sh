#!/bin/bash

cd `dirname $0`

echo "/-----　　 Install limo　　　-----/"

# packages for using limo
sudo apt-get install libpng++-dev
sudo apt-get install python-catkin-tools
sudo apt-get install ros-kinetic-opencv-apps
sudo apt-get install git

echo "/* clone limo */"
mkdir -p devel build logs
catkin init
cd src
#git submodule add https://github.com/Dokirobot-autonomous/limo.git

echo "/* install Ceres (http://ceres-solver.org/installation.html) */"
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev
# download ceres-solver
#wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
#tar zxf ceres-solver-1.14.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.14.0
make -j3
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
sudo make install

cd ../

echo "/* Build limo */"
cd limo
bash install_repos.sh
catkin run_tests limo --profile limo_release



