# uchariot-vo

This code is designed for use on NASA's uchariot rover based in a C++ implementation. It has been tested on a Jetson Nano running Ubuntu 20.04 but should be compatible with all future Ubuntu verisions assuming packages are still supported. Nvidia CUDA is attempted for the install and it packages are running with the assumption of a CUDA GPU avaliable.

## Required Dependencies
* OpenCV (4.4 or higher) 
* Pangolin
* Eigen3
* Nvidia CUDA Toolkit: https://developer.nvidia.com/cuda-toolkit
* Intel Realsense SDK (WIP): https://github.com/IntelRealSense/librealsense/releases
* ORB_SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
* DBoW2 and g2o (Included in ORB_SLAM3 Thirdparty folder)

## Installation
Everything else should be installed prior to ORB_SLAM3.

Dependency install:
```
sudo apt update
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y python-dev python3-dev python-numpy python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libeigen3-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev
```