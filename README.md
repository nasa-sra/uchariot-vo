# uchariot-vo

This code is designed for use on NASA's uchariot rover based in a C++ implementation. It has been tested on a Jetson Nano running Ubuntu 20.04 but should be compatible with all future Ubuntu verisions assuming packages are still supported. Nvidia CUDA is attempted for the install and it packages are running with the assumption of a CUDA GPU avaliable.

## Required Dependencies
* ORB_SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
* OpenCV (4.4 or higher)
* Pangolin
* Eigen3
* DBoW2 and g2o (Included in ORB_SLAM3 Thirdparty folder)
* Nvidia CUDA Toolkit
* Intel Realsense SDK (WIP)

## Installation
