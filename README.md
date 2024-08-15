# uchariot-vo

This code is designed for use on NASA's uchariot rover based in a C++ implementation. It has been tested on a Jetson Nano running Ubuntu 20.04 but should be compatible with all future Ubuntu verisions assuming packages are still supported. Nvidia CUDA is attempted for the install and it packages are running with the assumption of a CUDA GPU avaliable. This is partially complete with the goal of NASA-SRA students finishing this visual odometry implementation in 2025. CUDA setup has been proven, but may not be needed to run well.

Measurements have been taken on the Intel Realsense D455 and haven't been proven to be accurate but tracking works quite well in a 3D space with either Monocular or RGB-D-Inertial.

Both tracking programs have been heavily based on the example code provided by ORB_SLAM3 and need more work.

This also still needs to be implemented and merged with uchariot-vision so it can work seamlessly.

Calibration of sensors goes with this list:
* RGB Camera (fully tuned and tested to work well)
* Stereo Camera (Not tuned and barely tested)
* IMU (Not tuned and needs an algorithim developed to compensate for gravity)

## Required Dependencies
* Pangolin
* OpenCV (4.4 or higher)
* Eigen3 (Installed with dependencies)
* Nvidia CUDA Toolkit: https://developer.nvidia.com/cuda-toolkit
* Intel Realsense SDK: https://www.intelrealsense.com/sdk-2/
* ORB_SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
* DBoW2 and g2o (Included in ORB_SLAM3 Thirdparty folder and built with ORB_SLAM3)
* Python (<=3.9, tested with 3.8 included with Ubuntu 20.04)

## Installation
Everything else should be installed prior to ORB_SLAM3.

### Clone the Repo:
```
git clone https://github.com/nasa-sra/uchariot-vo.git
```

### Dependency install (make sure it downloads all dependencies and doesn't get caught up on a specific line):
```
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential
sudo apt install -y libeigen3-dev
sudo apt install -y cmake
sudo apt install -y libgtk2.0-dev pkg-config
sudo apt install -y libepoxy-dev
sudo apt install -y libboost-all-dev libssl-dev
sudo apt install -y build-essential dkms
sudo apt install -y cuda
sudo apt install -y libglvnd-dev libgl1-mesa-dev libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo apt install -y libgl1-mesa-dev libglew-dev libpython2.7-dev libpython3-dev python-numpy python3-numpy
sudo apt install -y libgl1-mesa-dev libglew-dev
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y python-dev python3-dev python-numpy python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libeigen3-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev
sudo apt install -y libgtk2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libdc1394-22-dev
sudo apt update
sudo apt upgrade -y
```
### Install Pangolin:
```
cd ~
cd uchariot-vo
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
### Install Nvidia CUDA Toolkit:
Follow install instructions here: https://developer.nvidia.com/cuda-downloads
### Install OpenCV with CUDA and FFMPEG support:
```
cd ~
cd uchariot-vo
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN="80" \
      -D CUDA_ARCH_PTX="80" \
      -D WITH_CUBLAS=ON \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_FFMPEG=ON \
      -D OPENCV_FFMPEG_SKIP_BUILD_CHECK=ON \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D BUILD_opencv_cudacodec=ON \
      -D WITH_GTK=ON \
      -D WITH_GTK_2_X=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      ..
make -j$(nproc)
sudo make install
sudo ldconfig
```
### Install Intel Realsense SDK:
Follow instructions here:
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

### Install ORB_SLAM3:
Clone the repository
```
cd ~
cd uchariot-vo
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
```
Edit the CMakeLists.txt file to enable CUDA support:
```
sed -i 's/set(USE_CUDA OFF)/set(USE_CUDA ON)/' CMakeLists.txt
```
Edit the CMakeLists.txt file to compile with C++14 (https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/903#issuecomment-2252526837)
```
sed -i 's/++11/++14/g' CMakeLists.txt
```
Build ORB_SLAM3
```
chmod +x build.sh
./build.sh
```

### Calibration:
In the calibration folder, there are files to get the factory settings of the realsense camera or tune it with a calibration board (https://calib.io/pages/camera-calibration-pattern-generator and choose the checkboard pattern with 15mm squares). The factory settings should be good enough for tuning the camera and can be copied from the .yaml file generated by the program or just copied from the text output and put in the config folder. IMU tuning should be done, but isn't the most pressing issue and can probably be ignored until final deployment.

### Development:
The `/scripts` folder has the current scripts for visual odometry they can be written in that current file and then can be compiled by running `./script.sh` in the parent directory (make sure you can run it by running `chmod +x script.sh` before the first time you run it).

This will copy any files in the `/scripts` directory and add them to the `CMakeLists.txt` file in the `/ORB_SLAM3` directory. All generated executables are then generated in `/ORB_SLAM3` but can be moved to run anywhere with `./file_name`.