# uchariot-vo

This code is designed for use on NASA's uchariot rover based in a C++ implementation. It has been tested on a Jetson Nano running Ubuntu 20.04 but should be compatible with all future Ubuntu verisions assuming packages are still supported. Nvidia CUDA is attempted for the install and it packages are running with the assumption of a CUDA GPU avaliable.

## Required Dependencies
* Pangolin
* OpenCV (4.4 or higher)
* Eigen3 (Installed with dependencies)
* Nvidia CUDA Toolkit: https://developer.nvidia.com/cuda-toolkit
* Intel Realsense SDK: https://www.intelrealsense.com/sdk-2/
* ORB_SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
* DBoW2 and g2o (Included in ORB_SLAM3 Thirdparty folder and built with ORB_SLAM3)

## Installation
Everything else should be installed prior to ORB_SLAM3.

### Clone the Repo:
```
git clone https://github.com/nasa-sra/uchariot-vo.git
```

### Dependency install:
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
### Install OpenCV with CUDA support:
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
      -D CUDA_ARCH_BIN="87" \
      -D CUDA_ARCH_PTX="87" \
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
