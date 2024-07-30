# uchariot-vo

This code is designed for use on NASA's uchariot rover based in a C++ implementation. It has been tested on a Jetson Nano running Ubuntu 20.04 but should be compatible with all future Ubuntu verisions assuming packages are still supported. Nvidia CUDA is attempted for the install and it packages are running with the assumption of a CUDA GPU avaliable.

## Required Dependencies
* Pangolin
* OpenCV (4.4 or higher)
* Eigen3
* Nvidia CUDA Toolkit: https://developer.nvidia.com/cuda-toolkit
* Intel Realsense SDK (WIP): https://github.com/IntelRealSense/librealsense/releases
* ORB_SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
* DBoW2 and g2o (Included in ORB_SLAM3 Thirdparty folder)

## Installation
Everything else should be installed prior to ORB_SLAM3.

### Dependency install:
```
sudo apt update
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install -y python-dev python3-dev python-numpy python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libeigen3-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev
```
### Install Pangolin:
```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
### Install OpenCV with CUDA support:
```
cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF ..
make -j$(nproc)
sudo make install
```
### Install Eigen3:
```

```

### Install Nvidia CUDA Toolkit:
```

```

### Install Intel Realsense SDK (WIP):
```

```

### Install ORB_SLAM3:
Clone the repository
```
cd ~
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
