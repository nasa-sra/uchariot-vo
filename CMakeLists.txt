cmake_minimum_required(VERSION 3.0)
project(ORB_SLAM3_Cpp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV 4.4 REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

# Find Eigen3
find_package(Eigen3 3.1.0 REQUIRED)

# Find Pangolin
find_package(Pangolin REQUIRED)

# Set the path to ORB_SLAM3 directory
set(ORB_SLAM3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3)

# Include ORB_SLAM3 headers
include_directories(
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/g2o # include the g2o directory containing config.h
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

# Link ORB_SLAM3 library
link_directories(${ORB_SLAM3_DIR}/lib)

# Add executable
add_executable(ORB_SLAM3_Cpp monocular_test.cpp)

# Link libraries
target_link_libraries(ORB_SLAM3_Cpp
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
    ORB_SLAM3
)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)