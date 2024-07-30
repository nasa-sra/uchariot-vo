cmake_minimum_required(VERSION 3.10)
project(Visual_Odometry)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Define the path to the ORB_SLAM3 installation
set(ORB_SLAM3_DIR "${CMAKE_SOURCE_DIR}/../ORB_SLAM3")

# Add ORB_SLAM3 and OpenCV paths
set(ORB_SLAM3_INCLUDE_DIR "${ORB_SLAM3_DIR}/include")
set(ORB_SLAM3_LIB_DIR "${ORB_SLAM3_DIR}/lib")

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${ORB_SLAM3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# Link directories
link_directories(${ORB_SLAM3_LIB_DIR})

# Add executable
add_executable(Visual_Odometry src/main.cpp)

# Link libraries
target_link_libraries(Visual_Odometry
    ${ORB_SLAM3_LIB_DIR}/libORB_SLAM3.so
    ${OpenCV_LIBS}
)
