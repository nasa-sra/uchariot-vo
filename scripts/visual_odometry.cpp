// Importing Libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "System.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sophus/se3.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <condition_variable>
#include <iostream>
#include <unistd.h>
#include <limits.h>

// Main function
int main(int argc, char** argv) {
    // Gets current working directory
    char currentDirectory[PATH_MAX];
    if (getcwd(currentDirectory, sizeof(currentDirectory)) == nullptr) {
        std::cerr << "Error getting current directory: " << errno << std::endl;
        return 1;
    }

    // Check for CUDA device
    if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
        std::cerr << "No CUDA capable devices found!" << std::endl;
        return -1;
    }

    // Convert currentDirectory to a std::string
    std::string currentDirStr(currentDirectory);

    // 
    std::string voc_file = currentDirStr + "/Vocabulary/ORBvoc.txt"; 
    std::string settings_file = currentDirStr + "/../config/RealSense_D455.yaml";

    