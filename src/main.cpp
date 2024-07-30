#include <iostream>
#include <opencv2/opencv.hpp>
#include <ORB_SLAM3/include/System.h>

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: ./my_slam_project path_to_vocabulary path_to_settings path_to_video" << std::endl;
        return -1;
    }

    // Load the video file
    cv::VideoCapture cap(argv[3]);
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open video file!" << std::endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cv::Mat frame;
    while (cap.read(frame))
    {
        double timestamp = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0; // Get timestamp in seconds
        SLAM.TrackMonocular(frame, timestamp);
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
