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

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ./YourORB_SLAM3Executable <path_to_vocabulary_file> <path_to_settings_file>" << std::endl;
        return -1;
    }
    if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
        std::cerr << "No CUDA capable devices found!" << std::endl;
        return -1;
    }
    std::string voc_file = argv[1];
    std::string settings_file = argv[2];
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, true);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 60);
    pipe.start(cfg);

    cv::Mat frame, gray_frame;
    cv::cuda::GpuMat d_frame, d_gray_frame;
    double timestamp = 0;
    double fps = 60;
    double frame_time = 1.0 / fps;

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        frame = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        d_frame.upload(frame);

        // Convert to grayscale on GPU
        cv::cuda::cvtColor(d_frame, d_gray_frame, cv::COLOR_BGR2GRAY);

        // Download grayscale frame from GPU to CPU
        d_gray_frame.download(gray_frame);

        // Process the grayscale frame
        Sophus::SE3f pose = SLAM.TrackMonocular(gray_frame, timestamp);

        Eigen::Vector3f translation = pose.translation();
        float x = translation.x();
        float y = translation.y();
        float z = translation.z();
        std::ostringstream position_stream;
        position_stream << std::fixed << std::setprecision(2)
                        << "Position (m): X=" << x << " Y=" << y << " Z=" << z;
        std::string position_text = position_stream.str();
        std::cout << "\r" << position_text << std::flush;

        timestamp += frame_time;
        frame_count++;
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (duration >= 1000) {
            double current_fps = frame_count / (duration / 1000.0);
            std::cout << "\rFPS: " << current_fps << std::flush;
            start = now;
            frame_count = 0;
        }
    }

    SLAM.Shutdown();
    pipe.stop();

    return 0;
}
