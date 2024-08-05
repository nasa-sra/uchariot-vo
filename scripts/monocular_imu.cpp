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

    // Initialize ORB-SLAM3 system
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true);

    // Initialize RealSense pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);  // Typical IMU rate
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);   // Typical IMU rate
    pipe.start(cfg);

    cv::Mat frame;
    cv::cuda::GpuMat d_frame, d_resized_frame, d_gray_frame;
    double timestamp = 0;
    double fps = 30;
    double frame_time = 1.0 / fps;

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    std::vector<ORB_SLAM3::IMU::Point> imu_data;

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();

        // Get color and IMU frames
        auto color_frame = frames.get_color_frame();
        auto accel_frame = frames.first(RS2_STREAM_ACCEL);
        auto gyro_frame = frames.first(RS2_STREAM_GYRO);

        // Get IMU data
        rs2_vector accel_data = accel_frame.as<rs2::motion_frame>().get_motion_data();
        rs2_vector gyro_data = gyro_frame.as<rs2::motion_frame>().get_motion_data();

        // Convert RealSense frame to OpenCV Mat
        frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Upload frame to GPU and convert to grayscale
        d_frame.upload(frame);
        cv::cuda::cvtColor(d_frame, d_gray_frame, cv::COLOR_BGR2GRAY);

        // Download grayscale frame from GPU
        cv::Mat gray_frame;
        d_gray_frame.download(gray_frame);

        // Prepare IMU data for ORB-SLAM3
        imu_data.push_back(ORB_SLAM3::IMU::Point(gyro_data.x, gyro_data.y, gyro_data.z,
                                                 accel_data.x, accel_data.y, accel_data.z,
                                                 timestamp));

        // Pass the frame and IMU data to ORB-SLAM3 and get the current pose
        Sophus::SE3f pose = SLAM.TrackMonocular(gray_frame, timestamp, imu_data);

        // Clear IMU data for the next iteration
        imu_data.clear();

        // Extract position from the pose
        Eigen::Vector3f translation = pose.translation();
        float x = translation.x();
        float y = translation.y();
        float z = translation.z();

        // Create a string with the current position
        std::ostringstream position_stream;
        position_stream << std::fixed << std::setprecision(2)
                        << "Position (m): X=" << x << " Y=" << y << " Z=" << z;
        std::string position_text = position_stream.str();

        // Print the position text in an updating line at the bottom of the terminal
        std::cout << "\r" << position_text << std::flush;

        // Optionally display the frame
        #ifdef DISPLAY_FRAMES
        cv::putText(frame, position_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // Exit on 'ESC' key
            break;
        }
        #endif

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

    // Shutdown ORB-SLAM3 and stop the RealSense pipeline
    SLAM.Shutdown();
    pipe.stop();

    return 0;
}