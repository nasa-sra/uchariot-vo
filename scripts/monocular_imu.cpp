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
    try {
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
        std::cout << "Initializing ORB-SLAM3..." << std::endl;
        ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true);

        // Initialize RealSense pipeline
        std::cout << "Initializing RealSense pipeline..." << std::endl;
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);

        std::cout << "Starting RealSense pipeline..." << std::endl;
        rs2::pipeline_profile profile = pipe.start(cfg);

        // Print the device information
        auto dev = profile.get_device();
        std::cout << "Using device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "    Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "    Firmware version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;

        cv::Mat frame;
        cv::cuda::GpuMat d_frame, d_resized_frame, d_gray_frame;
        double timestamp = 0;
        double fps = 30;
        double frame_time = 1.0 / fps;

        auto start = std::chrono::high_resolution_clock::now();
        int frame_count = 0;

        std::vector<ORB_SLAM3::IMU::Point> imu_data;

        std::cout << "Entering main loop..." << std::endl;
        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();

            // Get color and IMU frames
            auto color_frame = frames.get_color_frame();
            auto accel_frame = frames.first(RS2_STREAM_ACCEL);
            auto gyro_frame = frames.first(RS2_STREAM_GYRO);

            if (!color_frame || !accel_frame || !gyro_frame) {
                std::cerr << "Failed to get frames. Color: " << (color_frame ? "OK" : "Missing")
                          << ", Accel: " << (accel_frame ? "OK" : "Missing")
                          << ", Gyro: " << (gyro_frame ? "OK" : "Missing") << std::endl;
                continue;
            }

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

    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return -1;
    } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    return 0;
}