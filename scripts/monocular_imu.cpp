#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "System.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sophus/se3.hpp>
#include <librealsense2/rs.hpp> // Add RealSense header
#include <mutex>
#include <condition_variable>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ./YourORB_SLAM3Executable <path_to_vocabulary_file> <path_to_settings_file>" << std::endl;
        return -1;
    }

    // Check for CUDA device
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
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    pipe.start(cfg);

    cv::Mat frame;
    cv::cuda::GpuMat d_frame, d_resized_frame, d_gray_frame;
    double timestamp = 0;
    double fps = 30; // Assuming 30 FPS, adjust if needed
    double frame_time = 1.0 / fps;

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    std::mutex imu_mutex;
    std::condition_variable imu_cond;
    std::vector<rs2_vector> v_gyro_data;
    std::vector<double> v_gyro_timestamp;
    std::vector<rs2_vector> v_accel_data;
    std::vector<double> v_accel_timestamp;
    double offset = 0;

    auto imu_callback = [&](const rs2::frame& frame) {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
            if (m_frame.get_profile().stream_name() == "Gyro") {
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
            } else if (m_frame.get_profile().stream_name() == "Accel") {
                v_accel_data.push_back(m_frame.get_motion_data());
                v_accel_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
            }
        }
    };

    // Start RealSense pipeline with IMU callback
    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    while (true) {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the color frame
        rs2::frame color_frame = frames.get_color_frame();

        // Convert RealSense frame to OpenCV Mat
        frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Upload frame to GPU
        d_frame.upload(frame);

        // Convert to grayscale on GPU
        cv::cuda::cvtColor(d_frame, d_gray_frame, cv::COLOR_BGR2GRAY);

        // Download grayscale frame from GPU
        cv::Mat gray_frame;
        d_gray_frame.download(gray_frame);

        // Synchronize IMU data with visual data
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            std::unique_lock<std::mutex> lock(imu_mutex);
            for (size_t i = 0; i < v_gyro_timestamp.size(); ++i) {
                if (i < v_accel_timestamp.size()) {
                    ORB_SLAM3::IMU::Point imu_point(
                        v_accel_data[i].x, v_accel_data[i].y, v_accel_data[i].z,
                        v_gyro_data[i].x, v_gyro_data[i].y, v_gyro_data[i].z,
                        v_gyro_timestamp[i]
                    );
                    vImuMeas.push_back(imu_point);
                }
            }
        }

        // Pass the frame and IMU measurements to ORB-SLAM3 and get the current pose
        Sophus::SE3f pose = SLAM.TrackMonocular(gray_frame, timestamp, vImuMeas);

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
