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
#include <Eigen/Dense>

// Function to clear the line in terminal
void clearLine() {
    std::cout << "\033[2K";  // ANSI escape code to clear the current line
    std::cout << "\r";      // Move the cursor back to the beginning of the line
}

// Gravity vector (m/s^2) assuming z-axis is up for IMU
const Eigen::Vector3f gravity(0.0, 0.0, 9.81);

// Function to apply rotation to the gravity vector
Eigen::Vector3f rotateGravityVector(const Eigen::Vector3f& gravity, const Eigen::Matrix3f& rotation) {
    return rotation * gravity;
}

// Function to update the rotation matrix based on gyro data (placeholder)
Eigen::Matrix3f updateRotationMatrix(const Eigen::Vector3f& gyro, float delta_t, const Eigen::Matrix3f& current_rotation) {
    // Placeholder implementation, use integration of gyro data to update the rotation matrix
    // For simplicity, this example assumes no change
    return current_rotation;
}

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
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, true);

    // Initialize RealSense pipeline
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    pipe.start(cfg);

    cv::Mat frame;
    cv::cuda::GpuMat d_frame, d_gray_frame;
    double timestamp = 0;
    double fps = 30; // Assuming 30 FPS, adjust if needed
    double frame_time = 1.0 / fps;

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    // Define rotation matrix if needed to adjust for IMU orientation
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();  // Initial rotation matrix

    while (true) {
        // Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // Get the color frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
        rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);

        // Convert RealSense frame to OpenCV Mat
        frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Upload frame to GPU
        d_frame.upload(frame);

        // Convert to grayscale on GPU
        cv::cuda::cvtColor(d_frame, d_gray_frame, cv::COLOR_BGR2GRAY);

        // Download grayscale frame from GPU
        cv::Mat gray_frame;
        d_gray_frame.download(gray_frame);

        // Pass the frame to ORB-SLAM3 and get the current pose
        Sophus::SE3f pose = SLAM.TrackMonocular(gray_frame, timestamp);

        // Extract position from the pose
        Eigen::Vector3f translation = pose.translation();
        float x = translation.x();
        float y = translation.y();
        float z = translation.z();

        // Get IMU data if available
        std::string imu_data;
        if (gyro_frame && accel_frame) {
            auto accel = accel_frame.as<rs2::motion_frame>();
            auto accel_data = accel.get_motion_data();
            auto gyro = gyro_frame.as<rs2::motion_frame>();
            auto gyro_data = gyro.get_motion_data();

            // Convert accelerometer and gyro data to Eigen vectors
            Eigen::Vector3f accel_vec(accel_data.x, accel_data.y, accel_data.z);
            Eigen::Vector3f gyro_vec(gyro_data.x, gyro_data.y, gyro_data.z);

            // Update rotation matrix based on gyro data
            rotation_matrix = updateRotationMatrix(gyro_vec, frame_time, rotation_matrix);

            // Adjust gravity vector
            Eigen::Vector3f adjusted_gravity = rotateGravityVector(gravity, rotation_matrix);

            // Compensate for gravity in the accelerometer data
            Eigen::Vector3f corrected_accel = accel_vec - adjusted_gravity;

            std::ostringstream imu_stream;
            imu_stream << "Accel - X: " << corrected_accel.x() << " Y: " << corrected_accel.y() << " Z: " << corrected_accel.z();
            imu_data = imu_stream.str();
        }

        // Create a string with the current position
        std::ostringstream position_stream;
        position_stream << std::fixed << std::setprecision(2)
                        << "Position (m): X=" << x << " Y=" << y << " Z=" << z;
        std::string position_text = position_stream.str();

        // Print the position and IMU data in an updating line at the bottom of the terminal
        clearLine();
        std::cout << position_text << " | " << imu_data << std::flush;

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
