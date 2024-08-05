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
#include <librealsense2/hpp/rs_context.hpp>

class Realsense {
public:
    Realsense() {
        if (!IsIMUValid()) {
            std::cerr << "Device supporting IMU not found" << std::endl;
            throw std::runtime_error("IMU device not found");
        }

        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

        profile = pipe.start(cfg, [&](rs2::frame frame) {
            auto motion = frame.as<rs2::motion_frame>();
            if (motion) {
                if (motion.get_profile().stream_type() == RS2_STREAM_GYRO) {
                    double ts = motion.get_timestamp();
                    rs2_vector gyro_data = motion.get_motion_data();
                    // Process the gyro data with your algorithm if needed
                } else if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL) {
                    rs2_vector accel_data = motion.get_motion_data();
                    // Process the accel data with your algorithm if needed
                }
            }
        });
    }

    ~Realsense() {
        Stop();
    }

    bool IsIMUValid() {
        bool found_gyro = false;
        bool found_accel = false;
        rs2::context ctx;
        for (auto dev : ctx.query_devices()) {
            for (auto sensor : dev.query_sensors()) {
                for (auto profile : sensor.get_stream_profiles()) {
                    if (profile.stream_type() == RS2_STREAM_GYRO)
                        found_gyro = true;

                    if (profile.stream_type() == RS2_STREAM_ACCEL)
                        found_accel = true;
                }
            }
            if (found_gyro && found_accel)
                break;
        }
        return found_gyro && found_accel;
    }

    void Update() {
        frameset = pipe.wait_for_frames();
    }

    void Stop() {
        pipe.stop();
    }

    rs2::frameset get_frames() const {
        return frameset;
    }

    std::pair<rs2_vector, rs2_vector> get_imu_data() const {
        rs2_vector accel_data{0, 0, 0};
        rs2_vector gyro_data{0, 0, 0};

        auto accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL);
        auto gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO);

        if (accel_frame) {
            auto accel_motion = accel_frame.as<rs2::motion_frame>();
            accel_data = accel_motion.get_motion_data();
        }

        if (gyro_frame) {
            auto gyro_motion = gyro_frame.as<rs2::motion_frame>();
            gyro_data = gyro_motion.get_motion_data();
        }

        return {accel_data, gyro_data};
    }

private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile profile;
    rs2::frameset frameset;
};

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

    // Initialize RealSense
    Realsense realsense;

    // Check if the RealSense IMU is valid
    if (!realsense.IsIMUValid()) {
        std::cerr << "Device supporting IMU not found" << std::endl;
        return -1;
    }

    cv::Mat frame;
    cv::cuda::GpuMat d_frame, d_gray_frame;
    double timestamp = 0;
    double fps = 30; // Assuming 30 FPS, adjust if needed
    double frame_time = 1.0 / fps;

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    while (true) {
        // Update the RealSense to capture frames
        realsense.Update();

        // Get the color frame
        rs2::frame color_frame = realsense.get_frames().get_color_frame();

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

        // Create a string with the current position
        std::ostringstream position_stream;
        position_stream << std::fixed << std::setprecision(2)
                        << "Position (m): X=" << x << " Y=" << y << " Z=" << z;
        std::string position_text = position_stream.str();

        // Print the position text in an updating line at the bottom of the terminal
        std::cout << "\r" << position_text << std::flush;

        // Process and display IMU data if available
        auto imu_data = realsense.get_imu_data();
        std::ostringstream imu_stream;
        imu_stream << "IMU Accel (m/s²): X=" << imu_data.first.x << " Y=" << imu_data.first.y << " Z=" << imu_data.first.z
                   << " | Gyro (rad/s): X=" << imu_data.second.x << " Y=" << imu_data.second.y << " Z=" << imu_data.second.z;
        std::string imu_text = imu_stream.str();

        std::cout << "\r" << imu_text << std::flush;

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

    // Shutdown ORB-SLAM3 and stop the RealSense
    SLAM.Shutdown();
    realsense.Stop();

    return 0;
}
