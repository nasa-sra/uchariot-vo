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
#include <mutex>
#include <condition_variable>

void interpolateData(const std::vector<double> &vBase_times,
                     std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                     const rs2_vector &prev_data, const double &prev_time);

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

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

    // IMU data
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;
    std::vector<double> v_accel_timestamp;
    std::vector<rs2_vector> v_accel_data;
    std::vector<double> v_gyro_timestamp;
    std::vector<rs2_vector> v_gyro_data;
    std::vector<double> v_accel_timestamp_sync;
    std::vector<rs2_vector> v_accel_data_sync;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;

    cv::Mat frame;
    cv::cuda::GpuMat d_frame, d_resized_frame, d_gray_frame;
    double timestamp = 0;
    bool image_ready = false;

    // IMU callback
    auto imu_callback = [&](const rs2::frame& frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            rs2::video_frame color_frame = fs.get_color_frame();
            frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);

            timestamp = fs.get_timestamp() * 1e-3;
            image_ready = true;

            lock.unlock();
            cond_image_rec.notify_all();
        }
        else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>())
        {
            if (m_frame.get_profile().stream_name() == "Gyro")
            {
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back(m_frame.get_timestamp() * 1e-3);
            }
            else if (m_frame.get_profile().stream_name() == "Accel")
            {
                prev_accel_timestamp = current_accel_timestamp;
                prev_accel_data = current_accel_data;

                current_accel_data = m_frame.get_motion_data();
                current_accel_timestamp = m_frame.get_timestamp() * 1e-3;

                while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
                {
                    int index = v_accel_timestamp_sync.size();
                    double target_time = v_gyro_timestamp[index];

                    rs2_vector interp_data = interpolateMeasure(target_time, current_accel_data, current_accel_timestamp,
                                                                prev_accel_data, prev_accel_timestamp);

                    v_accel_data_sync.push_back(interp_data);
                    v_accel_timestamp_sync.push_back(target_time);
                }
            }
        }
    };

    // Start the pipeline with the callback
    pipe.start(cfg, imu_callback);

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    double current_fps = 0.0;

    while (true) {
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        cv::Mat gray_frame;

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!image_ready)
                cond_image_rec.wait(lk);

            // Copy the IMU data
            std::vector<rs2_vector> vGyro = v_gyro_data;
            std::vector<double> vGyro_times = v_gyro_timestamp;
            std::vector<rs2_vector> vAccel = v_accel_data_sync;
            std::vector<double> vAccel_times = v_accel_timestamp_sync;

            // Clear IMU vectors
            v_gyro_data.clear();
            v_gyro_timestamp.clear();
            v_accel_data_sync.clear();
            v_accel_timestamp_sync.clear();

            image_ready = false;

            // Prepare IMU measurements
            for(size_t i=0; i<vGyro.size(); ++i)
            {
                vImuMeas.emplace_back(vAccel[i].x, vAccel[i].y, vAccel[i].z,
                                      vGyro[i].x, vGyro[i].y, vGyro[i].z,
                                      vGyro_times[i]);
            }

            // Upload frame to GPU
            d_frame.upload(frame);

            // Convert to grayscale on GPU
            cv::cuda::cvtColor(d_frame, d_gray_frame, cv::COLOR_BGR2GRAY);

            // Download grayscale frame from GPU
            d_gray_frame.download(gray_frame);
        }

        // Pass the frame to ORB-SLAM3 and get the current pose
        Sophus::SE3f pose = SLAM.TrackMonocular(gray_frame, timestamp, vImuMeas);

        // Extract position from the pose
        Eigen::Vector3f translation = pose.translation();
        float x = translation.x();
        float y = translation.y();
        float z = translation.z();

        // Create a string with the current position and FPS
        std::ostringstream info_stream;
        info_stream << std::fixed << std::setprecision(2)
                    << "Position (m): X=" << x << " Y=" << y << " Z=" << z
                    << " | FPS: " << current_fps;
        std::string info_text = info_stream.str();

        // Print the information text in an updating line at the bottom of the terminal
        std::cout << "\r" << info_text << std::flush;

        // Optionally display the frame
        #ifdef DISPLAY_FRAMES
        cv::putText(frame, info_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // Exit on 'ESC' key
            break;
        }
        #endif

        frame_count++;

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (duration >= 1000) {
            current_fps = frame_count / (duration / 1000.0);
            start = now;
            frame_count = 0;
        }
    }

    // Shutdown ORB-SLAM3 and stop the RealSense pipeline
    SLAM.Shutdown();
    pipe.stop();

    return 0;
}

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time)
{
    // If there is no previous information, the current data is propagated
    if(prev_time == 0)
    {
        return current_data;
    }

    rs2_vector increment;
    rs2_vector value_interp;

    if(target_time > current_time) {
        value_interp = current_data;
    }
    else if(target_time > prev_time)
    {
        increment.x = current_data.x - prev_data.x;
        increment.y = current_data.y - prev_data.y;
        increment.z = current_data.z - prev_data.z;

        double factor = (target_time - prev_time) / (current_time - prev_time);

        value_interp.x = prev_data.x + increment.x * factor;
        value_interp.y = prev_data.y + increment.y * factor;
        value_interp.z = prev_data.z + increment.z * factor;
    }
    else {
        value_interp = prev_data;
    }

    return value_interp;
}