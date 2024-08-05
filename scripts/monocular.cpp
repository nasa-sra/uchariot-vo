#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "System.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sophus/se3.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Base64 decoding function
std::string base64_decode(const std::string& in) {
    std::string out;
    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++) T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] = i;

    int val = 0, valb = -8;
    for (unsigned char c : in) {
        if (T[c] == -1) break;
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0) {
            out.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return out;
}

size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

cv::Mat fetch_frame(const std::string& url) {
    CURL* curl = curl_easy_init();
    std::string response;
    
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        CURLcode res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if(res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            return cv::Mat();
        }
    }

    json j = json::parse(response);
    std::string base64_frame = j["frame"];
    std::string decoded_data = base64_decode(base64_frame);
    std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
    cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);
    return frame;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: ./YourORB_SLAM3Executable <path_to_vocabulary_file> <path_to_settings_file> <stream_url>" << std::endl;
        return -1;
    }

    // Check for CUDA device
    if (cv::cuda::getCudaEnabledDeviceCount() == 0) {
        std::cerr << "No CUDA capable devices found!" << std::endl;
        return -1;
    }

    std::string voc_file = argv[1];
    std::string settings_file = argv[2];
    std::string stream_url = std::string(argv[3]) + "/frame";

    // Initialize ORB-SLAM3 system
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, true);

    // Define the target resolution
    int target_width = 1280;
    int target_height = 720;

    cv::Mat frame;
    cv::cuda::GpuMat d_frame, d_resized_frame, d_gray_frame;
    double timestamp = 0;
    double fps = 60.0;  // Assume 30 FPS for the stream
    double frame_time = 1.0 / fps;

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;

    while (true) {
        frame = fetch_frame(stream_url);
        if (frame.empty()) {
            std::cerr << "Failed to grab frame from stream!" << std::endl;
            break;
        }

        // Resize the frame to the target resolution
        cv::resize(frame, frame, cv::Size(target_width, target_height));

        // Upload frame to GPU
        d_frame.upload(frame);

        // Convert to grayscale on GPU
        if (frame.channels() == 3) {
            cv::cuda::cvtColor(d_frame, d_gray_frame, cv::COLOR_BGR2GRAY);
        } else {
            d_gray_frame = d_frame;
        }

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

    // Shutdown ORB-SLAM3
    SLAM.Shutdown();

    return 0;
}