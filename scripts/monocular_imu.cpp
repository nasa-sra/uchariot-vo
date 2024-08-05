#include <iostream>
#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp> // For JSON parsing
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::milliseconds
#include <array>

using json = nlohmann::json;

// Callback function to handle the data fetched by libcurl
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// Integration parameters
const double dt = 0.1; // Time step in seconds, adjust based on your update frequency

class IMUData {
public:
    std::array<double, 3> pos = {0, 0, 0};
    std::array<double, 3> vel = {0, 0, 0};
    std::array<double, 3> accel = {0, 0, 0};
    std::array<double, 3> prev_accel = {0, 0, 0};

    void update(const std::array<double, 3>& new_accel) {
        for (int i = 0; i < 3; ++i) {
            // Verlet integration for position
            double new_pos = pos[i] + vel[i] * dt + 0.5 * accel[i] * dt * dt;
            
            // Update velocity using average acceleration
            vel[i] += 0.5 * (accel[i] + new_accel[i]) * dt;
            
            // Update position and acceleration
            pos[i] = new_pos;
            prev_accel[i] = accel[i];
            accel[i] = new_accel[i];
        }
    }
};

void fetchSensorData(IMUData& imu_data) {
    CURL* curl;
    CURLcode res;
    std::string readBuffer;
    std::string url = "http://192.168.2.31:8000/imu_data";

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        } else {
            // Parse JSON data
            try {
                auto jsonData = json::parse(readBuffer);

                // Extract data
                std::array<double, 3> new_accel = {
                    jsonData["accel_x"].get<double>(),
                    jsonData["accel_y"].get<double>(),
                    jsonData["accel_z"].get<double>()
                };

                // Update IMU data using Verlet integration
                imu_data.update(new_accel);

            } catch (const json::exception& e) {
                std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            }
        }
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
}

int main() {
    IMUData imu_data;

    while (true) {
        fetchSensorData(imu_data);
        std::cout << "\rPosition (x, y, z): (" 
                  << imu_data.pos[0] << ", " << imu_data.pos[1] << ", " << imu_data.pos[2] << ") | "
                  << "Velocity (x, y, z): (" 
                  << imu_data.vel[0] << ", " << imu_data.vel[1] << ", " << imu_data.vel[2] << ") | "
                  << "Acceleration (x, y, z): (" 
                  << imu_data.accel[0] << ", " << imu_data.accel[1] << ", " << imu_data.accel[2] << ")"
                  << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    return 0;
}