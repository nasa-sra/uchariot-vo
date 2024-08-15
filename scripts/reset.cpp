#include <librealsense2/rs.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    rs2::context ctx;
    
    std::cout << "Resetting all RealSense devices..." << std::endl;
    
    for (auto&& dev : ctx.query_devices()) {
        std::cout << "Resetting device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        dev.hardware_reset();
        
        // Wait for a moment to ensure the reset is complete
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    std::cout << "All devices have been reset." << std::endl;
    
    return 0;
}