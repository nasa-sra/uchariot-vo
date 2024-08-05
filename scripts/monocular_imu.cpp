#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>  // For usleep

// Function to clear the line in terminal
void clearLine() {
    std::cout << "\033[2K";  // ANSI escape code to clear the current line
    std::cout << "\r";      // Move the cursor back to the beginning of the line
}

int main() {
    // Initialize the RealSense context and pipeline
    rs2::context ctx;
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    
    // Enable the IMU stream
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    
    // Start the pipeline
    pipe.start(cfg);
    
    while (true) {
        // Wait for a new set of frames
        rs2::frameset frames = pipe.wait_for_frames();
        
        // Get IMU data
        rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
        rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        
        // Check if frames are valid
        if (gyro_frame && accel_frame) {
            auto gyro = gyro_frame.as<rs2::motion_frame>();
            auto accel = accel_frame.as<rs2::motion_frame>();
            
            // Get the data
            auto gyro_data = gyro.get_motion_data();
            auto accel_data = accel.get_motion_data();
            
            // Clear the line and print new data
            clearLine();
            std::cout << "Gyro - X: " << gyro_data.x << " Y: " << gyro_data.y << " Z: " << gyro_data.z
                      << " | Accel - X: " << accel_data.x << " Y: " << accel_data.y << " Z: " << accel_data.z
                      << std::flush;
        }
        
        // Sleep for a short duration to control the update rate
        usleep(50000);  // Sleep for 50 milliseconds
    }
    
    return 0;
}
