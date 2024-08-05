#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>  // For usleep

// Function to clear the line in terminal
void clearLine() {
    std::cout << "\033[2K";  // ANSI escape code to clear the current line
    std::cout << "\r";      // Move the cursor back to the beginning of the line
}

// Function to integrate data to estimate velocity and position
void integrate(double& position, double& velocity, double accel, double dt) {
    velocity += accel * dt;
    position += velocity * dt;
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
    
    double position_x = 0.0, velocity_x = 0.0;
    double position_y = 0.0, velocity_y = 0.0;
    double position_z = 0.0, velocity_z = 0.0;
    
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (true) {
        // Wait for a new set of frames
        rs2::frameset frames = pipe.wait_for_frames();
        
        // Get IMU data
        rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        
        if (accel_frame) {
            auto accel = accel_frame.as<rs2::motion_frame>();
            auto accel_data = accel.get_motion_data();
            
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - last_time;
            last_time = now;
            double dt = elapsed.count();
            
            // Integrate data
            integrate(position_x, velocity_x, accel_data.x, dt);
            integrate(position_y, velocity_y, accel_data.y, dt);
            integrate(position_z, velocity_z, accel_data.z, dt);
            
            // Clear the line and print new data
            clearLine();
            std::cout << "Accel - X: " << accel_data.x << " Y: " << accel_data.y << " Z: " << accel_data.z
                      << " | Vel - X: " << velocity_x << " Y: " << velocity_y << " Z: " << velocity_z
                      << " | Pos - X: " << position_x << " Y: " << position_y << " Z: " << position_z
                      << std::flush;
        }
        
        // Sleep for a short duration to control the update rate
        usleep(50000);  // Sleep for 50 milliseconds
    }
    
    return 0;
}
