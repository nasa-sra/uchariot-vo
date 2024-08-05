import pyrealsense2 as rs
import numpy as np

def get_camera_intrinsics(sensor):
    # Get intrinsics of the sensor
    intrinsics = sensor.get_stream_profiles()[0].as_video_stream_profile().get_intrinsics()
    return intrinsics

def get_extrinsics(depth_sensor, color_sensor):
    # Get extrinsics from depth to color sensor
    extrinsics = depth_sensor.get_extrinsics_to(color_sensor)
    return extrinsics

def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Configure the pipeline
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    config.enable_stream(rs.stream.color)
    
    # Start streaming
    pipeline.start(config)
    
    try:
        # Get device and sensors
        device = pipeline.get_active_profile().get_device()
        depth_sensor = device.first_depth_sensor()
        color_sensor = device.first_color_sensor()

        # Get and print IMU intrinsics
        intrinsics = get_camera_intrinsics(depth_sensor)
        print("Depth Sensor Intrinsics:")
        print(f"Width: {intrinsics.width}")
        print(f"Height: {intrinsics.height}")
        print(f"PPX: {intrinsics.ppx}")
        print(f"PPY: {intrinsics.ppy}")
        print(f"FX: {intrinsics.fx}")
        print(f"FY: {intrinsics.fy}")
        print(f"Distortion Model: {intrinsics.model}")
        print(f"Distortion Coeffs: {intrinsics.coeffs}")

        # Get and print extrinsics
        extrinsics = get_extrinsics(depth_sensor, color_sensor)
        print("Extrinsics from Depth to Color:")
        print(f"Rotation: {extrinsics.rotation}")
        print(f"Translation: {extrinsics.translation}")

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()
