import pyrealsense2 as rs
import numpy as np

def get_intrinsics(sensor):
    # Get intrinsics of the sensor
    profile = sensor.get_stream_profiles()[0].as_video_stream_profile()
    intrinsics = profile.get_intrinsics()
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

        # Get and print depth sensor intrinsics
        depth_intrinsics = get_intrinsics(depth_sensor)
        print("Depth Sensor Intrinsics:")
        print(f"Width: {depth_intrinsics.width}")
        print(f"Height: {depth_intrinsics.height}")
        print(f"PPX: {depth_intrinsics.ppx}")
        print(f"PPY: {depth_intrinsics.ppy}")
        print(f"FX: {depth_intrinsics.fx}")
        print(f"FY: {depth_intrinsics.fy}")
        print(f"Distortion Model: {depth_intrinsics.model}")
        print(f"Distortion Coeffs: {depth_intrinsics.coeffs}")

        # Get and print color sensor intrinsics
        color_intrinsics = get_intrinsics(color_sensor)
        print("Color Sensor Intrinsics:")
        print(f"Width: {color_intrinsics.width}")
        print(f"Height: {color_intrinsics.height}")
        print(f"PPX: {color_intrinsics.ppx}")
        print(f"PPY: {color_intrinsics.ppy}")
        print(f"FX: {color_intrinsics.fx}")
        print(f"FY: {color_intrinsics.fy}")
        print(f"Distortion Model: {color_intrinsics.model}")
        print(f"Distortion Coeffs: {color_intrinsics.coeffs}")

        # Get and print extrinsics
        extrinsics = depth_sensor.get_extrinsics_to(color_sensor)
        print("Extrinsics from Depth to Color:")
        print(f"Rotation: {extrinsics.rotation}")
        print(f"Translation: {extrinsics.translation}")

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()
