import pyrealsense2 as rs
import numpy as np

def get_intrinsics(stream_profile):
    # Get intrinsics of the stream profile
    intrinsics = stream_profile.get_intrinsics()
    return intrinsics

def get_extrinsics(depth_profile, color_profile):
    # Get extrinsics from depth to color sensor
    extrinsics = depth_profile.get_extrinsics_to(color_profile)
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
        # Get the active profile and its streams
        profile = pipeline.get_active_profile()
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()

        # Get and print depth sensor intrinsics
        depth_intrinsics = get_intrinsics(depth_stream)
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
        color_intrinsics = get_intrinsics(color_stream)
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
        extrinsics = get_extrinsics(depth_stream, color_stream)
        print("Extrinsics from Depth to Color:")
        print(f"Rotation: {extrinsics.rotation}")
        print(f"Translation: {extrinsics.translation}")

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()
