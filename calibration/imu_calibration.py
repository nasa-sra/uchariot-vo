import pyrealsense2 as rs
import numpy as np
import yaml

def get_intrinsics(stream_profile):
    # Get intrinsics of the stream profile
    intrinsics = stream_profile.get_intrinsics()
    return intrinsics

def get_extrinsics(depth_profile, color_profile):
    # Get extrinsics from depth to color sensor
    extrinsics = depth_profile.get_extrinsics_to(color_profile)
    return extrinsics

def generate_yaml_config(depth_intrinsics, color_intrinsics, extrinsics):
    # Convert data to YAML format
    config = {
        'Camera': {
            'Depth': {
                'Camera.fx': depth_intrinsics.fx,
                'Camera.fy': depth_intrinsics.fy,
                'Camera.cx': depth_intrinsics.ppx,
                'Camera.cy': depth_intrinsics.ppy,
                'Camera.k1': depth_intrinsics.coeffs[0],
                'Camera.k2': depth_intrinsics.coeffs[1],
                'Camera.p1': depth_intrinsics.coeffs[2],
                'Camera.p2': depth_intrinsics.coeffs[3],
                'Camera.k3': depth_intrinsics.coeffs[4],
                'Camera.width': depth_intrinsics.width,
                'Camera.height': depth_intrinsics.height,
                'Camera.model': 'pinhole'  # Assuming pinhole model, adjust if necessary
            },
            'Color': {
                'Camera.fx': color_intrinsics.fx,
                'Camera.fy': color_intrinsics.fy,
                'Camera.cx': color_intrinsics.ppx,
                'Camera.cy': color_intrinsics.ppy,
                'Camera.k1': color_intrinsics.coeffs[0],
                'Camera.k2': color_intrinsics.coeffs[1],
                'Camera.p1': color_intrinsics.coeffs[2],
                'Camera.p2': color_intrinsics.coeffs[3],
                'Camera.k3': color_intrinsics.coeffs[4],
                'Camera.width': color_intrinsics.width,
                'Camera.height': color_intrinsics.height,
                'Camera.model': 'pinhole'  # Assuming pinhole model, adjust if necessary
            }
        },
        'Extrinsics': {
            'DepthToColor': {
                'Rotation': extrinsics.rotation,  # Directly using list-like format
                'Translation': extrinsics.translation  # Directly using list-like format
            }
        }
    }

    # Print YAML content
    print("Generated YAML Configuration:")
    print(yaml.dump(config, default_flow_style=False))

    # Write to YAML file
    with open('orb_slam_config.yaml', 'w') as file:
        yaml.dump(config, file, default_flow_style=False)

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

        # Get intrinsics and extrinsics
        depth_intrinsics = get_intrinsics(depth_stream)
        color_intrinsics = get_intrinsics(color_stream)
        extrinsics = get_extrinsics(depth_stream, color_stream)
        
        # Print and generate YAML config file
        generate_yaml_config(depth_intrinsics, color_intrinsics, extrinsics)

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()
