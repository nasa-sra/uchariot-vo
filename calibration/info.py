import pyrealsense2 as rs

# Create a context
ctx = rs.context()

# Get the first connected device
dev = ctx.get_device(0)

# Get the intrinsic parameters
intrinsics = dev.get_intrinsics()

# Print the intrinsic parameters
print(intrinsics)