import cv2
import os
import sys
import glob
import numpy as np
import time
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
import allan_variance

capture_checkerboard_size = (7, 9)  # Checkerboard size to detect during capture
max_checkerboard_size = (8, 11)  # Maximum checkerboard size to check during post-processing
scale = 15  # mm size of each checker

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Specify desired resolution here
camera_resolution = (1280, 720)  # Example resolution (width, height)

def img_dir(i):
    s = 'calimgs/' + str(i)
    if not os.path.exists(s):
        os.makedirs(s)
    return s

def load_realsense_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, camera_resolution[0], camera_resolution[1], rs.format.bgr8, 30)
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    pipeline.start(config)
    return pipeline

def preprocess_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def find_checkerboard(frame, size, draw_debug=False):
    processed = preprocess_image(frame)
    
    flags = [
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS,
        cv2.CALIB_CB_FAST_CHECK,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_FAST_CHECK
    ]
    
    for flag in flags:
        ret, corners = cv2.findChessboardCorners(processed, size, flag)
        if ret:
            corners_refined = cv2.cornerSubPix(processed, corners, (11, 11), (-1, -1), criteria)
            if draw_debug:
                debug_frame = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(debug_frame, size, corners_refined, ret)
                cv2.imshow(f"Debug Checkerboard {size}", debug_frame)
            return True, corners_refined

    if draw_debug:
        debug_frame = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)
        cv2.putText(debug_frame, "Processed Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Debug", debug_frame)
    
    return False, None

def save_image(frame, id_, pic_id):
    fn = img_dir(id_) + f"/cal-{pic_id}.jpg"
    cv2.imwrite(fn, frame)
    print(f"Image saved as {fn}")

def update_live(frame_r, id_, pic_id, last_save_time, accel_data, gyro_data):
    current_time = time.time()
    ret, corners = find_checkerboard(frame_r, capture_checkerboard_size, draw_debug=True)

    if ret:
        status_text = f"Checkerboard {capture_checkerboard_size} Detected"
        frame_to_save = frame_r.copy()
        frame_r = cv2.drawChessboardCorners(frame_r, capture_checkerboard_size, corners, ret)
        if current_time - last_save_time >= 1:
            save_image(frame_to_save, id_, pic_id)
            # Save IMU data
            save_imu_data(id_, pic_id, accel_data, gyro_data)
            pic_id += 1
            last_save_time = current_time
    else:
        status_text = "Checkerboard Not Detected"

    cv2.putText(frame_r, status_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    return frame_r, pic_id, last_save_time

def save_imu_data(id_, pic_id, accel_data, gyro_data):
    imu_fn = img_dir(id_) + f"/imu-{pic_id}.txt"
    with open(imu_fn, 'w') as f:
        f.write(f"Accelerometer: {accel_data}\n")
        f.write(f"Gyroscope: {gyro_data}\n")
    print(f"IMU data saved as {imu_fn}")

def loop(pipeline, id_):
    print("Press 'd' to toggle debug view, 'q' to quit")
    pic_id = 1
    debug_view = False
    last_save_time = time.time() - 1

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        accel_frame = frames.first(rs.stream.accel)
        gyro_frame = frames.first(rs.stream.gyro)
        
        if not color_frame or not accel_frame or not gyro_frame:
            continue

        frame_r = np.asanyarray(color_frame.get_data())
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()

        frame_a, pic_id, last_save_time = update_live(frame_r.copy(), id_, pic_id, last_save_time, accel_data, gyro_data)

        cv2.imshow("FEED", frame_a)

        key = cv2.waitKey(1)
        if key == ord('d'):
            debug_view = not debug_view
            print(f"Debug view {'enabled' if debug_view else 'disabled'}")
        elif key == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()

def calculate_allan_deviation(data):
    """
    Calculates the Allan deviation for the given data.

    Args:
        data (list): A list of IMU readings (e.g., [[x1, y1, z1], [x2, y2, z2], ...]).

    Returns:
        tuple: (taus, allan_devs) 
               - taus: Averaging times (tau values).
               - allan_devs: Allan deviation values corresponding to each tau.
    """
    taus, allan_devs = [], []
    for i in range(3):  # Calculate for x, y, z axes
        tau, dev, _, _ = allan_variance(np.array(data)[:, i], rate=200) # Assuming IMU rate is 200 Hz
        taus.append(tau)
        allan_devs.append(dev)
    return taus, allan_devs 

def extract_bias_instability(allan_devs, taus):
    """
    Extracts the bias instability from the Allan deviation curve.

    This is a simplified example. A more robust method may be needed
    depending on the characteristics of your IMU data.

    Args:
        allan_devs (list): List of Allan deviation values.
        taus (list): List of corresponding averaging times (tau values).

    Returns:
        float: The estimated bias instability.
    """
    # Find the index of the minimum Allan deviation
    min_index = np.argmin(allan_devs)
    # Get the bias instability (Allan deviation at the minimum point)
    bias_instability = allan_devs[min_index]
    return bias_instability

def update_calc(frame_r, points2d):
    ret, corners = find_checkerboard(frame_r, capture_checkerboard_size)
    if ret:
        points2d.append(corners)
    return points2d

def calculate(id_):
    points2d = []
    capture_object_p3d = np.zeros((1, capture_checkerboard_size[0] * capture_checkerboard_size[1], 3), np.float32)
    capture_object_p3d[0, :, :2] = np.mgrid[0:capture_checkerboard_size[0], 0:capture_checkerboard_size[1]].T.reshape(-1, 2) * scale

    print("Loading images...\n")
    for fn in glob.glob(img_dir(id_) + '/*.jpg'):
        print(f"Loading image {fn}")
        frame_r = cv2.imread(fn)
        points2d = update_calc(frame_r, points2d)

    if len(points2d) == 0:
        print("No images with detected corners found. Calibration cannot proceed.")
        return None, None, None, None, None

    all_corners = {}
    for size_x in range(max_checkerboard_size[0], capture_checkerboard_size[0] - 1, -1):
        for size_y in range(max_checkerboard_size[1], capture_checkerboard_size[1] - 1, -1):
            size = (size_x, size_y)
            for fn in glob.glob(img_dir(id_) + '/*.jpg'):
                if fn in all_corners:
                    continue
                print(f"Checking for checkerboard {size} in image {fn}")
                frame_r = cv2.imread(fn)
                if size[0] > 2 and size[1] > 2:
                    ret, corners = find_checkerboard(frame_r, size)
                    if ret:
                        print(f"Found pattern for image {fn} with size {size}")
                        all_corners[fn] = (size, corners)

    if len(all_corners) == 0:
        print("No images with detected corners for the given sizes. Calibration cannot proceed.")
        return None, None, None, None, None

    points3d = []
    points2d = []
    for fn, (size, corners) in all_corners.items():
        object_p3d = np.zeros((1, size[0] * size[1], 3), np.float32)
        object_p3d[0, :, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2) * scale
        points3d.append(object_p3d)
        points2d.append(corners)

    print("\nCalculating intrinsic matrix...")
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
            points3d, points2d, frame_r.shape[:2][::-1], None, None)

    if not ret:
        print("Calibration failed.")
        return None, None, None, None, None

    print("\nResulting matrix:")
    print(matrix)
    print("\nDistortion values:")
    print(distortion)

    T_imu_cam = estimate_imu_camera_transform(id_, r_vecs, t_vecs)

    return matrix, distortion, r_vecs, t_vecs, T_imu_cam

def estimate_imu_camera_transform(id_, r_vecs, t_vecs):
    imu_data = []
    for fn in glob.glob(img_dir(id_) + '/imu-*.txt'):
        with open(fn, 'r') as f:
            lines = f.readlines()
            accel = eval(lines[0].split(': ')[1])
            gyro = eval(lines[1].split(': ')[1])
            imu_data.append((accel, gyro))
    
    # Estimate rotation between camera and IMU
    gravity_vectors = []
    for accel, _ in imu_data:
        gravity_vectors.append([-accel.x, -accel.y, -accel.z])
    
    camera_z_vectors = []
    for r_vec in r_vecs:
        R, _ = cv2.Rodrigues(r_vec)
        camera_z_vectors.append(R[:, 2])
    
    R_imu_cam = Rotation.align_vectors(np.array(gravity_vectors), np.array(camera_z_vectors))[0].as_matrix()
    
    # Estimate translation between camera and IMU
    # This is a simplified approach and might need refinement based on your specific setup
    t_imu_cam = np.mean(t_vecs, axis=0).reshape(3, 1)
    
    # Construct the 4x4 transformation matrix
    T_imu_cam = np.eye(4)
    T_imu_cam[:3, :3] = R_imu_cam
    T_imu_cam[:3, 3] = t_imu_cam.flatten()
    
    return T_imu_cam

def calculate_imu_parameters(id_):
    accel_data = []
    gyro_data = []
    
    for fn in glob.glob(img_dir(id_) + '/imu-*.txt'):
        with open(fn, 'r') as f:
            lines = f.readlines()
            accel = eval(lines[0].split(': ')[1])
            gyro = eval(lines[1].split(': ')[1])
            accel_data.append([accel.x, accel.y, accel.z])
            gyro_data.append([gyro.x, gyro.y, gyro.z])
    
    accel_mean = np.mean(accel_data, axis=0)
    gyro_mean = np.mean(gyro_data, axis=0)
    
    accel_noise = np.std(accel_data, axis=0)
    gyro_noise = np.std(gyro_data, axis=0)
    
    # Calculate GyroWalk and AccelWalk (Bias Instability)
    # NOTE: Implementation for Allan Deviation and bias instability extraction not included
    # You will need to implement these functions or use a suitable library
    # Calculate GyroWalk and AccelWalk (Bias Instability)
    gyro_taus, gyro_allan_devs = calculate_allan_deviation(gyro_data) 
    accel_taus, accel_allan_devs = calculate_allan_deviation(accel_data)

    gyro_walk = []
    accel_walk = []

    for i in range(3):  # Calculate for x, y, z axes
        gyro_walk.append(extract_bias_instability(gyro_allan_devs[i], gyro_taus[i]))
        accel_walk.append(extract_bias_instability(accel_allan_devs[i], accel_taus[i]))

    gyro_walk = np.array(gyro_walk)
    accel_walk = np.array(accel_walk)

    
    return accel_mean, gyro_mean, accel_noise, gyro_noise, gyro_walk, accel_walk 

def print_orb_slam3_config(id_, matrix, distortion, accel_mean, gyro_mean, accel_noise, gyro_noise, gyro_walk, accel_walk, T_imu_cam):
    print(f"\nORB_SLAM3 Configuration for Camera {id_}:")
    print("-------------------------------------------")
    print(f"Camera.fx: {matrix[0, 0]}")
    print(f"Camera.fy: {matrix[1, 1]}")
    print(f"Camera.cx: {matrix[0, 2]}")
    print(f"Camera.cy: {matrix[1, 2]}")
    print(f"Camera.k1: {distortion[0, 0]}")
    print(f"Camera.k2: {distortion[0, 1]}")
    print(f"Camera.p1: {distortion[0, 2]}")
    print(f"Camera.p2: {distortion[0, 3]}")
    print(f"Camera.width: {camera_resolution[0]}")
    print(f"Camera.height: {camera_resolution[1]}")
    print(f"Camera.fps: 30")
    print(f"IMU.NoiseGyro: {gyro_noise.tolist()}")
    print(f"IMU.NoiseAcc: {accel_noise.tolist()}")
    print(f"IMU.GyroWalk: {gyro_walk.tolist()}")
    print(f"IMU.AccWalk: {accel_walk.tolist()}")
    print(f"IMU.Frequency: 200")
    print("IMU.T_b_c1: !!opencv-matrix")
    print("   rows: 4")
    print("   cols: 4")
    print("   dt: f")
    print("   data: [%.9f,%.9f,%.9f,%.9f," % tuple(T_imu_cam[0]))
    print("         %.9f,%.9f,%.9f,%.9f," % tuple(T_imu_cam[1]))
    print("         %.9f,%.9f,%.9f,%.9f," % tuple(T_imu_cam[2]))
    print("         %.9f,%.9f,%.9f,%.9f]" % tuple(T_imu_cam[3]))

def help():
    return """cal-cam.py <nominal_id> <command>

Commands:
    pics    take pictures for processing
    calc    calculate intrinsic camera properties and write to the database
    reset   delete calibration images for specified camera
"""

def main(args):
    if len(args) < 3:
        print("Invalid arguments.\n" + help())
        return

    nominal_id = int(args[1])
    cmd = args[2]
    i_dir = img_dir(nominal_id)

    if cmd == 'reset':
        os.system(f'rm -rf {i_dir}')
    elif cmd == 'pics':
        if not os.path.exists(i_dir):
            os.makedirs(i_dir)
        pipeline = load_realsense_camera()
        loop(pipeline, nominal_id)
    elif cmd == 'calc':
        if not os.path.exists(i_dir):
            print("Calibration images don't exist, please take some")
            return
        matrix, distortion, r_vecs, t_vecs, T_imu_cam = calculate(nominal_id)
        if matrix is not None and distortion is not None:
            print("\nIntrinsic matrix:\n", matrix)
            print("\nDistortion coefficients:\n", distortion)
            
            accel_mean, gyro_mean, accel_noise, gyro_noise, gyro_walk, accel_walk = calculate_imu_parameters(nominal_id)
            print("\nAccelerometer mean:", accel_mean)
            print("Gyroscope mean:", gyro_mean)
            print("Accelerometer noise:", accel_noise)
            print("Gyroscope noise:", gyro_noise)
            
            print("\nEstimated IMU to Camera Transformation:")
            print(T_imu_cam)
            
            print_orb_slam3_config(nominal_id, matrix, distortion, accel_mean, gyro_mean, accel_noise, gyro_noise, gyro_walk, accel_walk, T_imu_cam)
    else:
        print("Invalid command.\n" + help())

if __name__ == '__main__':
    main(sys.argv)