import cv2
import os
import sys
import glob
import numpy as np
import time

complex_camera_read = False
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

def load_camera(camera_id):
    if complex_camera_read:
        s = f"v4l2src device=/dev/video{camera_id} ! video/x-raw, width={camera_resolution[0]}, height={camera_resolution[1]} ! videoconvert ! video/x-raw,format=BGR ! appsink"
        camera = cv2.VideoCapture(s)
    else:
        camera = cv2.VideoCapture(camera_id)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_resolution[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_resolution[1])
    return camera

def preprocess_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def find_checkerboard(frame, size, draw_debug=False):
    processed = preprocess_image(frame)
    
    # Try different flags combinations
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

def update_live(frame_r, id_, pic_id, last_save_time):
    current_time = time.time()
    ret, corners = find_checkerboard(frame_r, capture_checkerboard_size, draw_debug=True)

    if ret:
        status_text = f"Checkerboard {capture_checkerboard_size} Detected"
        frame_to_save = frame_r.copy()  # Create a copy of the frame before drawing the chessboard pattern
        frame_r = cv2.drawChessboardCorners(frame_r, capture_checkerboard_size, corners, ret)
        # Save image every second
        if current_time - last_save_time >= 1:
            save_image(frame_to_save, id_, pic_id)
            pic_id += 1
            last_save_time = current_time
    else:
        status_text = "Checkerboard Not Detected"

    cv2.putText(frame_r, status_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    return frame_r, pic_id, last_save_time

def loop(camera, id_):
    print("Press 'd' to toggle debug view, 'q' to quit")
    pic_id = 1
    debug_view = False
    last_save_time = time.time() - 1  # Initialize to allow saving immediately on first detection

    while True:
        ret, frame_r = camera.read()
        if not ret:
            break

        frame_a, pic_id, last_save_time = update_live(frame_r.copy(), id_, pic_id, last_save_time)

        cv2.imshow("FEED", frame_a)

        key = cv2.waitKey(1)
        if key == ord('d'):
            debug_view = not debug_view
            print(f"Debug view {'enabled' if debug_view else 'disabled'}")
        elif key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

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
        return None, None

    # Post-processing to check for all sizes starting from max_checkerboard_size down to capture_checkerboard_size
    all_corners = {}
    for size_x in range(max_checkerboard_size[0], capture_checkerboard_size[0] - 1, -1):
        for size_y in range(max_checkerboard_size[1], capture_checkerboard_size[1] - 1, -1):
            size = (size_x, size_y)
            for fn in glob.glob(img_dir(id_) + '/*.jpg'):
                if fn in all_corners:
                    continue
                print(f"Checking for checkerboard {size} in image {fn}")
                frame_r = cv2.imread(fn)
                if size[0] > 2 and size[1] > 2:  # Ensure valid size
                    ret, corners = find_checkerboard(frame_r, size)
                    if ret:
                        print(f"Found pattern for image {fn} with size {size}")
                        all_corners[fn] = (size, corners)

    if len(all_corners) == 0:
        print("No images with detected corners for the given sizes. Calibration cannot proceed.")
        return None, None

    points3d = []
    points2d = []
    for fn, (size, corners) in all_corners.items():
        # Create object points for current checkerboard size
        object_p3d = np.zeros((1, size[0] * size[1], 3), np.float32)
        object_p3d[0, :, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2) * scale
        points3d.append(object_p3d)
        points2d.append(corners)

    print("\nCalculating intrinsic matrix...")
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
            points3d, points2d, frame_r.shape[:2][::-1], None, None)

    if not ret:
        print("Calibration failed.")
        return None, None

    print("\nResulting matrix:")
    print(matrix)
    print("\nDistortion values:")
    print(distortion)

    # Extract intrinsic parameters
    fx = matrix[0, 0]
    fy = matrix[1, 1]
    cx = matrix[0, 2]
    cy = matrix[1, 2]

    # Extract distortion coefficients
    k1 = distortion[0, 0]
    k2 = distortion[0, 1]
    p1 = distortion[0, 2]
    p2 = distortion[0, 3]

    print(f"\nCamera1 Parameters:")
    print(f"Camera1.fx: {fx}")
    print(f"Camera1.fy: {fy}")
    print(f"Camera1.cx: {cx}")
    print(f"Camera1.cy: {cy}\n")
    print(f"Camera1.k1: {k1}")
    print(f"Camera1.k2: {k2}")
    print(f"Camera1.p1: {p1}")
    print(f"Camera1.p2: {p2}")

    return matrix, distortion

def help():
    return """cal-cam.py <nominal_id> <command> [camera_id]

Commands:
    pics    take pictures for processing
    calc    calculate intrinsic camera properties and write to the database
    reset   delete calibration images for specified camera

Options:
    camera_id   (optional) Specify the camera ID (default is 0)
"""

def main(args):
    if len(args) < 3:
        print("Invalid arguments.\n" + help())
        return

    nominal_id = int(args[1])
    cmd = args[2]
    camera_id = int(args[3]) if len(args) > 3 else 0
    i_dir = img_dir(nominal_id)

    if cmd == 'reset':
        os.system(f'rm -rf {i_dir}')
    elif cmd == 'pics':
        if not os.path.exists(i_dir):
            os.makedirs(i_dir)
        camera = load_camera(camera_id)
        loop(camera, nominal_id)
    elif cmd == 'calc':
        if not os.path.exists(i_dir):
            print("Calibration images don't exist, please take some")
            return
        matrix, distortion = calculate(nominal_id)
        if matrix is not None and distortion is not None:
            print("\nIntrinsic matrix:\n", matrix)
            print("\nDistortion coefficients:\n", distortion)
    else:
        print("Invalid command.\n" + help())

if __name__ == '__main__':
    main(sys.argv)