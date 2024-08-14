# Import necessary libraries
import pyrealsense2 as rs
import cv2
import open3d as o3d
import numpy as np
from math import tan, pi
from threading import Lock

# Function to get rotation (R) and translation (T) matrices from source to destination camera
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3, 3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

# Function to create a camera matrix (K) from RealSense intrinsics
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx, 0, intrinsics.ppx],
                     [0, intrinsics.fy, intrinsics.ppy],
                     [0, 0, 1]])

# Function to get fisheye distortion coefficients from RealSense intrinsics
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# Initialize a mutex to safely share data between threads
frame_mutex = Lock()
frame_data = {"left": None, "right": None, "timestamp_ms": None}

# Callback function to handle new frames from the RealSense camera
def callback(frame):
    global frame_data
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        with frame_mutex:
            frame_data["left"] = left_data
            frame_data["right"] = right_data
            frame_data["timestamp_ms"] = ts

# Initialize and configure the RealSense pipeline
pipe = rs.pipeline()
cfg = rs.config()
pipe.start(cfg, callback)

try:
    # Set up an OpenCV window for visualization
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    # Configure the OpenCV stereo algorithm
    window_size = 5
    min_disp = 0
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=16,
        P1=8 * 3 * window_size**2,
        P2=32 * 3 * window_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    # Retrieve stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()
    streams = {
        "left": profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
        "right": profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()
    }
    intrinsics = {
        "left": streams["left"].get_intrinsics(),
        "right": streams["right"].get_intrinsics()
    }

    # Print camera information
    print("Left camera:", intrinsics["left"])
    print("Right camera:", intrinsics["right"])

    # Translate intrinsics to OpenCV format
    K_left = camera_matrix(intrinsics["left"])
    D_left = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get relative extrinsics between the left and right cameras
    (R, T) = get_extrinsics(streams["left"], streams["right"])

    # Calculate the undistorted focal length for stereo rectification
    stereo_fov_rad = 90 * (pi / 180)  # Desired FOV of 90 degrees
    stereo_height_px = 300  # Stereo output height in pixels
    stereo_focal_px = stereo_height_px / 2 / tan(stereo_fov_rad / 2)

    # Set rotation matrices for rectification
    R_left = np.eye(3)
    R_right = R

    # Calculate stereo output size and projection matrices
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1) / 2 + max_disp
    stereo_cy = (stereo_height_px - 1) / 2

    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0, 0, 1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0] * stereo_focal_px

    # Construct Q matrix for reprojecting disparity to 3D points
    Q = np.array([[1, 0, 0, -(stereo_cx - max_disp)],
                  [0, 1, 0, -stereo_cy],
                  [0, 0, 0, stereo_focal_px],
                  [0, 0, -1 / T[0], 0]])

    # Create undistortion and rectification maps
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left": (lm1, lm2), "right": (rm1, rm2)}

    mode = "stack"

    # Initialize Open3D visualization
    window_height = 900
    window_width = 900
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Visualization", height=window_height, width=window_width)

    # Set visualization options
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2
    opt.line_width = 7
    opt.light_on = True

    # Initialize point cloud for visualization
    pcd = o3d.geometry.PointCloud()
    pcd_coor = [[0, 0, 0]]
    pcd.points = o3d.utility.Vector3dVector(pcd_coor)
    vis.add_geometry(pcd)

    # Add coordinate frame to the scene
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
    vis.add_geometry(axis_pcd)

    # Set view control parameters
    ctr = vis.get_view_control()
    ctr.set_lookat(np.array([0, 0, 0]))
    ctr.set_constant_z_far(200)
    ctr.rotate(0, -1000.0)
    ctr.set_zoom(2.5)

    vis.poll_events()
    vis.update_renderer()

    counter = 1
    while True:
        # Check if new frames are available
        with frame_mutex:
            valid = frame_data["timestamp_ms"] is not None

        if valid:
            # Copy the frame data for processing
            with frame_mutex:
                frame_copy = {"left": frame_data["left"].copy(), "right": frame_data["right"].copy()}

            # Undistort and rectify the frames
            center_undistorted = {
                "left": cv2.remap(frame_copy["left"], undistort_rectify["left"][0], undistort_rectify["left"][1], cv2.INTER_LINEAR),
                "right": cv2.remap(frame_copy["right"], undistort_rectify["right"][0], undistort_rectify["right"][1], cv2.INTER_LINEAR)
            }

            # Compute disparity map
            disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0
            disparity = disparity[:, max_disp:]

            # Reproject disparity to 3D points
            points = cv2.reprojectImageTo3D(disparity, Q)
            points = points.reshape(-1, 3)
            idx = ~np.isinf(points[:, 0:3]).any(axis=1)
            points = points[idx, :]
            idx_2 = points[:, 2] > 0
            points = points[idx_2, :]

            # Colorize disparity map
            disp_vis = 255 * (disparity - min_disp) / num_disp
            disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis, 1), cv2.COLORMAP_JET)
            color_image = cv2.cvtColor(center_undistorted["left"][:, max_disp:], cv2.COLOR_GRAY2RGB)

            # Update point cloud with new points and colors
            pcd.points = o3d.utility.Vector3dVector(points)
            color_vector = color_image.reshape(-1, 3)
            color_vector = color_vector[idx, :]
            color_vector = color_vector[idx_2, :] / 255
            pcd.colors = o3d.utility.Vector3dVector(color_vector)
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            # Display the results using OpenCV
            if mode == "stack":
                cv2.imshow(WINDOW_TITLE, np.hstack((color_image, disp_color)))
            if mode == "overlay":
                ind = disparity >= min_disp
                color_image[ind, 0] = disp_color[ind, 0]
                color_image[ind, 1] = disp_color[ind, 1]
                color_image[ind, 2] = disp_color[ind, 2]
                cv2.imshow(WINDOW_TITLE, color_image)

        # Handle key presses
        key = cv2.waitKey(30)
        if key == ord('s'):
            mode = "stack"
        if key == ord('o'):
            mode = "overlay"
        if key == 32:  # Space key
            print("Collecting depth")
            o3d.io.write_point_cloud(f"./t265_depth_{counter:03d}.ply", pcd, write_ascii=True)
            counter += 1
            key = ''
        if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            break
finally:
    pipe.stop()
