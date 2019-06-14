#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function
import sys

"""
First, set up the virtual enviroment:

$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install opencv-contrib-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings

Then, for every new terminal:

$ source py3librs/bin/activate  # Activate the virtual environment
$ python3 t265_aruco.py        # Run the example
"""

# First import the library
import pyrealsense2 as rs

# Import OpenCV and numpy
import cv2
import numpy as np

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# Set up a mutex to share data between threads 
from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None
              }

"""
This callback is called on a separate thread, so we must use a mutex
to ensure that data is synchronized properly. We should also be
careful not to do much work on this thread to avoid data backing up in the
callback queue.
"""
def callback(frame):
    global frame_data
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["timestamp_ms"] = ts
        frame_mutex.release()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and stream everything
cfg = rs.config()

# Start streaming with our callback
pipe.start(cfg, callback)

board_width = 16
board_height = 10
checker_size_m = 0.015
april_size_m = 0.0075
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(board_width, board_height, checker_size_m, april_size_m, dictionary)
parameters = cv2.aruco.DetectorParameters_create()
#parameters.adaptiveThreshWinSizeStep = 2
#parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

def detect_markers(frame):
    (markers, ids, rejected) = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    frame_copy = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy, markers, ids)
    cv2.imshow("detected markers", frame_copy)
    cv2.waitKey(1)
    cv2.imwrite("1_detect_markers.png", frame_copy)

    (markers, ids, rejected, _) = cv2.aruco.refineDetectedMarkers(frame, board, markers, ids, rejected, errorCorrectionRate=1, parameters=parameters)

    frame_copy2 = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy2, markers, ids)
    cv2.imshow("detected markers (refined)", frame_copy2)
    cv2.waitKey(1)
    cv2.imwrite("2_detect_markers_refined.png", frame_copy2)

    ok = ids is not None and len(ids) > 15
    if ok:
        (num_refined, chess_corners, chess_ids) = cv2.aruco.interpolateCornersCharuco(markers, ids, frame, board, minMarkers=1)

        frame_copy3 = frame.copy()
        cv2.aruco.drawDetectedCornersCharuco(frame_copy3, chess_corners, chess_ids)
        cv2.imshow("detected interpol. corners", frame_copy3)
        cv2.waitKey(1)
        cv2.imwrite("3_detect_corners_interpol.png", frame_copy3)

        ok = num_refined > 15

    if not ok:
        return (False, None, None, None)

    image_size = (frame.shape[1], frame.shape[0])

    object_points = []
    for cid in chess_ids:
        object_points.append(board.chessboardCorners[cid[0],:])
    object_points = np.array([object_points])
    image_points = np.reshape(np.array(chess_corners), (1, -1, 2))
    return (True, object_points, image_points, chess_ids)

    """
        K_guess = np.array([[285,   0, (image_size[0]-1)/2],
                            [  0, 285, (image_size[1]-1)/2],
                            [  0,   0, 1]])
        D_guess = np.zeros((4,1))
        D_guess = np.array([-0.000624021, 0.0357663, -0.0336763, 0.00530492])
        print(object_points.shape)
        print(object_points.dtype)
        object_points = np.reshape(object_points, (1, 1, -1, 3))
        image_points = np.reshape(image_points, (1, 1, -1, 2))
        flags = cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS

        (rms_error, camera_matrix, distortion_coeffs, rvec, tvec) = cv2.fisheye.calibrate(objectPoints = object_points,
                                                                                        imagePoints = image_points,
                                                                                        image_size = image_size,
                                                                                        K = K_guess,
                                                                                        D = D_guess,
                                                                                        flags = flags)
    """

observations = {"left" : [], "right" : []}

def add_observation(camera_name, object_points, image_points, ids):
    id_to_image_pt = {}
    for i in range(len(ids)):
        id_to_image_pt[ids[i,0]] = image_points[0,i,:]

    observations[camera_name].append((id_to_image_pt, object_points, image_points, ids))
    print("Total observations for %s: %d" % (camera_name, len(observations[camera_name]))) 

def min_dist_for_id(camera_name, feature_id, image_point):
    min_dist = 1000
    for (id_to_image_pt, object_points, image_points, ids) in observations[camera_name]:
        if feature_id not in id_to_image_pt: continue
        dist = np.linalg.norm(id_to_image_pt[feature_id] - image_point)
        if dist < min_dist:
            min_dist = dist
    return min_dist

def calibrate_observations(camera_name):
    obs = observations[camera_name]
    object_points = []
    image_points = []
    for (id_to_image_pt, obj_i, img_i, ids) in obs:
        print(obj_i.shape)
        object_points.append(obj_i)
        image_points.append(img_i)
    #op = np.concatenate(object_points, axis=1)
    #ip = np.concatenate(image_points, axis=1)
    #print("P{", op.shape)
    #print(ip.shape)
    #object_points = np.reshape(op, (1, 1, -1, 3))
    #image_points = np.reshape(ip, (1, 1, -1, 2))
    flags = cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
    D = np.array([-0.00626438, 0.0493399, -0.0463255, 0.00896666])
    K = np.zeros((3,3))
    image_size = (848, 800)

    (rms_error, camera_matrix, distortion_coeffs, rvec, tvec) = cv2.fisheye.calibrate(objectPoints = object_points,
                                                                                    imagePoints = image_points,
                                                                                    image_size = image_size,
                                                                                    K = None,
                                                                                    D = D,
                                                                                    flags = flags,
                                                                                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 500, 0.5))
    print("rms", rms_error)
    print("camera", camera_matrix)
    print("distortion_coeffs", distortion_coeffs)
    #print("rvec", rvec)
    #print("tvec", tvec)
    
try:
    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()
    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}
    calibration_folder = "tmp"
    measured = 0

    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)

    # Print information about both cameras
    print("Left camera:",  intrinsics["left"])
    print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get thre relative extrinsics between the left and right camera
    (R, T) = get_extrinsics(streams["right"], streams["left"])

    mode = "stack"

    while True:
        # Check if the camera has acquired any frames
        frame_mutex.acquire()
        valid = frame_data["timestamp_ms"] is not None
        frame_mutex.release()

        # If frames are ready to process
        if valid:
            # Hold the mutex only long enough to copy the stereo frames
            frame_mutex.acquire()
            frame_copy = {"left"  : frame_data["left"].copy(),
                          "right" : frame_data["right"].copy()}
            frame_mutex.release()

            (ok, object_points, image_points, chess_ids) = detect_markers(frame_copy["left"])
            if ok:
                good = True
                for i in range(len(chess_ids)):
                    if min_dist_for_id("left", chess_ids[i,0], image_points[0,i,:]) < 10:
                        good = False
                        break
                if good:
                    add_observation("left", object_points, image_points, chess_ids)
                    print("good left image")
                    if len(observations["left"]) > 10:
                        calibrate_observations("left")
            """
            (ok, object_points, image_points, chess_ids) = detect_markers(frame_copy["right"])
            if ok:
                for i in range(len(chess_ids)):
                    print(len(chess_ids),image_points.shape)
                    if min_dist_for_id(chess_ids[i], image_points[0,i,:]) > 10:
                        print("good right image")
                        break
            """

            color_image0 = cv2.cvtColor(frame_copy["left"], cv2.COLOR_GRAY2RGB)
            cv2.imshow(WINDOW_TITLE,color_image0)

        key = cv2.waitKey(1)
        if key == ord('i'): mode = "ids"
        if key == ord('o'): mode = "overlay"
        if key == ord('r'): mode = "r"
        if key == ord('p'): mode = "p"
        if key == ord('s'):
            cv2.imwrite("left.pgm", frame_copy["left"])
            cv2.imwrite("right.pgm", frame_copy["right"])
        if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            break
finally:
    pipe.stop()
