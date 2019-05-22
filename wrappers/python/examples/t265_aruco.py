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

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(16,10,.015,.0075,dictionary)

try:
    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()
    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}

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

    parameters =  cv2.aruco.DetectorParameters_create()

    #parameters.adaptiveThreshWinSizeStep = 2
    #parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

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

            imsize = frame_copy["left"].shape

            res = cv2.aruco.detectMarkers(frame_copy["left"],dictionary, parameters=parameters)
            #print(res)
            color_image0 = cv2.cvtColor(frame_copy["left"], cv2.COLOR_GRAY2RGB)
            if mode == "ids":
                cv2.aruco.drawDetectedMarkers(color_image0,res[0],res[1])

            res3 = cv2.aruco.refineDetectedMarkers(frame_copy["left"], board, res[0], res[1], res[2], errorCorrectionRate=1, parameters=parameters)

            if mode == "r":
                cv2.aruco.drawDetectedMarkers(color_image0,res3[0],res3[1])

            if len(res3[0])>0:
                res2 = cv2.aruco.interpolateCornersCharuco(res3[0],res3[1],frame_copy["left"],board, minMarkers=1)
                if res2[1] is not None and res2[2] is not None:
                    #print(res2)
                    allCorners = []
                    allIds = []
                    allCorners.append(res2[1])
                    allIds.append(res2[2])

                    if mode == "p":
                        cv2.aruco.drawDetectedCornersCharuco(color_image0, res2[1], res2[2])
                    #cv2.drawChessboardCorners(color_image0, (36,25), res2[1], len(res2[1]))

                    if False and len(res2[1]) > 3:
                        cameraMatrixInit = K_left

                        distCoeffsInit = np.zeros((5,1))
                        flags = (cv2.CALIB_RATIONAL_MODEL)
                        (ret, camera_matrix, distortion_coefficients0,
                        rotation_vectors, translation_vectors,
                        stdDeviationsIntrinsics, stdDeviationsExtrinsics,
                        perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                                        charucoCorners=allCorners,
                                        charucoIds=allIds,
                                        board=board,
                                        imageSize=imsize,
                                        cameraMatrix=cameraMatrixInit,
                                        distCoeffs=distCoeffsInit,
                                        flags=flags,
                                        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
                        print(distortion_coefficients0, end="\r", flush=True)
                        res = cv2.aruco.detectMarkers(frame_copy["left"],dictionary,cameraMatrix = camera_matrix, distCoeff = distortion_coefficients0)
                        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],frame_copy["left"], board, cameraMatrix = camera_matrix, distCoeffs = distortion_coefficients0)
                        cv2.aruco.drawDetectedCornersCharuco(color_image0, res2[1], res2[2], cornerColor=(0,255,0))

            color_image1 = cv2.cvtColor(frame_copy["right"], cv2.COLOR_GRAY2RGB)
            cv2.imshow(WINDOW_TITLE,color_image0)
            #cv2.imshow(WINDOW_TITLE, np.hstack((color_image0, color_image1)))

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
