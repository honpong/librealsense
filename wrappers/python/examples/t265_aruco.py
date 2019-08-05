#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function

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

import pyrealsense2 as rs
import cv2
import numpy as np
import math as m
import matplotlib.pyplot as plt
import os
from collections import OrderedDict
import json
import argparse
import sys

visualize = False
validate = False  # use factory calibration as GT

# CALIB_FIX_SKEW - Fixes the skew (K[0][1]) to 0
# CALIB_USE_INTRINSIC_GUESS - uses the K and D input as a guess
# CALIB_RECOMPUTE_EXTRINSICS - Recomputing the pose of the target every iteration, if we don't do this it only computes it once (at the start)
flags = cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-9)

board_width = 16
board_height = 10
checker_size_m = 0.015
april_size_m = 0.0075
min_detections = 15

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)  # 16*10/2 = 80
board = cv2.aruco.CharucoBoard_create(board_width, board_height, checker_size_m, april_size_m, dictionary)
parameters = cv2.aruco.DetectorParameters_create()


import errno
def ensure_path(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

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



def visualize_markers(frame, markers, ids):
    frame_copy = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy, markers, ids)
    return frame_copy

def detect_markers(frame, K, D):
    (markers, ids, rejected) = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    if visualize:
        detections = visualize_markers(frame, markers, ids)
        cv2.imshow("detected markers", detections)
        cv2.waitKey(1)
        #cv2.imwrite("1_detect_markers.png", detections)

    (markers, ids, rejected, _) = cv2.aruco.refineDetectedMarkers(frame, board, markers, ids, rejected, errorCorrectionRate=0, parameters=parameters)

    if visualize:
        detections = visualize_markers(frame, markers, ids)
        cv2.imshow("detected markers (refined)", detections)
        cv2.waitKey(1)
        #cv2.imwrite("2_detect_markers_refined.png", detections)


    ok = ids is not None and len(ids) > min_detections
    if not ok:
        return (False, None, None, None)

    if ok:
        (num_refined, chess_corners, chess_ids) = cv2.aruco.interpolateCornersCharuco(markers, ids, frame, board, minMarkers=2) #, cameraMatrix=K) #, distCoeffs=D)

        frame_copy3 = frame.copy()
        cv2.aruco.drawDetectedCornersCharuco(frame_copy3, chess_corners, chess_ids)
        if visualize:
            cv2.imshow("detected interpol. corners", frame_copy3)
            cv2.waitKey(1)
            #cv2.imwrite("3_detect_corners_interpol.png", frame_copy3)

        ok = num_refined > 15

    if not ok:
        return (False, None, None, None)

    image_size = (frame.shape[1], frame.shape[0])

    object_points = []
    for cid in chess_ids:
        object_points.append(board.chessboardCorners[cid[0],:])
    object_points = np.array([object_points])
    image_points = np.reshape(np.array(chess_corners), (1, -1, 2))

    print(object_points.shape)
    #print(image_points.shape)
    #print(chess_ids.shape)
    return (True, object_points, image_points, chess_ids)

observations = {"left" : [], "right" : []}

def add_observation(camera_name, object_points, image_points, ids):
    global observations
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

def theta_d(theta, D):
    return theta*( 1+D[0]*np.power(theta,2)+D[1]*np.power(theta,4)+D[2]*np.power(theta,6)+D[3]*np.power(theta,8) )

def evaluate_calibration(object_points, image_points, identification, rvec, tvec, K, D, Korig, Dorig, pixel_thresh):
    if validate:
        print("dfx[%]:", (K[0,0]/Korig[0,0]-1)*100 )  # to percent
        print("dfy[%]:", (K[1,1]/Korig[1,1]-1)*100 )  # to percent
        print("cx[px]:", K[0,2]/Korig[0,2])
        print("cy[px]:", K[1,2]/Korig[1,2])
        print("/nratio fx/fy:", K[0,0]/K[1,1] )
        # TODO: D 70, 80, 85, 90

    plot_scatter = False
    rms = 0.0
    N = 0
    Nframes = len(object_points)  # number of frames
    inlier_object_points = []
    inlier_image_points = []
    for i in range(Nframes):
        proj = cv2.fisheye.projectPoints(object_points[i], rvec[i], tvec[i], K, D)
        proj_err = image_points[i][0] - proj[0][0]
        if plot_scatter:
            plt.scatter(proj_err[:,0], proj_err[:,1])

        Npoints = len(image_points[i][0])  # number of points
        inlier_object = []
        inlier_image = []
        for j in range(Npoints):
            #print(j)
            #print(image_points[i][0][j])
            #print(proj[i][0][j])
            proj_err = image_points[i][0][j] - proj[0][0][j]
            #print(proj_err)

            if plot_scatter:
                plt.text(proj_err[0], proj_err[1], str(i)+","+str(identification[i][j][0]))

            pt_rms = np.array(proj_err).dot(proj_err)
            if pt_rms < pixel_thresh:
                inlier_object.append(object_points[i][0][j])
                inlier_image.append(image_points[i][0][j])
            rms = rms + np.array(proj_err).dot(proj_err)
        if len(inlier_object) > 5:
            inlier_object = np.reshape(np.array(inlier_object), (1, -1, 3))
            inlier_image  = np.reshape(np.array(inlier_image), (1, -1, 2))
            inlier_object_points.append(inlier_object)
            inlier_image_points.append(inlier_image)
        N = N + Npoints
    rms = m.sqrt(rms/N)
    #print("rms:", rms)

    if plot_scatter:
        plt.savefig("reproj_err.png")
        #if visualize:
        #plt.show()
        plt.show(block=False)
        plt.pause(2)
        plt.close()

    # support

    # distortion
    if visualize:
        theta = np.linspace(0,m.pi/2)

        Dmean = np.array([-0.00626438, 0.0493399, -0.0463255, 0.00896666])
        Dorig = [-0.00217445590533316,  	0.0376055203378201,  	-0.0364043116569519,  0.00593686196953058]  # TODO: read

        plt.figure()
        plt.xlabel('deg')

        plt.plot(theta, theta_d(theta, D))
        #plt.plot(theta, theta_d(theta, Dmean), '--')
        #plt.plot(theta, theta_d(theta, Dorig))

        #plt.plot(theta/m.pi*180, theta_d(theta, D)/theta - theta_d(theta, Dmean)/theta)

        #plt.plot([0,90],[0,0],'k--')
        #plt.plot([0,90],[-0.01,-0.01],'--')

        plt.legend(['new','mean','orig'])
        plt.savefig("theta_d.png")
        #plt.show()
        plt.show(block=False)
        plt.pause(3)
        plt.close()

    return (inlier_object_points, inlier_image_points)

def add_camera_calibration(K,D):
    cam = OrderedDict()
    cam['size_px'] = [848, 800]  # TODO: read
    cam['center_px'] = [K[0,2], K[1,2]]
    cam['focal_length_px'] = [K[0,0], K[1,1]]
    cam['distortion'] = OrderedDict()
    cam['distortion']['k'] = D.flatten().tolist()
    cam['distortion']['type'] = 'kannalabrandt4'
    return cam

def save_calibration(directory, sn, K1, D1, K2, D2):
    calib = OrderedDict()  # in order (cam1,cam2)
    calib['device_id'] = sn
    calib['device_type'] = ""
    calib['calibration_version'] = 10
    calib['cameras'] = []
    calib['cameras'].append( add_camera_calibration(K1,D1) )
    calib['cameras'].append( add_camera_calibration(K2,D2) )

    if not os.path.exists(directory):
        os.mkdir(directory)
    with open(directory + '/cam' + str(sn)[-4:] + '_intrinsics.json', 'w') as f:
        json.dump(calib, f, indent=4)

def calibrate_observations(camera_name, Korig, Dorig):
    obs = observations[camera_name]
    object_points = []
    image_points = []
    identification = []
    for (id_to_image_pt, obj_i, img_i, ids) in obs:
        #print(obj_i.shape)
        object_points.append(obj_i)
        image_points.append(img_i)
        identification.append(ids)
    #Dguess = np.zeros((4,1))
    Dguess = np.array([-0.00626438, 0.0493399, -0.0463255, 0.00896666])
    image_size = (848, 800)
    Kguess = np.zeros((3,3))
    Kguess[0][0] = Kguess[1][1] = 285
    Kguess[0][2] = image_size[0]/2
    Kguess[1][2] = image_size[1]/2

    initial_flags = flags
    (rmse, K, D, rvec, tvec) = cv2.fisheye.calibrate(objectPoints = object_points,
                                                                                    imagePoints = image_points,
                                                                                    image_size = image_size,
                                                                                    K = Kguess,
                                                                                    D = Dguess,
                                                                                    flags = initial_flags,
                                                                                    criteria = criteria)
    print("rmse", rmse)
    print("camera", K)
    print("distortion_coeffs", np.array2string(D, separator=', '))
    #print("rvec", rvec)
    #print("tvec", tvec)


    (inlier_object, inlier_image) = evaluate_calibration(object_points, image_points, identification, rvec, tvec, K, Dguess, Korig, Dorig, rmse*3) # 2 pixel
    # object points is a python list of M items which are each (1, N, 3) np.arrays
    # image points is a python list of M items which are each (1, N, 2) np.arrays
    print(len(inlier_image), "images remaining")
    final_flags = flags
    (rmse, K, D, rvec, tvec) = cv2.fisheye.calibrate(objectPoints = inlier_object,
                                                                                    imagePoints = inlier_image,
                                                                                    image_size = image_size,
                                                                                    K = K,
                                                                                    D = D,
                                                                                    flags = final_flags,
                                                                                    criteria = criteria)
    print("refined rmse", rmse)
    print("camera", K)
    print("distortion_coeffs", np.array2string(D, separator=', '))

    return (rmse, K, D)


parser = argparse.ArgumentParser()
parser.add_argument('--path',   default="cam", help='output path prefix')
parser.add_argument('--record',                help='record <filename> to rosbag')
parser.add_argument('--play',                  help='playback <filename> from rosbag')
args = parser.parse_args()

if args.record is not None and args.play is not None:
    print("Error: Cannot specify record and play simulataneously\n")
    parser.print_help()
    sys.exit(1)

try:
    pipe = rs.pipeline()
    cfg = rs.config()
    if args.record:
        cfg.enable_record_to_file(args.record)  # record to rosbag (all streams)
    elif args.play:
        cfg.enable_device_from_file(args.play, repeat_playback = False)  # playback from rosbag
    pipe.start(cfg)

    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()

    dev = profiles.get_device()

    if args.play:
        playback = dev.as_playback()  # loop playback, check timestamp
        playback.set_real_time(False)  # playback non-RT not to drop frames (https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1playback.html#aca02f3a4b789b150d2b7699d27a94324)

    sn = dev.get_info(rs.camera_info.serial_number)
    print("Serial number:", sn)
    args.path  = args.path + "_" + sn
    ensure_path(args.path)

    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}
    calibration_folder = "tmp"
    measured = 0

    # Set up an OpenCV window to visualize the results
    if visualize:
        WINDOW_TITLE = 'Realsense'
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)

    # Print information about both cameras
    print("Left camera:",  intrinsics["left"])
    print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K0l  = camera_matrix(intrinsics["left"])
    D0l  = fisheye_distortion(intrinsics["left"])
    K0r = camera_matrix(intrinsics["right"])
    D0r = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get thre relative extrinsics between the left and right camera
    if args.play is None: # can't get this from playback
        (R, T) = get_extrinsics(streams["right"], streams["left"])

    z = 0
    nobservations = 0
    K1 = D1 = K2= D2 = None
    while True:
        ts = None
        success, frames = pipe.try_wait_for_frames(timeout_ms=1000)
        if not success:
            break
        if frames.is_frameset():
            z+=1
            #print(z)
            frameset = frames.as_frameset()
            f1 = frameset.get_fisheye_frame(1).as_video_frame()
            f2 = frameset.get_fisheye_frame(2).as_video_frame()
            left_data = np.asanyarray(f1.get_data())
            right_data = np.asanyarray(f2.get_data())
            ts = frameset.get_timestamp() # valid
            frame_copy = {"left"  : left_data,
                          "right" : right_data}

        if z % 20 != 0:  # subsample
            continue

        if args.record:
            cv2.imshow("left", frame_copy["left"])
            cv2.waitKey(1)
            continue

        # Check if the camera has acquired any frames
        if ts is not None:
            (ok_l, object_points_l, image_points_l, chess_ids_l) = detect_markers(frame_copy["left"], K0l, D0l)
            if ok_l:
                (ok_r, object_points_r, image_points_r, chess_ids_r) = detect_markers(frame_copy["right"], K0r, D0r)
                if ok_r:
                    add_observation("left", object_points_l, image_points_l, chess_ids_l)
                    add_observation("right", object_points_r, image_points_r, chess_ids_r)
                    nobservations += 1

    print("Finished replay")
    print("Calibrating", nobservations)
    (rms1, K1, D1) = calibrate_observations("left", K0l, D0l)
    (rms2, K2, D2) = calibrate_observations("right", K0r, D0r)

    save_calibration('cals', sn, K1, D1, K2, D2)

    f = open(args.path + '/rmse.txt','a')
    np.savetxt(f, np.array([rms1, rms2]).reshape(1,2))
    f.close()

finally:
    #d(rms2, K2, D2) = calibrate_observations("right", K0r, D0r)
    #cv2.destroyAllWindows()
    pipe.stop()
