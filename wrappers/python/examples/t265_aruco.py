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
np.set_printoptions(precision=6, suppress=True)
import math as m
import matplotlib.pyplot as plt
import os
from collections import OrderedDict
import json
import argparse
import sys

visualize = False
validate = True  # use factory calibration as GT
debug = False

flags = cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_PRINCIPAL_POINT
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-9)

board_width = 16
board_height = 10
checker_size_m = 0.015
april_size_m = 0.0075
min_detections = 5
n_stereo_matches = 15

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
board = cv2.aruco.CharucoBoard_create(board_width, board_height, checker_size_m, april_size_m, dictionary)
parameters = cv2.aruco.DetectorParameters_create()

import errno
def ensure_path(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

def visualize_markers(frame, markers, ids):
    frame_copy = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy, markers, ids)
    return frame_copy

def detect_markers(camera_name, frame, K, D):
    (markers, ids, rejected) = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    print("detections:", len(markers))

    global nobservations
    if visualize:
        detections = visualize_markers(frame, markers, ids)
        cv2.imshow("detected markers", detections)
        cv2.waitKey(1)
        cv2.imwrite(args.path + "/" + camera_name + "_%03d_1_detect_markers.png" % nobservations, detections)

    (markers, ids, rejected, _) = cv2.aruco.refineDetectedMarkers(frame, board, markers, ids, rejected, errorCorrectionRate=0, parameters=parameters)
    print("refined:", len(markers))

    if visualize:
        detections = visualize_markers(frame, markers, ids)
        cv2.imshow("detected markers (refined)", detections)
        cv2.waitKey(1)
        cv2.imwrite(args.path + "/" + camera_name + "_%03d_2_detect_markers_refined.png" % nobservations, detections)


    ok = ids is not None and len(ids) > min_detections
    if not ok:
        return (False, None, None, None)

    if ok:
        (num_refined, chess_corners, chess_ids) = cv2.aruco.interpolateCornersCharuco(markers, ids, frame, board, minMarkers=2) #, cameraMatrix=K) #, distCoeffs=D)
        print("chess corners:", len(chess_corners))

        frame_copy = frame.copy()
        cv2.aruco.drawDetectedCornersCharuco(frame_copy, chess_corners, chess_ids)
        if visualize:
            cv2.imshow("detected interpol. corners", frame_copy)
            cv2.waitKey(1)
            cv2.imwrite(args.path + "/" + camera_name + "_%03d_3_detect_corners_interpol.png" % nobservations, frame_copy)

        ok = num_refined > min_detections

    if not ok:
        return (False, None, None, None)

    image_size = (frame.shape[1], frame.shape[0])

    object_points = []
    for cid in chess_ids:
        object_points.append(board.chessboardCorners[cid[0],:])
    object_points = np.array([object_points])
    image_points = np.reshape(np.array(chess_corners), (1, -1, 2))

    print(object_points.shape)
    return (True, object_points, image_points, chess_ids)

observations = {"left" : [], "right" : []}

def add_observation(camera_name, object_points, image_points, ids):
    global observations
    id_to_image_pt = {}
    for i in range(len(ids)):
        id_to_image_pt[ids[i,0]] = image_points[0,i,:]

    observations[camera_name].append((id_to_image_pt, object_points, image_points, ids))
    print("Total observations for %s: %d" % (camera_name, len(observations[camera_name])))

def add_camera_calibration(K,D):
    cam = OrderedDict()
    cam['size_px'] = [848, 800]
    cam['center_px'] = [K[0,2], K[1,2]]
    cam['focal_length_px'] = [K[0,0], K[1,1]]
    cam['distortion'] = OrderedDict()
    cam['distortion']['k'] = D.flatten().tolist()
    cam['distortion']['type'] = 'kannalabrandt4'
    return cam

def save_calibration(directory, sn, K1, D1, K2, D2):
    # This will be converted to use librealsense API to write to
    # T261 eeprom
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

def evaluate_calibration(camera_name, object_points, image_points, identification, rvec, tvec, K, D, pixel_thresh = None):
    plot_scatter = False
    rms = 0.0
    N = 0
    Nframes = len(object_points)  # number of frames
    inlier_object_points = []
    inlier_image_points = []
    d_max = 0
    for i in range(Nframes):
        proj = cv2.fisheye.projectPoints(object_points[i], rvec[i], tvec[i], K, D)
        proj_err = image_points[i][0] - proj[0][0]
        if plot_scatter:
            plt.scatter(proj_err[:,0], proj_err[:,1], marker='.')

        Npoints = len(image_points[i][0])  # number of points
        inlier_object = []
        inlier_image = []
        for j in range(Npoints):
            #print(j)
            #print(image_points[i][0][j])
            #print(proj[i][0][j])
            proj_err = image_points[i][0][j] - proj[0][0][j]
            #print(proj_err)

            d = np.linalg.norm(image_points[i][0][j]-K[:2,2])
            if d > d_max:
                d_max = d

            if plot_scatter and identification and pixel_thresh:
                if np.linalg.norm(proj_err) > pixel_thresh:
                    plt.text(proj_err[0], proj_err[1], str(i)+","+str(identification[i][j][0]))

            pt_rms = np.array(proj_err).dot(proj_err)
            if pixel_thresh and pt_rms < pixel_thresh:
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
        if pixel_thresh:
            plt.savefig("reproj_err_outlier_" + camera_name + ".png")
        else:
            plt.savefig("reproj_err_" + camera_name + ".png")
        #if visualize:
        #plt.show()
        plt.show(block=False)
        plt.pause(2)
        plt.close()

    # support
    print("max. distance [px]:", d_max)

    # distortion
    if visualize:
        def theta_d(theta, D):
            return theta*( 1+D[0]*np.power(theta,2)+D[1]*np.power(theta,4)+D[2]*np.power(theta,6)+D[3]*np.power(theta,8) )
        theta = np.linspace(0,m.pi/2)
        Dmean = np.array([-0.00626438, 0.0493399, -0.0463255, 0.00896666])
        Dorig = [-0.00217445590533316,      0.0376055203378201,     -0.0364043116569519,  0.00593686196953058]  # TODO: read
        plt.figure()
        plt.xlabel('deg')
        plt.plot(theta, theta_d(theta, D))
        plt.legend(['new','mean','orig'])
        plt.savefig("theta_d.png")
        #plt.show()
        plt.show(block=False)
        plt.pause(3)
        plt.close()

    return (inlier_object_points, inlier_image_points)

def calibrate_observations(camera_name):
    obs = observations[camera_name]
    object_points = []
    image_points = []
    identification = []
    for (id_to_image_pt, obj_i, img_i, ids) in obs:
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


    (inlier_object, inlier_image) = evaluate_calibration(camera_name, object_points, image_points, identification, rvec, tvec, K, Dguess, rmse*3)
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

    evaluate_calibration(camera_name, inlier_object, inlier_image, None, rvec, tvec, K, Dguess)

    return (rmse, K, D)

def calibrate_extrinsics(observations, K1, D1, K2, D2):
    # https://docs.opencv.org/trunk/db/d58/group__calib3d__fisheye.html#gadbb3a6ca6429528ef302c784df47949b
    obs_l = observations["left"]
    obs_r = observations["right"]

    object_points = []
    image_points_l = []
    image_points_r = []
    N = len(obs_l)
    N_stereo = 0
    for i in range(N):
        print("Frame #", i)
        (id_to_image_pt_l, obj_i_l, img_i_l, ids_l) = obs_l[i]
        (id_to_image_pt_r, obj_i_r, img_i_r, ids_r) = obs_r[i]

        obj_i = []
        img_i_lr = []
        img_i_rl = []
        for j in range(len(ids_l)): # points (left)
            if ids_l[j] in ids_r:
                obj_i.append(obj_i_l[0,j,:])
                img_i_lr.append(img_i_l[0,j,:])
                img_i_rl.append(id_to_image_pt_r[ids_l[j,0]])

                if debug:
                    print("ID #", ids_l[j,0], "found in both images")
                    print("p3D:", obj_i_l[0,j,:])
                    print("p2D (img1):", img_i_l[0,j,:])
                    print("p2D (img2):",id_to_image_pt_r[ids_l[j,0]])
                    print("delta:", img_i_l[0,j,:]-id_to_image_pt_r[ids_l[j,0]])

        print("detections:", len(obj_i))

        if len(obj_i) >= n_stereo_matches:
            object_points.append(obj_i[:n_stereo_matches])
            image_points_l.append(img_i_lr[:n_stereo_matches])
            image_points_r.append(img_i_rl[:n_stereo_matches])
            N_stereo = N_stereo + 1

    # https://github.com/opencv/opencv/issues/11085
    object_points = np.asarray(object_points, dtype=np.float64)
    image_points_l = np.asarray(image_points_l, dtype=np.float64)
    image_points_r = np.asarray(image_points_r, dtype=np.float64)

    # shape: (N_frame, 1, N_points, {3,2})
    object_points = object_points.reshape(N_stereo,1,-1,3)
    image_points_l = image_points_l.reshape(N_stereo,1,-1,2)
    image_points_r = image_points_r.reshape(N_stereo,1,-1,2)

    print(object_points.shape)
    print(image_points_l.shape)
    print(image_points_r.shape)

    (rms, K1, D1, K2, D2, R, T) = cv2.fisheye.stereoCalibrate(
        object_points, image_points_l, image_points_r,
        K1, D1,
        K2, D2,
        (800,848),
        None, None,
        cv2.fisheye.CALIB_FIX_INTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-9)
    )

    return (rms, R, T)


parser = argparse.ArgumentParser()
parser.add_argument('--path',   default="cam", help='output path prefix')
parser.add_argument('--record',                help='record <filename> to rosbag')
parser.add_argument('--play',                  help='playback <filename> from rosbag')
parser.add_argument('--extrinsics', default=False, help='calibrate extrinsics', action='store_true')
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

    nobservations = 0
    K1 = D1 = K2= D2 = None
    while True:
        ts = None
        success, frames = pipe.try_wait_for_frames(timeout_ms=1000)
        if not success:
            break
        if frames.is_frameset():
            frameset = frames.as_frameset()
            f1 = frameset.get_fisheye_frame(1).as_video_frame()
            f2 = frameset.get_fisheye_frame(2).as_video_frame()
            left_data = np.asanyarray(f1.get_data())
            right_data = np.asanyarray(f2.get_data())
            ts = frameset.get_timestamp() # valid
            frame_copy = {"left"  : left_data,
                          "right" : right_data}

        cv2.imshow("left", frame_copy["left"])
        key = cv2.waitKey(1)

        if key == ord('c'):
            break

        # Check if the camera has acquired any frames
        if ts is not None and key == ord('s'):
            print("snapshot")
            (ok_l, object_points_l, image_points_l, chess_ids_l) = detect_markers("fe1", frame_copy["left"], K0l, D0l)
            if ok_l:
                print("left ok")
                (ok_r, object_points_r, image_points_r, chess_ids_r) = detect_markers("fe2", frame_copy["right"], K0r, D0r)
                if ok_r:
                    print("right ok")
                    cv2.imwrite(args.path + "/fe1_%03d.png" % nobservations, frame_copy["left"])
                    cv2.imwrite(args.path + "/fe2_%03d.png" % nobservations, frame_copy["right"])
                    add_observation("left", object_points_l, image_points_l, chess_ids_l)
                    add_observation("right", object_points_r, image_points_r, chess_ids_r)
                    nobservations += 1

    print("Finished replay")
    print("Calibrating", nobservations)
    (rms1, K1, D1) = calibrate_observations("left", K0l, D0l)
    print()
    (rms2, K2, D2) = calibrate_observations("right", K0r, D0r)

    save_calibration(args.path, sn, K1, D1, K2, D2)

    f = open(args.path + '/rmse.txt','w')
    np.savetxt(f, np.array([rms1, rms2]).reshape(1,2))
    f.close()

    if args.extrinsics:
        (rms, R, T) = calibrate_extrinsics(observations, K1, D1, K2, D2)
        print("stereo:")
        print("rms:", rms)
        print("R:", R)
        print("T:", T)

        H = np.eye(4)
        H[:3,:3] = R
        H[:3, 3] = T.flatten()
        np.savetxt("H.txt", H, fmt='%.6f')

finally:
    pipe.stop()
