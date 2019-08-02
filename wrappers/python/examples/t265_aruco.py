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


#TODO: make cmd. line arg.
visualize = False
validate = False  # use factory calibration as GT

# CALIB_FIX_SKEW - Fixes the skew (K[0][1]) to 0
# CALIB_USE_INTRINSIC_GUESS - uses the K and D input as a guess
# CALIB_RECOMPUTE_EXTRINSICS - Recomputing the pose of the target every iteration, if we don't do this it only computes it once (at the start)
flags = cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-6)

board_width = 16
board_height = 10
checker_size_m = 0.015
april_size_m = 0.0075
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)  # 16*10/2 = 80
board = cv2.aruco.CharucoBoard_create(board_width, board_height, checker_size_m, april_size_m, dictionary)
parameters = cv2.aruco.DetectorParameters_create()
#parameters.adaptiveThreshWinSizeStep = 2
#parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX


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



def detect_markers(frame, K, D):
    (markers, ids, rejected) = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    frame_copy = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy, markers, ids)
    cv2.imshow("detected markers", frame_copy)
    cv2.waitKey(1)
    #cv2.imwrite("1_detect_markers.png", frame_copy)

    (markers, ids, rejected, _) = cv2.aruco.refineDetectedMarkers(frame, board, markers, ids, rejected, errorCorrectionRate=1, parameters=parameters)

    frame_copy2 = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy2, markers, ids)
    cv2.imshow("detected markers (refined)", frame_copy2)
    cv2.waitKey(1)
    #cv2.imwrite("2_detect_markers_refined.png", frame_copy2)

    ok = ids is not None and len(ids) > 15
    if ok:
        (num_refined, chess_corners, chess_ids) = cv2.aruco.interpolateCornersCharuco(markers, ids, frame, board, minMarkers=2) #, cameraMatrix=K) #, distCoeffs=D)

        frame_copy3 = frame.copy()
        cv2.aruco.drawDetectedCornersCharuco(frame_copy3, chess_corners, chess_ids)
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
    return (True, object_points, image_points, chess_ids)

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

    rms = 0.0
    N = 0
    Nframes = len(object_points)  # number of frames
    inlier_object_points = []
    inlier_image_points = []
    for i in range(Nframes):
        proj = cv2.fisheye.projectPoints(object_points[i], rvec[i], tvec[i], K, D)
        proj_err = image_points[i][0] - proj[0][0]
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

    if visualize:
        plt.figure()
        plt.xlabel('deg')

        plt.plot(theta, theta_d(theta, D)/theta-1)
        #plt.plot(theta, theta_d(theta, Dmean)/theta-1, '--')
        #plt.plot(theta, theta_d(theta, Dorig)/theta-1)
        #plt.plot(theta, theta_d(theta, D)/theta - theta_d(theta, Dmean)/theta)
        #plt.plot(theta, theta_d(theta, D)/theta - theta_d(theta, Dorig)/theta)

        plt.plot([0,m.pi/2],[0,0],'k--')
        plt.plot([0,m.pi/2],[0.01,0.01],'b--')
        plt.plot([0,m.pi/2],[-0.01,-0.01],'b--')

        plt.legend(['new','mean','orig', 'delta_mean', 'delta_orig', '1%'])

        plt.savefig("distortion.png")
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
    Dguess = np.zeros((4,1))
    image_size = (848, 800)
    Kguess = np.zeros((3,3))
    Kguess[0][0] = Kguess[1][1] = 287
    Kguess[0][2] = image_size[0]/2
    Kguess[1][2] = image_size[1]/2

    initial_flags = flags | cv2.fisheye.CALIB_FIX_K3 | cv2.fisheye.CALIB_FIX_K4
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


    (inlier_object, inlier_image) = evaluate_calibration(object_points, image_points, identification, rvec, tvec, K, D, Korig, Dorig, rmse*2) # 2 pixel
    # object points is a python list of M items which are each (1, N, 3) np.arrays
    # image points is a python list of M items which are each (1, N, 2) np.arrays
    print(len(inlier_image), "images remaining")
    final_flags = flags
    (rmse, K, D, rvec, tvec) = cv2.fisheye.calibrate(objectPoints = inlier_object,
                                                                                    imagePoints = inlier_image,
                                                                                    image_size = image_size,
                                                                                    K = Kguess,
                                                                                    D = D,
                                                                                    flags = final_flags,
                                                                                    criteria = criteria)
    print("refined rmse", rmse)
    print("camera", K)
    print("distortion_coeffs", np.array2string(D, separator=', '))

    return (rmse, K, D)


frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None
              }


parser = argparse.ArgumentParser()
parser.add_argument('--path',   default="cam", help='image path')
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
        cfg.enable_device_from_file(args.play)  # playback from rosbag
    pipe.start(cfg)

    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()

    dev = profiles.get_device()
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
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)

    # Print information about both cameras
    #print("Left camera:",  intrinsics["left"])
    #print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K0l  = camera_matrix(intrinsics["left"])
    D0l  = fisheye_distortion(intrinsics["left"])
    K0r = camera_matrix(intrinsics["right"])
    D0r = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get thre relative extrinsics between the left and right camera
    if args.play is None: # can't get this from playback
        (R, T) = get_extrinsics(streams["right"], streams["left"])

    mode = "stack"

    #K1 = D1 = K2= D2 = None
    n_img1 = 0
    n_img2 = 0
    z = 0
    left_computed = False
    right_computed = False
    K1 = D1 = K2= D2 = None
    while True:
        frames = pipe.wait_for_frames()  # blocking
        #if frame.is_frameset():
        frameset = frames.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        frame_copy = {"left"  : left_data,
                      "right" : right_data}
        z+=1
        print(z)
        if z % 20 != 0:  # subsample
            continue

        # Check if the camera has acquired any frames
        valid = ts is not None

        # If frames are ready to process
        if valid:
            # undistort
            #frame = frame_copy["left"]
            #cv2.imwrite("0_distorted.png", frame)
            #frame_ud = cv2.fisheye.undistortImage(frame, K1, D1, None, K_left) # keep same K
            #cv2.imshow("undistorted", frame_ud)
            #cv2.waitKey(1)
            #cv2.imwrite("0_undistorted.png", frame_ud)

            #K1 = D1 = K2= D2 = None
            # left
            (ok, object_points, image_points, chess_ids) = detect_markers(frame_copy["left"], K0l, D0l)
            #(ok, object_points, image_points, chess_ids) = detect_markers(frame_ud, K_left, D_left)  # undistort first
            if ok:
                good = True
                """
                for i in range(len(chess_ids)):
                    if min_dist_for_id("left", chess_ids[i,0], image_points[0,i,:]) < 10:
                        good = False
                        break
                """
                if good:
                    add_observation("left", object_points, image_points, chess_ids)
                    print("good left image")

                    cv2.imwrite(args.path+"/fe1_ "+str(n_img1)+".png", frame_copy["left"])
                    n_img1 = n_img1 +1

                    if not left_computed and len(observations["left"]) > 20:
                        (rms1, K1, D1) = calibrate_observations("left", K0l, D0l)
                        left_computed = True

            # right
            (ok, object_points, image_points, chess_ids) = detect_markers(frame_copy["right"], K0r, D0r)
            if ok:
                good = True
                """
                for i in range(len(chess_ids)):
                    if min_dist_for_id("right", chess_ids[i,0], image_points[0,i,:]) < 10:
                        good = False
                        break
                """
                if good:
                    add_observation("right", object_points, image_points, chess_ids)
                    print("good right image")

                    cv2.imwrite(args.path+"/fe2_ "+str(n_img2)+".png", frame_copy["right"])
                    n_img2 = n_img2 +1

                    if not right_computed and len(observations["right"]) > 20:
                        (rms2, K2, D2) = calibrate_observations("right", K0r, D0r)
                        right_computed = True

            if K1 is not None and K2 is not None:
                #print(K1, D1, K2, D2)
                save_calibration('cals', sn, K1, D1, K2, D2)

                f = open(args.path + '/rmse.txt','a')
                np.savetxt(f, np.array([rms1, rms2]).reshape(1,2))
                f.close()
                sys.exit(0)

            #color_image0 = cv2.cvtColor(frame_copy["left"], cv2.COLOR_GRAY2RGB)
            #cv2.imshow(WINDOW_TITLE,color_image0)

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
