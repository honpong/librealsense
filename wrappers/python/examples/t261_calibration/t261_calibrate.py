#!/usr/bin/env python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
from __future__ import print_function  # Python 2/3 compatibility

"""
First, set up the virtual enviroment:

$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install opencv-contrib-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings
$ pip install matplotlib        # install matplotlib in the venv
$ pip install transformations   # install transformations in the venv

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
import errno
from collections import OrderedDict
import json
import argparse
import sys
import csv
import transformations as tf


# Enable this for more verbose printing
debug = False

"""
This example uses a ChArUco target and the parameters of it are
defined below. You can generate your own target using a pattern
generator, for example:

https://calib.io/pages/camera-calibration-pattern-generator

and you can purchase ready-made targets as well:

https://calib.io/collections/checkerboards/products/charuco-targets

If you decide to use your own target, you should adjust
detect_markers below to detect your pattern and return the image and
corresponding object points and make sure that the relative position
of the target and the T261 is such that you cover the same regions of
the lenses.
"""
board_width = 16
board_height = 10
checker_size_m = 0.015
april_size_m = 0.0075

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
board = cv2.aruco.CharucoBoard_create(board_width, board_height, checker_size_m, april_size_m, dictionary)
parameters = cv2.aruco.DetectorParameters_create()

# detection parameters
min_detections = 5
n_stereo_matches = 15

# OpenCV fisheye calibration parameters
flags = cv2.fisheye.CALIB_FIX_SKEW                      # Fix the skew of K to 0
flags = flags | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS   # Use a guess for the camera matrix
flags = flags | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC   # Recompute the relative poses of the target each time
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-9)


# global variable to store detections
observations = {"left" : [], "right" : []}
image_size = None

# Check that a path exists and create it if needed
def ensure_path(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

# Returns a camera matrix K from librealsense intrinsics
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

# Returns the kb4 distortion coefficients
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# Show the AruCo marker detections
def visualize_markers(frame, markers, ids):
    frame_copy = frame.copy()
    cv2.aruco.drawDetectedMarkers(frame_copy, markers, ids)
    return frame_copy

# Show the interpolated chessboard corners derived from the AruCo detections
def visualize_chess_corners(frame, chess_corners, chess_ids):
    frame_copy = frame.copy()
    cv2.aruco.drawDetectedCornersCharuco(frame_copy, chess_corners, chess_ids)
    return frame_copy

"""
Detect the ChAruCo target in a frame and return object and image points for it.

ChAruCo targets essentially composed of a chessboard overlayed with Apriltag fiducial markers in the white squares.

Detection proceeds in four steps:
    - Detect the initial set of Apriltags in the image
    - Reject or correct the Apriltags based on a known board layout
    - Find a subpixel location for the chessboard corner nearest to each Apriltag
    - Return the chessboard corners, ids, and real world points (aka object points) for this detection
"""
def detect_markers(camera_name, frame):
    global n_frame, image_size

    (markers, ids, rejected) = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    if debug:
        print("Number of detections:", len(markers))
    detections = visualize_markers(frame, markers, ids)
    cv2.imwrite(tmp_folder + camera_name + "_%03d_1_detect_markers.png" % n_frame, detections)

    (markers_refined, ids_refined, _, _) = cv2.aruco.refineDetectedMarkers(frame, board, markers, ids, rejected, errorCorrectionRate=0, parameters=parameters)
    if debug:
        print("Number of refined detections:", len(markers_refined))
    detections = visualize_markers(frame, markers_refined, ids_refined)
    cv2.imwrite(tmp_folder + camera_name + "_%03d_2_detect_markers_refined.png" % n_frame, detections)

    if ids is None or len(ids) < min_detections:
        return (False, None, None, None)

    (n_chess, chess_corners, chess_ids) = cv2.aruco.interpolateCornersCharuco(markers, ids, frame, board, minMarkers=2)
    if debug:
        print("Number of chess corners:", len(chess_corners))
    detections = visualize_chess_corners(frame, chess_corners, chess_ids)
    cv2.imwrite(tmp_folder + camera_name + "_%03d_3_detect_corners_interpol.png" % n_frame, detections)

    if n_chess < min_detections:
        return (False, None, None, None)

    image_size = (frame.shape[1], frame.shape[0])

    # aggregate detections
    object_points = []
    for cid in chess_ids:
        object_points.append(board.chessboardCorners[cid[0],:])
    object_points = np.array([object_points])
    image_points = np.reshape(np.array(chess_corners), (1, -1, 2))
    if debug:
        print(object_points.shape)

    return (True, object_points, image_points, chess_ids)

"""
Adds an observation of the target for camera_name
"""
def add_observation(camera_name, object_points, image_points, ids):
    global observations
    id_to_image_pt = {}
    for i in range(len(ids)):
        id_to_image_pt[ids[i,0]] = image_points[0,i,:]
    observations[camera_name].append((id_to_image_pt, object_points, image_points, ids))
    print("Total frames for %s camera: %d" % (camera_name, len(observations[camera_name])))

"""
The next three functions define a temporary interface for writing calibrations to file for inspection purposes. This will be replaced with direct calls to T261 librealsense APIs for setting the calibration in a future version. Once that is ready, it will look something like:

    fe1 = profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile()
    fe2 = profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()
    fe1.set_intrinsics(fe1_intrinsics)
    fe2.set_intrinsics(fe2_intrinsics)
    fe1.set_extrinsics_to(fe2, extrinsics)
"""
def add_camera_calibration(K, D):
    cam = OrderedDict()
    cam['size_px'] = [848, 800]  # hard-coded
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
    calib['calibration_version'] = 10  # do not change
    calib['cameras'] = []
    calib['cameras'].append( add_camera_calibration(K1,D1) )
    calib['cameras'].append( add_camera_calibration(K2,D2) )

    if not os.path.exists(directory):
        os.mkdir(directory)
    with open(directory + '/cam_' + str(sn) + '_intrinsics.json', 'w') as f:
        json.dump(calib, f, indent=4)

def save_extrinsics(filename, R, T):
    H = np.eye(4)
    H[:3,:3] = R
    H[:3, 3] = T.flatten()
    np.savetxt(filename, H, fmt='%.6f')

"""
Check the result of a calibration, plotting the distribution of the reprojection error and identifying likely outlier points
"""
def evaluate_calibration(camera_name, object_points, image_points, identification, rvec, tvec, K, D, pixel_thresh = None):
    N_frames = len(object_points)  # number of frames
    inlier_object_points = []
    inlier_image_points = []
    d_max = 0  # measure image support region
    plt.figure()
    for i in range(N_frames):
        proj = cv2.fisheye.projectPoints(object_points[i], rvec[i], tvec[i], K, D)
        proj_err = image_points[i][0] - proj[0][0]

        # plot reprojection error (per detection)
        plt.scatter(proj_err[:,0], proj_err[:,1], marker='.')

        N_points = len(image_points[i][0])  # number of points
        inlier_object = []
        inlier_image = []
        for j in range(N_points):
            proj_err = image_points[i][0][j] - proj[0][0][j]

            # identify outlier
            if identification and pixel_thresh and np.linalg.norm(proj_err) > pixel_thresh:
                plt.text(proj_err[0], proj_err[1], str(i)+","+str(identification[i][j][0]))

            # outlier removal
            pt_rms = np.array(proj_err).dot(proj_err)
            if pixel_thresh and pt_rms < pixel_thresh:
                inlier_object.append(object_points[i][0][j])
                inlier_image.append(image_points[i][0][j])

            # image support measure
            d = np.linalg.norm(image_points[i][0][j]-K[:2,2])
            if d > d_max:
                d_max = d

        if len(inlier_object) > 5:  # hard-coded
            inlier_object = np.reshape(np.array(inlier_object), (1, -1, 3))
            inlier_image  = np.reshape(np.array(inlier_image), (1, -1, 2))
            inlier_object_points.append(inlier_object)
            inlier_image_points.append(inlier_image)

    if pixel_thresh:
        plt.savefig(tmp_folder + "reproj_err_outlier_" + camera_name + ".png")
    else:
        plt.savefig(tmp_folder + "reproj_err_" + camera_name + ".png")

    # plot distortion
    def theta_d(theta, D):
        return theta*( 1+D[0]*np.power(theta,2)+D[1]*np.power(theta,4)+D[2]*np.power(theta,6)+D[3]*np.power(theta,8) )
    theta = np.linspace(0, m.pi/2)
    plt.figure()
    plt.plot(theta, theta_d(theta, D))
    plt.savefig(tmp_folder + "theta_d_" + camera_name + ".png")

    return (inlier_object_points, inlier_image_points, d_max)

def save_poses(filename, rvec, tvec):
    with open(filename, "w") as f:
        o = csv.writer(f)
        o.writerow(["frame number", " homogeneous transformation 3x4 (row-major) camera w.r.t. target"])
        for i in range(len(tvec)):
            t = tvec[i].flatten()
            r = rvec[i].flatten()
            # invert => w.r.t. target
            theta = np.linalg.norm(r)
            u = r/theta
            H = tf.rotation_matrix(theta, u)
            H[:3,3] = t
            H = np.linalg.inv(H)  # invert => w.r.t. target
            o.writerow([i] + [" {:6.3f}".format(x) for x in H[:3,:].flatten().tolist()])

"""
Take a set of observations of the targets and use OpenCVs fisheye calibration module to calibrate the intrinsics of the camera. Do this by first estimating a rough calibration, then using this rough identification to reject poor detections, and then compute a refined calibration.
"""
def calibrate_observations(camera_name, K0, D0):
    global image_size
    obs = observations[camera_name]
    object_points = []
    image_points = []
    identification = []
    for (id_to_image_pt, obj_i, img_i, ids) in obs:
        object_points.append(obj_i)
        image_points.append(img_i)
        identification.append(ids)
    # mean distortion
    Dguess = np.array([-0.00626438, 0.0493399, -0.0463255, 0.00896666])
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
    if debug:
        print("rmse", rmse)
        print("K:", K)
        print("D:", np.array2string(D, separator=', '))


    (inlier_object, inlier_image, _) = evaluate_calibration(camera_name, object_points, image_points, identification, rvec, tvec, K, Dguess, rmse*3)
    # object points is a python list of M items which are each (1, N, 3) np.arrays
    # image points is a python list of M items which are each (1, N, 2) np.arrays
    if debug:
        print(len(inlier_image), "images remaining after outlier removal")
    final_flags = flags
    (rmse, K, D, rvec, tvec) = cv2.fisheye.calibrate(objectPoints = inlier_object,
                                                     imagePoints = inlier_image,
                                                     image_size = image_size,
                                                     K = K,
                                                     D = D,
                                                     flags = final_flags,
                                                     criteria = criteria)
    if debug:
        print("rmse", rmse)
        print("K:", K)
        print("D:", np.array2string(D, separator=', '))

    #save_poses(tmp_folder + 'poses_' + camera_name + ".txt", rvec, tvec)

    (_, _, support) = evaluate_calibration(camera_name, inlier_object, inlier_image, None, rvec, tvec, K, Dguess)

    return (rmse, K, D, support)

"""
Compute the extrinsics between the two cameras. This assumes the intrinsics for both cameras are already correct
"""
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
        if debug:
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

    if debug:
        print(object_points.shape)
        print(image_points_l.shape)
        print(image_points_r.shape)

    (rms, K1, D1, K2, D2, R, T) = cv2.fisheye.stereoCalibrate(
        object_points, image_points_l, image_points_r,
        K1, D1,
        K2, D2,
        (848, 800),
        None, None,
        cv2.fisheye.CALIB_FIX_INTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5000, 1e-9)  # TODO: determine flags and criteria
    )

    return (rms, R, T)

def lrs_intrinsics(K, D):
    fe_intrinsics = rs.intrinsics()  # width: 0, height: 0, ppx: 0, ppy: 0, fx: 0, fy: 0, model: None, coeffs: [0, 0, 0, 0, 0]
    fe_intrinsics.width = 848
    fe_intrinsics.height = 800
    fe_intrinsics.fx = K[0,0]
    fe_intrinsics.fy = K[1,1]
    fe_intrinsics.ppx = K[0,2]
    fe_intrinsics.ppy = K[1,2]
    fe_intrinsics.model = rs.distortion.kannala_brandt4  # kb4 
    fe_intrinsics.coeffs = D.tolist() + [0]
    return fe_intrinsics

def reset_calibration():
    ctx = rs.context()
    devs = ctx.query_devices()
    for dev in devs:
        tm2 = dev.as_tm2()
        if tm2:
            tm2.reset_to_factory_calibration()

def read_calibration():
    ctx = rs.context()
    devs = ctx.query_devices()
    for dev in devs:
        tm2 = dev.as_tm2()
        if tm2:
            sensors = tm2.query_sensors()
            for sensor in sensors:
                profiles = sensor.get_stream_profiles()
                for profile in profiles:
                    if profile.is_video_stream_profile():
                        vp = profile.as_video_stream_profile()
                        print(vp.get_intrinsics())
                    elif profile.is_motion_stream_profile():
                        vp = profile.as_motion_stream_profile()
                        print(vp.get_motion_intrinsics())


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default=".", help='calibration output path')
    parser.add_argument('--images', help='image folder input path')
    parser.add_argument('--extrinsics', default=False, help='calibrate extrinsics', action='store_true')
    parser.add_argument('--confirm', default=False, help='write calibration to device (w/o prompt)', action='store_true')
    parser.add_argument('--reset', default=False, help='reset calibration to factory default', action='store_true')
    parser.add_argument('--read', default=False, help='reset calibration to factory default', action='store_true')
    args = parser.parse_args()
    tmp_folder = "tmp"

    if args.reset:
        reset_calibration()
        sys.exit()

    if args.read:
        read_calibration()
        sys.exit()

    try:

        dev = None
        if args.images:
            sn = "playback"
        #else:
            pipe = rs.pipeline()
            profile = pipe.start()
            dev = profile.get_device()
            sn = dev.get_info(rs.camera_info.serial_number)

        print("Serial number:", sn)
        tmp_folder = tmp_folder + "/cam_" + sn + "/"
        ensure_path(tmp_folder)

        if not args.images:
            streams = (
                profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
                profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()
            )
            intrinsics = (streams[0].get_intrinsics(), streams[1].get_intrinsics())
            print("Left camera:",  intrinsics[0])
            print("Right camera:", intrinsics[1])

            # Internal calibration (initial guess, converted to OpenCV format)
            K0l  = camera_matrix(intrinsics[0])
            D0l  = fisheye_distortion(intrinsics[0])
            K0r = camera_matrix(intrinsics[1])
            D0r = fisheye_distortion(intrinsics[1])
            (width, height) = (intrinsics[0].width, intrinsics[0].height)
        else:
            K0l = K0r = np.eye(3)
            D0l = D0r = np.array((4,1))

        n_frame = 0
        frame_index = 0
        while True:
            if args.images:
                left = cv2.imread("%s/fe1_%03d.png" % (args.images, frame_index))
                right = cv2.imread("%s/fe2_%03d.png" % (args.images, frame_index))
                frame_index += 1
                if left is None or right is None:
                    break
                stereo_pair = (left, right)
            else:
                success, frames = pipe.try_wait_for_frames(timeout_ms=1000)
                if not success or not frames.is_frameset():
                    break
                frameset = frames.as_frameset()
                f1 = frameset.get_fisheye_frame(1).as_video_frame()
                f2 = frameset.get_fisheye_frame(2).as_video_frame()
                stereo_pair = (np.asanyarray(f1.get_data()), np.asanyarray(f2.get_data()))

            # display (for visual alignment)
            stereo_horizontal = np.hstack((stereo_pair[0], stereo_pair[1]))
            cv2.namedWindow('Stereo fisheye', cv2.WINDOW_NORMAL)
            cv2.imshow("Stereo fisheye", stereo_horizontal)
            key = cv2.waitKey(1)

            if key == ord('s') or args.images:
                print("Acquire images")
                print("Running detection...")
                (ok1, object_points1, image_points1, chess_ids1) = detect_markers("fe1", stereo_pair[0])
                (ok2, object_points2, image_points2, chess_ids2) = detect_markers("fe2", stereo_pair[1])
                if ok1 and ok2:
                    print("Images accepted")
                    # save images
                    cv2.imwrite(tmp_folder + "fe1_%03d.png" % n_frame, stereo_pair[0])
                    cv2.imwrite(tmp_folder + "fe2_%03d.png" % n_frame, stereo_pair[1])
                    # add observations
                    add_observation("left", object_points1, image_points1, chess_ids1)
                    add_observation("right", object_points2, image_points2, chess_ids2)
                    n_frame += 1

            elif key == ord('c'):
                break
            elif key == ord('q'):
                sys.exit(0)

        print("\nCalibrating over", n_frame, "frames")
        (rms1, K1, D1, support1) = calibrate_observations("left", K0l, D0l)
        (rms2, K2, D2, support2) = calibrate_observations("right", K0r, D0r)

        print("K_left:", K1)
        print("D_left:", np.array2string(D1, separator=', '))
        print("K_right:", K2)
        print("D_right:", np.array2string(D2, separator=', '))

        print("RMSE left [px]:", rms1)
        print("RMSE right [px]:", rms2)

        print("Left image support region [px]:", support1)
        print("Right image support region [px]:", support2)

        # TODO: pass criteria

        save_calibration(args.path, sn, K1, D1, K2, D2)

        f = open(tmp_folder + 'rmse.txt','w')
        #np.savetxt(f, np.array([rms1, rms2]).reshape(1,2))
        f.close()

        if args.extrinsics:
            print("\nStereo calibration")
            (rms, R_fe2_fe1, T_fe2_fe1) = calibrate_extrinsics(observations, K1, D1, K2, D2)  # cam1 w.r.t. cam2
            print("Left fisheye w.r.t. right")
            print("R_fe2_fe1:", R_fe2_fe1)
            print("T_fe2_fe1:", T_fe2_fe1)
            #save_extrinsics(args.path + "/H_fe2_fe1.txt", R_fe2_fe1, T_fe2_fe1)

        # write to device
        print()
        while key not in ['y', 'n'] and not args.confirm:
            key = input("Write to device? [y/n]: ")
        if key == 'n' and not args.confirm:
            sys.exit()
        else:
            print("Writing to device...")
            tm2 = None
            if not dev:
                ctx = rs.context()
                devs = ctx.query_devices()
                for dev in devs:
                    tm2 = dev.as_tm2()
            else:
                tm2 = dev.as_tm2()

            if tm2:
                fe_intrinsics = lrs_intrinsics(K1, D1)
                tm2.set_intrinsics(1, fe_intrinsics)

                fe_intrinsics = lrs_intrinsics(K2, D2)
                tm2.set_intrinsics(2, fe_intrinsics)

                tm2.write_calibration()
                print("Finished")


    finally:
        if not args.images:
            pipe.stop()
