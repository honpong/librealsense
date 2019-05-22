#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function
import sys

# Import OpenCV and numpy
import cv2
import numpy as np

left = cv2.imread("left.pgm")
left = cv2.cvtColor(left, cv2.COLOR_RGB2GRAY)
print(left.shape)

right = cv2.imread("right.pgm")
right = cv2.cvtColor(right, cv2.COLOR_RGB2GRAY)

board_width = 16
board_height = 10
checker_size_m = 0.015
april_size_m = 0.0075
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
board = cv2.aruco.CharucoBoard_create(board_width, board_height, checker_size_m, april_size_m, dictionary)

checker_width = board_width - 1
checker_height = board_height - 1
# 0,0 is bottom left
# checker_width,checker_height is upper right
(ygrid, xgrid) = np.mgrid[0:checker_height,0:checker_width]
ygrid = (checker_height - 1) - ygrid
x_obj_points = xgrid * checker_size_m
y_obj_points = ygrid * checker_size_m
obj_ind = ygrid * checker_width + xgrid
obj_dict = {}
for r in range(obj_ind.shape[0]):
    for c in range(obj_ind.shape[1]):
        obj_dict[obj_ind[r,c]] = np.array([x_obj_points[r,c], y_obj_points[r,c], 0], dtype=np.float32)

# print(board.ids) # array of ids 0-80 for my dictionary
# print(board.objPoints) # array of object points x,y,0 for my dictionary
# board.chessboardCorners
print(board.chessboardCorners)

parameters = cv2.aruco.DetectorParameters_create()
#parameters.adaptiveThreshWinSizeStep = 2
#parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

def detect_markers(frame, title="Detections"):
    (markers, ids, rejected) = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    (markers, ids, rejected, _) = cv2.aruco.refineDetectedMarkers(frame, board, markers, ids, rejected, errorCorrectionRate=1, parameters=parameters)
    ok = ids is not None and len(ids) > 10
    if ok:
        (num_refined, chess_corners, chess_ids) = cv2.aruco.interpolateCornersCharuco(markers, ids, frame, board, minMarkers=1)
        ok = num_refined > 10
    #print("num_refined", num_refined)
    #print("chess_ids", chess_ids)
    #print("chess_corners", chess_corners)

    #print(res2[0]) # quad coord, not subpixel
    #print(res2[1]) # ids, including things not on board
    #print(res2[2]) # quad coord, not subpixel - bad detections
    # calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs[, rvecs[, tvecs[, flags[, criteria]]]]) -> retval, cameraMatrix, distCoeffs, rvecs, tvecs
    cameraMatrix = np.zeros(3)
    distCoeffs = np.zeros((3,1))
    #calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs[, rvecs[, tvecs[, flags[, criteria]]]]) -> retval, cameraMatrix, distCoeffs, rvecs, tvecs
    if ok:
        image_size = (frame.shape[1], frame.shape[0])
        print("ncorners", chess_corners.shape)
        print("ids", chess_ids.shape)
        (rms_error, camera_matrix, distortion_coeffs, rvec, tvec) = cv2.aruco.calibrateCameraCharuco(np.array([chess_corners]), np.array([chess_ids]), board, image_size, None, None)
        print(rms_error)
        print(camera_matrix)
        print(distortion_coeffs)
        object_points = []
        for cid in chess_ids:
            object_points.append(obj_dict[cid[0]])
            #object_points.append(board.chessboardCorners[cid[0],:])
            print(obj_dict[cid[0]], board.chessboardCorners[cid[0],:])
        object_points = np.array([object_points])
        image_points = np.reshape(np.array(chess_corners), (1, -1, 2))
        (rms_error, camera_matrix, distortion_coeffs, rvec, tvec) = cv2.calibrateCamera(objectPoints = object_points,
                                                                                        imagePoints = image_points,
                                                                                        imageSize = image_size,
                                                                                        cameraMatrix = None,
                                                                                        distCoeffs = None)
        print(rms_error)
        print(camera_matrix)
        print(distortion_coeffs)
        newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, image_size, 1, image_size)
        print("roi", roi)
        print("newcam", newCameraMtx)

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
        print("fisheye", rms_error)
        print(camera_matrix)
        print(distortion_coeffs)
        """ 
        undistortedImg = cv2.undistort(frame, camera_matrix, distortion_coeffs, None, newCameraMtx)
        cv2.namedWindow("undist", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("undist", undistortedImg)
        cv2.waitKey(0)
        cv2.destroyWindow("undist")
        """



    patternSize = (checker_width, checker_height)
    pattern_found, corners = cv2.findChessboardCorners(frame, patternSize)
    print("pattern_found", pattern_found)
    print("corners", corners)
    cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    color_image0 = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
    if ok:
        cv2.aruco.drawDetectedCornersCharuco(color_image0, chess_corners, chess_ids)
    if pattern_found:
        color_image0 = cv2.drawChessboardCorners(color_image0, patternSize, corners, False)
    #cv2.aruco.drawDetectedMarkers(color_image0,res[0],res[1])
    #cv2.aruco.drawDetectedMarkers(color_image0,res3[0],res3[1])
    cv2.imshow(title, color_image0)
    cv2.waitKey(0)
    cv2.destroyWindow(title)

detect_markers(left, "Left")
detect_markers(right, "Right")
