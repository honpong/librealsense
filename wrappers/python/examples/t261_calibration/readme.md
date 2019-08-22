# Calibrating a T261 module after assembly into a product

In order to provide the best tracking, an accurate calibration of T261 is required. When you install the T261 module in your device you will change the extrinsics of this calibration slightly, and when you cover the module with glass you change the distortion of the images, so recalibration is required. In this example we provide a sample flow to calculate the new intrinsics and extrinsics of the stereo camera pair in T261 and write these parameters to the internal device memory on T261. The maker of the product integrating T261 should take this flow as a representative example and adapt it to whatever is most appropriate for the product and production line.

## Overview

This python sample supports the basic functionality of capturing N stereo image pairs, performing an intrinsic calibration for both cameras and writing out the calibration data into a json calibration file. In a future version this will be replaced with calls to librealsense to directly write the calibration to the device.

## Setup

### Hardware
* Calibration target: The calibration process requires a ChAruCo target (10x16) printed on a rigid surface of approximately the size of a US letter or A4 at 100% scale. (Please check printer settings which might be different from default!)

* Lighting: Sufficient and uniform lighting and a uniform background (that can cover up to around half of the field of view) are important for good and repeatable calibration results.

* View points: Six static poses are provided, your procedure should aim to replicating them as closely as possible. It is recommended to create a mechanical fixture or use a robot to execute the poses as closely as possible and achieve repeatable calibration results. The camera images in poses.pdf can be used for a visual check of the camera poses for your device setup.


### Software
Please follow the installation instructions at the start of the python script.

## Running the T261 calibration sample

Steps:
0. Make sure that the camera lens is clean to get the best images.
1. Capture N stereo image pairs (press key 's'). Images are accepted if minimum number of detections are found in both images.
2. After N stereo pairs are captured, calibration can be run (press key 'c').
    * Two calibration steps are performed for each camera, a first initial calibration and, after outlier removal, a second refined calibration.
    * The reprojection error (RMSE) of the refined calibration should be below 0.5 pixel for a good calibration (for the below defined set of poses and calibration target).
    * Also good coverage of the whole field of view is important and for this the below listed poses should be followed as closely as possible to obtain repeatable calibration results within our defined target accuracy. The maximum distance of detections to the camera principal point should be at least 350 pixel to allow estimation of the fisheye lens distortion towards the edges.
3. After a successful calibration the calibration data is saved to a json file (in the same folder where the calibration was run).

### Playback

The tool also supports playback from a folder of images for debugging. Use the argument `--images` to provide the input data

## Positions
### Overview
A list of camera poses can be found in [poses_left.txt](poses_left.txt) and [poses_right.txt](poses_right.txt).
The camera poses are expressed with respect to the calibration target frame (as depicted below).

<img class="image" src="doc/img/poses_xz_centered.png" width="45%"> <img class="image" src="doc/img/poses_yz_centered.png" width="45%">

<img class="image" src="doc/img/chaxis.png" width="50%">


### Position 1
![pose1](doc/img/pose1.png)

<img class="image" src="data/fe1_000.png" width="45%"> <img class="image" src="data/fe2_000.png" width="45%">

### Position 2
![pose2](doc/img/pose2.png)

<img class="image" src="data/fe1_001.png" width="45%"> <img class="image" src="data/fe2_001.png" width="45%">

### Position 3
![pose3](doc/img/pose3.png)

<img class="image" src="data/fe1_002.png" width="45%"> <img class="image" src="data/fe2_002.png" width="45%">

### Position 4
![pose4](doc/img/pose4.png)

<img class="image" src="data/fe1_003.png" width="45%"> <img class="image" src="data/fe2_003.png" width="45%">

### Position 5
![pose5](doc/img/pose5.png)

<img class="image" src="data/fe1_004.png" width="45%"> <img class="image" src="data/fe2_004.png" width="45%">

### Position 6
![pose6](doc/img/pose6.png)

<img class="image" src="data/fe1_005.png" width="45%"> <img class="image" src="data/fe2_005.png" width="45%">
