# Calibrating a T261 module after assembly into a product

In order to provide the best tracking, an accurate calibration of T261 is required. When you install the T261 module in your device you will change the extrinsics of this calibration slightly, and when you cover the module with glass you change the distortion of the images, so recalibration is required. In this example we provide a sample flow to calculate the new intrinsics and extrinsics of the stereo camera pair in T261 and write these parameters to the internal device memory on T261. The maker of the product integrating T261 should take this flow as a representative example and adapt it to whatever is most appropriate for the product and production line.

## Overview

This python sample supports the basic functionality of capturing N stereo image pairs, performing an intrinsic calibration for both cameras and writing out the calibration data into a json calibration file. In a future version this will be replaced with calls to librealsense to directly write the calibration to the device.

Steps:
* Capture N stereo image pairs (press key 's'). Images are accepted if minimum number of detections are found in both images.
* After N stereo pairs are captured, calibration can be run (press key 'c').
    * Two calibration steps are performed for each camera, a first initial calibration and, after outlier removal, a second refined calibration.
    * The reprojection error (RMSE) of the refined calibration should be below 0.5 pixel for a good calibration (for the below defined set of poses and calibration target).
    * Also good coverage of the whole field of view is important and for this the below listed poses should be followed as closely as possible to obtain repeatable calibration results within our defined target accuracy. The maximum distance of detections to the camera principal point should be at least 350 pixel to allow estimation of the fisheye lens distortion towards the edges.
* After a successful calibration the calibration data is saved to a json file (in the same folder where the calibration was run).

This calibration requires a ChAruCo target (10x16) printed in US letter size (100% scale) on a rigid surface.

Six static poses are provided, your procedure should aim to replicating them as closely as possible. The camera images in poses.pdf can be used to visually align the camera pose to your device setup.

In addition to a rigid calibration target, sufficient and uniform lighting and a uniform background (that can cover up to around half of the field of view) are important for good and repeatable calibration results.

Please follow the installation instructions at the start of the python script.

## Playback

The tool also supports playback from a folder of images for debugging. Use the argument `--images` to provide the input data
