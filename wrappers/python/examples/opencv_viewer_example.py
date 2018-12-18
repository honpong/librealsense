## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 640, 360, rs.format.y8, 30)

# Start streaming
pipeline.start(config)
zo = rs.zero_order_fix()
#align = rs.align()
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames(15000)
	#zo.set_option(rs.option.filter_zo_ir_threshold, 0)
        frames = zo.process(frames).as_frameset()
        #frames = align.process(frames).as_frameset()
        depth_frame = frames.get_depth_frame()
        #color_frame = frames.get_color_frame()
        if not depth_frame:
            continue

        ## Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        #color_image = np.asanyarray(color_frame.get_data())

        ## Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        ## Stack both images horizontally
        #images = np.hstack((depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', depth_colormap)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
