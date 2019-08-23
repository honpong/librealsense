#!/usr/bin/env python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

####################################################################
##           librealsense T261 calibration r/w sample             ##
####################################################################

import pyrealsense2 as rs
import argparse
import sys

def read_calibration():
    global devs
    global tm2

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

def write_calibration():
    global devs
    global tm2

    fisheye_intrinsics = rs.intrinsics()
    motion_intrinsics = rs.motion_device_intrinsic()

    for dev in devs:
        tm2 = dev.as_tm2()
        if tm2:
            tm2.set_intrinsics(1, fisheye_intrinsics)
            tm2.set_intrinsics(2, fisheye_intrinsics)

            tm2.set_motion_device_intrinsics(rs.stream.accel, motion_intrinsics)
            tm2.set_motion_device_intrinsics(rs.stream.gyro, motion_intrinsics)

            tm2.write_calibration()

def reset_calibration():
    global devs
    global tm2

    for dev in devs:
        tm2 = dev.as_tm2()
        if tm2:
            tm2.reset_to_factory_calibration()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--read', default=False, help='read only', action='store_true')
    parser.add_argument('--write', default=False, help='read only', action='store_true')
    parser.add_argument('--reset', default=False, help='read only', action='store_true')
    args = parser.parse_args()
    
    ctx = rs.context()
    devs = ctx.query_devices()
    if args.read:
        read_calibration()
        sys.exit()
    if args.write:
        write_calibration()
        sys.exit()
    if args.reset:
        reset_calibration()
        sys.exit()

    read_calibration()
    write_calibration()
    read_calibration()
    reset_calibration()
    read_calibration()
