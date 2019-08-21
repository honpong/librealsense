#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

####################################################################
##           librealsense T261 calibration r/w sample             ##
####################################################################

import pyrealsense2 as rs
import time

ctx = rs.context()

devs = None
while not devs:
    devs = ctx.query_devices()
    print("Waiting for device...")
    time.sleep(1)
print(len(devs), "devices found")

tm2 = None
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

#write_calibration()
fisheye_intrinsics = rs.intrinsics()
motion_intrinsics = rs.motion_device_intrinsic()

tm2.set_intrinsics(1, fisheye_intrinsics)
tm2.set_intrinsics(2, fisheye_intrinsics)

tm2.set_motion_device_intrinsics(rs.stream.accel, motion_intrinsics)
tm2.set_motion_device_intrinsics(rs.stream.gyro, motion_intrinsics)

tm2.write_calibration()

# read
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

tm2.reset_to_factory_calibration()

# read
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