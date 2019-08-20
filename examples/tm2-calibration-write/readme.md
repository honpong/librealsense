# rs-pose Sample

> In order to run this example, a device supporting pose stream (T265) is required.

## Overview
This sample demonstrates how to write new calibration data into a T265 device.

## Expected Output
This sample will test write some calibration data into the OEM session of the device's EEPROM, read them back and them reset to factory calibration.  

## Code Overview

First, we include the Intel® RealSense™ Cross-Platform API.  
All but advanced functionality is provided through a single header:
```cpp
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
```

First we test write some calibration data into the OEM session of the device's EEPROM inside `test_write_oem_calibration()`:

```cpp
// set calibration data into the device's virtual table
for (auto sensor_id : { 1,2 }) {
    tm2.set_intrinsics(sensor_id, fisheye_intrinsics[sensor_id - 1]);
}
for (auto imu : { RS2_STREAM_GYRO, RS2_STREAM_ACCEL }) {
    tm2.set_motion_device_intrinsics(imu, motion_intrinsics[imu - RS2_STREAM_GYRO]);
}
tm2.write_calibration(); //write new calibration data into EEPROM
```

Then, in `test_read_oem_calibration()`, we retrieve the above new calibration data back using regular librealsense API:

```cpp
auto tm2 = dev.as<rs2::tm2>();
for (auto sensor : tm2.query_sensors()) {
    for (auto profile : sensor.get_stream_profiles()) {
        // read camera intrinsics calibration
        if (auto vp = profile.as<rs2::video_stream_profile>()) {
            print(vp.get_intrinsics());
        }
        // read imu intrinsics calibration
        if (auto mp = profile.as<rs2::motion_stream_profile>()) {
            print(mp.get_motion_intrinsics());
        }
```

At the end of `test_read_oem_calibration()`, we reset the calibration back to factory default:

```cpp
        tm2.reset_to_factory_calibration(); // reset to factory calibration
    }
}
```

