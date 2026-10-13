// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra opt-in SLAM camera capture and dataset recording.
 *
 * Ties the UVC camera capture, the frame decoding, the camera/IMU clock mapping and the dataset
 * recorder to the HMD driver through three hooks. Everything is opt-in, without the environment
 * variables below @ref xreal_air_capture_create returns NULL and the driver behaves exactly as
 * before (the camera node is never opened).
 *
 *  - XREAL_AIR_CAMERAS=1              Capture the SLAM cameras (debug GUI sinks, record button).
 *  - XREAL_AIR_EUROC_RECORD=/path     Implies cameras, record a dataset from the start into /path
 *                                     (or /path_<datetime> if /path is not empty). Also the base
 *                                     name for recordings started from the debug GUI.
 *  - XREAL_AIR_CAMERA_DEVICE=/dev/vN  Override the camera node, found by USB id otherwise.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "util/u_logging.h"

#include "xreal_air_hmd.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xreal_air_capture;

/*!
 * Create the capture if enabled by the environment, NULL otherwise (or on failure).
 */
struct xreal_air_capture *
xreal_air_capture_create(enum u_logging_level log_level);

/*!
 * The factory calibration arrived, @p buffer is the reassembled HID blob.
 */
void
xreal_air_capture_set_calibration(struct xreal_air_capture *cap,
                                  const char *buffer,
                                  size_t size,
                                  const struct xreal_air_parsed_calibration *calib);

/*!
 * An IMU sample was fused, called on the sensor thread.
 *
 * @param device_ts   The raw HID timestamp of the sample.
 * @param fusion_ns   The timestamp handed to the 3DoF fusion.
 * @param raw         The raw sample.
 * @param temperature Celsius.
 * @param gyro        Factory calibrated, in the driver's output frame (rad/s).
 * @param accel       Factory calibrated, in the driver's output frame (m/s^2).
 */
void
xreal_air_capture_push_imu(struct xreal_air_capture *cap,
                           uint64_t device_ts,
                           int64_t fusion_ns,
                           const struct xreal_air_parsed_sample *raw,
                           float temperature,
                           const struct xrt_vec3 *gyro,
                           const struct xrt_vec3 *accel);

/*!
 * Stop capturing and close any recording. Sets @p cap_ptr to NULL.
 */
void
xreal_air_capture_destroy(struct xreal_air_capture **cap_ptr);

#ifdef __cplusplus
}
#endif
