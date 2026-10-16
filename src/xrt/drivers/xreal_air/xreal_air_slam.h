// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra 6DoF: visual-inertial SLAM through Monado's t_slam / VIT (Basalt).
 *
 * Opt-in, default off: without XREAL_AIR_SLAM=1 (or in a build without XRT_FEATURE_SLAM)
 * @ref xreal_air_slam_create returns NULL and the driver is the plain 3DoF one, cameras off.
 *
 * Data flow: the capture (xreal_air_capture.c) decodes and pairs the stereo frames and maps the
 * device clock onto the host clock; it hands IMU samples (factory IMU frame, the same mapped clock)
 * and stereo pairs (stamped at mid-exposure) to this module, which feeds t_slam. The HMD asks for
 * the head pose, which is the SLAM pose converted from Basalt's frames (Z-up world, IMU body) to
 * OpenXR's (Y-up world, head body), blended with the 3DoF fusion so that SLAM starting, losing
 * tracking or re-initialising never makes the view jump (see xreal_air_pose_blend.h).
 *
 * Environment:
 *  - XREAL_AIR_SLAM=1                  Enable (also turns the camera capture on).
 *  - VIT_SYSTEM_LIBRARY_PATH=/x/libbasalt.so   The VIT implementation (Monado's t_slam option).
 *  - XREAL_AIR_SLAM_CONFIG=/x/xreal_air.toml   Basalt pipeline settings (the calibration always comes
 *                                      from the glasses). Default: share/basalt/xreal_air.toml next to
 *                                      the VIT library, else Basalt's built-in defaults.
 *  - XREAL_AIR_SLAM_CAM_OFFSET_US=0    Extra camera time offset on top of mid-exposure stamping.
 *  - XREAL_AIR_SLAM_STALE_MS=300       No new SLAM pose for this long = not tracking (3DoF fallback).
 *  - XREAL_AIR_SLAM_FRESH_MS=200       ... and only tracking again once the newest pose is younger
 *  - XREAL_AIR_SLAM_FRESH_HOLD_MS=500  than this for this long (hysteresis against flapping).
 *  - XREAL_AIR_SLAM_WARMUP_MS=1500     Let Basalt settle this long before the output follows it.
 *  - XREAL_AIR_SLAM_WEAR=1             Stop the tracker while the glasses are off (display off, the wear
 *                                      sensor; 3DoF with the position held), start a fresh one when
 *                                      they are put on again.
 *  - XREAL_AIR_SLAM_WEAR_DEBOUNCE_MS=400  How long a wear edge must hold.
 *  - XREAL_AIR_SLAM_DOFF_LOOKBACK_MS=1000 Hold the position from this long before the glasses were
 *                                      taken off (the removal itself moved them).
 *  - XREAL_AIR_SLAM_DIVERGENCE=1       Detect a tracker running away and restart it: faster than
 *                                      XREAL_AIR_SLAM_MAX_SPEED=3 m/s for 200 ms, or faster than
 *                                      XREAL_AIR_SLAM_STILL_SPEED=0.1 m/s for 300 ms while the IMU is at
 *                                      rest, or discontinuities less than 2 s apart. The output stops
 *                                      following SLAM at the first suspicious pose, and after
 *                                      a divergence only follows the fresh tracker once the
 *                                      IMU has seen the head move.
 *  - XREAL_AIR_SLAM_RESTART_MIN_MS=3000  At most one divergence restart this often.
 *  - XREAL_AIR_SLAM_KB4_STEP_PX=8      Grid step of the fisheye624 -> KB4 fit.
 *  - SLAM_* (SLAM_PREDICTION_TYPE, SLAM_WRITE_CSVS, SLAM_CSV_PATH, SLAM_LOG, ...) as for t_slam.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "util/u_logging.h"

#include "xreal_air_hmd.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct xreal_air_slam;

//! XREAL_AIR_SLAM=1 was asked for (whether or not SLAM is built in).
bool
xreal_air_slam_requested(void);

/*!
 * Create the SLAM module if requested and built in, NULL otherwise. The tracker itself is only
 * created once the factory calibration arrives, see @ref xreal_air_slam_set_calibration.
 */
struct xreal_air_slam *
xreal_air_slam_create(enum u_logging_level log_level);

/*!
 * The factory calibration arrived. Converts it and creates the tracker on the SLAM's own thread,
 * never blocks the caller.
 */
void
xreal_air_slam_set_calibration(struct xreal_air_slam *s, const struct xreal_air_parsed_calibration *calib);

/*!
 * The glasses' display went on or off: the wear sensor (proximity). The tracker is stopped once the
 * glasses have been off for the debounce time, and a fresh one started once they are back on.
 * Before the first call the glasses count as worn.
 */
void
xreal_air_slam_set_display_on(struct xreal_air_slam *s, bool on);

/*!
 * An IMU sample, factory IMU frame (x right, y down, z forward), host clock. Never blocks, drops
 * the sample when the queue to the tracker is full or the tracker is not up yet.
 */
void
xreal_air_slam_push_imu(struct xreal_air_slam *s,
                        int64_t timestamp_ns,
                        const struct xrt_vec3 *gyro_rad_s,
                        const struct xrt_vec3 *accel_m_s2);

/*!
 * A stereo pair (cam0 = left = device_1). @p left_start_ns is the left frame's telemetry device time
 * (the exposure start) mapped onto the IMU's host clock, @p exposure_ns its telemetry exposure; the
 * pair is stamped at mid-exposure (see @ref xreal_air_slam_frame_timestamp). May block while the
 * tracker's frame queue is full, call from the camera thread only.
 */
void
xreal_air_slam_push_stereo(struct xreal_air_slam *s,
                           int64_t left_start_ns,
                           uint32_t exposure_ns,
                           struct xrt_frame *left,
                           struct xrt_frame *right);

/*!
 * The head pose at @p at_timestamp_ns: SLAM when it is tracking, else @p fusion (the 3DoF
 * relation, Y-up world, head body), blended without jumps.
 */
void
xreal_air_slam_get_head_pose(struct xreal_air_slam *s,
                             int64_t at_timestamp_ns,
                             const struct xrt_space_relation *fusion,
                             struct xrt_space_relation *out);

/*!
 * For tools and logs: the tracker is up (@p out_ready), the output follows SLAM right now
 * (@p out_tracking), the tracker could not be created (@p out_failed).
 */
void
xreal_air_slam_get_status(struct xreal_air_slam *s, bool *out_ready, bool *out_tracking, bool *out_failed);

/*!
 * Stop the tracker and free everything. Call after nothing pushes any more.
 */
void
xreal_air_slam_destroy(struct xreal_air_slam **s_ptr);

#ifdef __cplusplus
}
#endif
