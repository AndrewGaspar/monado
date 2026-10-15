// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra SLAM calibration conversion (factory blob -> what Basalt/VIT takes).
 *
 * Pure math, no device or tracker access, so it can be unit tested on the factory values.
 *
 *  - Extrinsics: the factory imu_q_cam is a JPL quaternion of the camera in the IMU frame, the
 *    Hamilton rotation taking camera vectors into the IMU frame is its conjugate. Verified offline:
 *    every other reading (plain Hamilton, inverted, swapped cameras) collapses stereo matching.
 *  - Intrinsics: the cameras are calibrated as "fisheye624" (radial k0..k5, tangential p0 p1, thin
 *    prism s0..s3) but VIT only carries Kannala-Brandt KB4, so a KB4 model is least-squares fitted
 *    to the fisheye624 one over a dense pixel grid (RMS 1.5 / 0.6 px on the two cameras of the
 *    user's unit, tracking quality identical to native fisheye624 offline).
 *  - Timestamps: the telemetry device time marks the start of the exposure, Basalt wants the
 *    middle of it.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xreal_air_hmd.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Default pixel step of the KB4 fitting grid (4800 points on 480x640).
#define XREAL_AIR_KB4_FIT_DEFAULT_STEP_PX 8.0

//! Used for the mid-exposure offset when a frame carries no usable exposure time.
#define XREAL_AIR_SLAM_DEFAULT_HALF_EXPOSURE_NS (8 * 1000 * 1000)

//! Exposures above this are not believed (the telemetry field is only MED confidence).
#define XREAL_AIR_SLAM_MAX_EXPOSURE_NS (40 * 1000 * 1000)

/*!
 * A Kannala-Brandt (KB4) camera: u = fx * d(theta) * cos(phi) + cx, v = fy * d(theta) * sin(phi) + cy
 * with d(theta) = theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8).
 */
struct xreal_air_kb4
{
	double fx, fy, cx, cy;
	double k[4];

	// Fit statistics, pixels, over the fitting grid.
	double rms_px;
	double max_px;
	int points;
	int iterations;
};

/*!
 * T_imu_cam (the camera's pose in the IMU frame) from the factory imu_p_cam / imu_q_cam, the
 * rotation as a normalized Hamilton quaternion (x, y, z, w): the conjugate of the JPL one.
 */
void
xreal_air_slam_T_imu_cam(const struct xreal_air_slam_camera_calibration *cam, double out_q_xyzw[4], double out_p[3]);

/*!
 * Project a point in the camera frame with the fisheye624 model (Basalt's Fisheye624Camera).
 *
 * @param p16 fx, fy, cx, cy, k0..k5, p0, p1, s0..s3.
 */
void
xreal_air_fisheye624_project(const double p16[16], const double point[3], double out_uv[2]);

/*!
 * Unproject a pixel with the fisheye624 model (Newton inversion of the distortion and of the radial
 * polynomial), giving the incidence angle theta (from the optical axis) and the azimuth phi.
 *
 * @return false if the inversion did not converge.
 */
bool
xreal_air_fisheye624_unproject(const double p16[16], const double uv[2], double *out_theta, double *out_phi);

//! Project a direction given as (theta, phi) with a KB4 model.
void
xreal_air_kb4_project_angles(const struct xreal_air_kb4 *kb4, double theta, double phi, double out_uv[2]);

/*!
 * Fit a KB4 model to a factory fisheye624 camera over a grid of pixels covering the whole image
 * (step @p step_px, <= 0 for the default), by Levenberg-Marquardt on the pixel reprojection error.
 *
 * @return false if the camera is not a valid fisheye624 one or the fit failed.
 */
bool
xreal_air_kb4_fit(const struct xreal_air_slam_camera_calibration *cam, double step_px, struct xreal_air_kb4 *out);

/*!
 * The timestamp Basalt wants for a stereo pair: the middle of the exposure. @p left_start_ns is the
 * host-clock mapping of the left frame's telemetry device time (exposure start), @p exposure_ns the
 * telemetry exposure (falls back to @ref XREAL_AIR_SLAM_DEFAULT_HALF_EXPOSURE_NS * 2 when zero or
 * unbelievable), @p extra_offset_ns an additional user tweak.
 */
int64_t
xreal_air_slam_frame_timestamp(int64_t left_start_ns, uint32_t exposure_ns, int64_t extra_offset_ns);

#ifdef __cplusplus
}
#endif
