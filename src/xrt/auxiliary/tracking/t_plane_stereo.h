// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Dense stereo for plane detection: a fisheye stereo pair in, a depth image out.
 *
 * Rectifies both cameras onto a common pinhole pair through the cameras' OWN projection (given as a
 * callback, so any model works; for the XREAL Air 2 Ultra the factory fisheye624, whose KB4 fit bends
 * the floor by ~17 cm across a room), box-downsamples to half resolution, runs OpenCV's SGBM (the
 * left edge padded so the disparity search does not blank the first columns) and turns the
 * disparities into depth.
 *
 * Rectified frame: x along the baseline (left to right camera), z the mean of both optical axes,
 * principal point at the image center. The remap and downsampling are hand-written and SGBM runs in
 * its single-threaded mode, so nothing here uses OpenCV's thread pool.
 *
 * Only the depth source interface is exported; a learned stereo network can replace it behind
 * @ref t_plane_depth_source.
 *
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Project a point in the camera frame (x right, y down, z forward) to pixels.
 *
 * @return false if it does not project (behind, outside the model's range).
 */
typedef bool (*t_plane_project_fn)(void *user, const double p_cam[3], double out_uv[2]);

struct t_plane_stereo_camera
{
	t_plane_project_fn project;
	void *user;
	struct xrt_pose T_imu_cam; //!< The camera's pose in the IMU (body) frame.
	int width, height;         //!< Raw image size.
};

struct t_plane_depth_source;

struct t_plane_stereo_config
{
	int rect_width;         //!< Full-resolution rectified width (default 400).
	int rect_height;        //!< ... height (default 560).
	float focal_px;         //!< Rectified focal length at full resolution (default 170).
	bool half_res;          //!< SGBM at half resolution (default true).
	int num_disparities;    //!< At the SGBM resolution, multiple of 16 (default 48 half-res, 96 full).
	int block_size;         //!< SGBM block (default 7 half-res, 5 full).
	bool pad_left;          //!< Pad the left edge by num_disparities (default true).
	float min_disparity_px; //!< Full-resolution pixels, smaller is dropped (default 2).
};

void
t_plane_stereo_default_config(struct t_plane_stereo_config *cfg);

/*!
 * SGBM as a plane detection depth source (@ref t_plane_depth.h): the depth image is the SGBM grid
 * (half resolution by default) of the rectified left camera, the confidence half a pixel of
 * full-resolution disparity.
 *
 * @param cams 0 = left, 1 = right.
 * @return 0 on success.
 */
int
t_plane_stereo_create_depth_source(const struct t_plane_stereo_config *cfg,
                                   const struct t_plane_stereo_camera cams[2],
                                   struct t_plane_depth_source **out_src);

#ifdef __cplusplus
}
#endif
