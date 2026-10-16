// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Depth source interface for plane detection: a stereo pair in, a depth image with a
 *         per-pixel confidence out.
 *
 * The plane detector only needs metric depth on a pinhole grid plus how much to trust each pixel's
 * height, so where the depth comes from is swappable: half-resolution SGBM today
 * (@ref t_plane_stereo_create_depth_source), a learned stereo network later.
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
 * A depth image on a pinhole camera (x right, y down, z forward). The buffers belong to the source
 * and stay valid until its next compute.
 */
struct t_plane_depth_image
{
	uint32_t width, height;
	float fx, fy, cx, cy;
	struct xrt_pose T_imu_cam; //!< The depth camera in the IMU (body) frame.
	const float *depth;        //!< Meters along z, row major, <= 0 for no depth.
	/*!
	 * The confidence: a point's height uncertainty is |its height - the camera's| * rel_sigma (for
	 * stereo, half a pixel of disparity: 0.5 / disparity). Row major, like @ref depth.
	 */
	const float *rel_sigma;
};

//! Where the time went, for the statistics (meaning per source; SGBM: rectify, match).
struct t_plane_depth_timing
{
	float prepare_ms;
	float match_ms;
};

struct t_plane_depth_source
{
	/*!
	 * Depth for a stereo pair (cam0 = left). Called from the detector's thread only.
	 *
	 * @return false if there is no depth for this pair.
	 */
	bool (*compute)(struct t_plane_depth_source *src,
	                const struct xrt_frame *left,
	                const struct xrt_frame *right,
	                struct t_plane_depth_image *out,
	                struct t_plane_depth_timing *out_timing);

	void (*destroy)(struct t_plane_depth_source *src);
};

static inline void
t_plane_depth_source_destroy(struct t_plane_depth_source **src_ptr)
{
	struct t_plane_depth_source *src = *src_ptr;
	if (src == NULL) {
		return;
	}
	*src_ptr = NULL;
	src->destroy(src);
}

#ifdef __cplusplus
}
#endif
