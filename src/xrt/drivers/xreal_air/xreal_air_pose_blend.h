// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air: hand the head pose over between 3DoF fusion and SLAM without jumps.
 *
 * Both sources are gravity aligned (Y up) but each has its own arbitrary yaw and the SLAM has its
 * own position origin, which also changes whenever the SLAM system (re)initialises. So the output
 * frame is defined by continuity instead:
 *
 *  - While SLAM is tracking: out = align * slam, where align (a rotation plus a translation) was
 *    chosen when SLAM (re)started so that the output did not move.
 *  - While SLAM is not tracking: orientation = correction * fusion, where correction was chosen when
 *    SLAM was lost so that the orientation did not move; the position is held at the last output.
 *
 * Only the yaw (about +Y) part of a correction is kept for good: both sources know where down is,
 * so a lasting tilt correction would be wrong. The tilt part (the two estimates of down differ by
 * a degree or two) is faded out over @ref XREAL_AIR_POSE_BLEND_TILT_FADE_NS instead of snapping.
 *
 * Pure bookkeeping, the caller serialises the calls.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! How long the tilt part of a hand-over correction takes to fade out.
#define XREAL_AIR_POSE_BLEND_TILT_FADE_NS (1000 * 1000 * 1000)

struct xreal_air_pose_blend
{
	//! The output currently follows SLAM.
	bool slam_active;

	//! When the last hand-over happened (pose time), drives the tilt fade.
	int64_t switch_ns;

	//! The full and the yaw-only correction of the last hand-over, the fade goes from one to the other.
	struct xrt_quat correction_full;
	struct xrt_quat correction_yaw;

	//! While SLAM is active: out position = anchor_out + R * (slam - anchor_slam).
	struct xrt_vec3 anchor_slam;
	struct xrt_vec3 anchor_out;

	//! The position output while not @ref slam_active.
	struct xrt_vec3 held_position;

	//! The last output pose.
	struct xrt_pose last_out;

	//! How often the output switched to SLAM / back to the fusion.
	uint32_t slam_acquired;
	uint32_t slam_lost;
};

void
xreal_air_pose_blend_init(struct xreal_air_pose_blend *b);

/*!
 * Produce the head pose.
 *
 * @param at_ns  The time the pose is for (drives the tilt fade).
 * @param slam   The SLAM head relation in a Y-up world, or NULL when SLAM is not tracking right now.
 * @param fusion The 3DoF fusion head relation (orientation, optional angular velocity), Y-up world.
 * @param out    The blended relation.
 */
void
xreal_air_pose_blend_update(struct xreal_air_pose_blend *b,
                            int64_t at_ns,
                            const struct xrt_space_relation *slam,
                            const struct xrt_space_relation *fusion,
                            struct xrt_space_relation *out);

/*!
 * The rotation about +Y (yaw) part of @p q, as a quaternion.
 */
void
xreal_air_pose_blend_yaw_of(const struct xrt_quat *q, struct xrt_quat *out_yaw);

/*!
 * The fixed transforms from Basalt's frames (Z-up world, factory IMU body: x right, y down,
 * z forward) to the OpenXR head pose (Y-up world, head body: x right, y up, z back):
 * head = R_xr_world * T_world_imu * R_imu_head.
 */
void
xreal_air_basalt_to_head(const struct xrt_space_relation *world_imu, struct xrt_space_relation *out_head);

#ifdef __cplusplus
}
#endif
