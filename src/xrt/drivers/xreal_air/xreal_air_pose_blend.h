// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air: hand the head pose over between 3DoF fusion and SLAM without jumps.
 *
 * Both sources are gravity aligned (Y up) but each has its own arbitrary yaw and the SLAM has its
 * own position origin, which also changes whenever the SLAM system re-initialises. So:
 *
 *  - While SLAM is tracking: out = align * slam. When SLAM is first acquired (or its world is new,
 *    see @p world_id) align is chosen so that the output does not move, then only its yaw and
 *    translation are kept: the tilt part (the two estimates of down differ by a degree or two)
 *    fades out over @ref XREAL_AIR_POSE_BLEND_TILT_FADE_NS.
 *  - While SLAM is not tracking: orientation = correction * fusion, correction chosen when SLAM was
 *    lost so that the orientation does not move (again only its yaw is kept for good); the position
 *    is held where SLAM left it.
 *  - When SLAM comes back in the SAME world (it only stalled, it did not re-initialise), the output
 *    starts where it is and then glides back onto the old alignment over
 *    @ref XREAL_AIR_POSE_BLEND_GLIDE_NS, so that the room stays where it was: the head may have
 *    moved during the stall and SLAM, unlike the held position, knows by how much.
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
#define XREAL_AIR_POSE_BLEND_TILT_FADE_NS ((int64_t)1000 * 1000 * 1000)

//! How long the output takes to glide back onto the old SLAM alignment after a stall.
#define XREAL_AIR_POSE_BLEND_GLIDE_NS ((int64_t)2000 * 1000 * 1000)

struct xreal_air_pose_blend
{
	//! The output currently follows SLAM.
	bool slam_active;

	//! out = align * slam, align going from align_from to align_to over [align_start_ns, + align_ns].
	struct xrt_pose align_from;
	struct xrt_pose align_to;
	int64_t align_start_ns;
	int64_t align_ns;

	//! align_to belongs to SLAM world @ref world_id.
	bool have_world;
	uint32_t world_id;

	//! While not @ref slam_active: orientation = correction * fusion, the correction fading from
	//! the full one to its yaw part from @ref fusion_switch_ns on.
	struct xrt_quat fusion_full;
	struct xrt_quat fusion_yaw;
	int64_t fusion_switch_ns;

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
 * @param at_ns    The time the pose is for (drives the fades).
 * @param slam     The SLAM head relation in a Y-up world, or NULL when SLAM is not tracking right now.
 * @param world_id Changes whenever the SLAM system re-initialised (its world is a new one).
 * @param fusion   The 3DoF fusion head relation (orientation, optional angular velocity), Y-up world.
 * @param out      The blended relation.
 */
void
xreal_air_pose_blend_update(struct xreal_air_pose_blend *b,
                            int64_t at_ns,
                            const struct xrt_space_relation *slam,
                            uint32_t world_id,
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
