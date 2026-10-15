// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air: hand the head pose over between 3DoF fusion and SLAM without jumps.
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_pose_blend.h"

#include "math/m_api.h"
#include "math/m_space.h"

#include <string.h>

static const struct xrt_quat identity_quat = {0, 0, 0, 1};

void
xreal_air_pose_blend_yaw_of(const struct xrt_quat *q, struct xrt_quat *out_yaw)
{
	const struct xrt_vec3 up = {0, 1, 0};
	struct xrt_quat swing, twist;
	math_quat_decompose_swing_twist(q, &up, &swing, &twist);
	math_quat_normalize(&twist);
	*out_yaw = twist;
}

void
xreal_air_pose_blend_init(struct xreal_air_pose_blend *b)
{
	memset(b, 0, sizeof(*b));
	b->correction_full = identity_quat;
	b->correction_yaw = identity_quat;
	b->last_out.orientation = identity_quat;
}

//! Start a hand-over: the world-frame correction taking @p from onto @p to, and its yaw part.
static void
start_switch(struct xreal_air_pose_blend *b, int64_t at_ns, const struct xrt_quat *to, const struct xrt_quat *from)
{
	struct xrt_quat inv_from, full;
	math_quat_invert(from, &inv_from);
	math_quat_rotate(to, &inv_from, &full); // full * from = to
	math_quat_normalize(&full);

	b->correction_full = full;
	xreal_air_pose_blend_yaw_of(&full, &b->correction_yaw);
	b->switch_ns = at_ns;
}

//! The current correction: full right after the hand-over, yaw only after the fade.
static void
current_correction(const struct xreal_air_pose_blend *b, int64_t at_ns, struct xrt_quat *out)
{
	const int64_t dt = at_ns - b->switch_ns;
	if (dt >= XREAL_AIR_POSE_BLEND_TILT_FADE_NS) {
		*out = b->correction_yaw;
		return;
	}
	const float t = dt <= 0 ? 0.0f : (float)dt / (float)XREAL_AIR_POSE_BLEND_TILT_FADE_NS;
	math_quat_slerp(&b->correction_full, &b->correction_yaw, t, out);
	math_quat_normalize(out);
}

void
xreal_air_pose_blend_update(struct xreal_air_pose_blend *b,
                            int64_t at_ns,
                            const struct xrt_space_relation *slam,
                            const struct xrt_space_relation *fusion,
                            struct xrt_space_relation *out)
{
	struct xrt_quat fusion_q = fusion->pose.orientation;
	math_quat_normalize(&fusion_q);

	if (slam != NULL) {
		if (!b->slam_active) {
			/*
			 * (Re)acquire: rotate the SLAM world so that it matches what we output right now and
			 * shift it so that its position lands on the held position.
			 */
			struct xrt_quat current, correction;
			current_correction(b, at_ns, &correction);
			math_quat_rotate(&correction, &fusion_q, &current);

			struct xrt_quat slam_q = slam->pose.orientation;
			math_quat_normalize(&slam_q);
			start_switch(b, at_ns, &current, &slam_q);

			b->anchor_slam = slam->pose.position;
			b->anchor_out = b->held_position;
			b->slam_active = true;
			b->slam_acquired++;
		}

		struct xrt_pose align;
		current_correction(b, at_ns, &align.orientation);
		struct xrt_vec3 rotated;
		math_quat_rotate_vec3(&align.orientation, &b->anchor_slam, &rotated);
		align.position.x = b->anchor_out.x - rotated.x;
		align.position.y = b->anchor_out.y - rotated.y;
		align.position.z = b->anchor_out.z - rotated.z;

		struct xrt_relation_chain xrc = {0};
		m_relation_chain_push_relation(&xrc, slam);
		m_relation_chain_push_pose(&xrc, &align);
		m_relation_chain_resolve(&xrc, out);
		out->relation_flags = (enum xrt_space_relation_flags)(
		    out->relation_flags | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		b->last_out = out->pose;
		return;
	}

	if (b->slam_active) {
		// Lost: keep the orientation where SLAM left it, hold the position.
		b->held_position = b->last_out.position;
		start_switch(b, at_ns, &b->last_out.orientation, &fusion_q);
		b->slam_active = false;
		b->slam_lost++;
	}

	struct xrt_quat correction;
	current_correction(b, at_ns, &correction);

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	math_quat_rotate(&correction, &fusion_q, &rel.pose.orientation);
	rel.pose.position = b->held_position;
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                     XRT_SPACE_RELATION_POSITION_VALID_BIT);
	if ((fusion->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0) {
		math_quat_rotate_vec3(&correction, &fusion->angular_velocity, &rel.angular_velocity);
		rel.relation_flags =
		    (enum xrt_space_relation_flags)(rel.relation_flags | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	}

	*out = rel;
	b->last_out = rel.pose;
}

void
xreal_air_basalt_to_head(const struct xrt_space_relation *world_imu, struct xrt_space_relation *out_head)
{
	/*
	 * Basalt's body is the factory IMU frame (x right, y down, z forward), the OpenXR head frame is
	 * (x right, y up, z back): 180 degrees about x, the same mapping the 3DoF path applies to the
	 * samples (post_biased_coordinate_system). Basalt's world is Z-up (gravity along -z), OpenXR's
	 * is Y-up: -90 degrees about x, (x, y, z) -> (x, z, -y). With the IMU looking forward at
	 * initialisation, Basalt puts forward along +y, which lands on OpenXR's forward, -z.
	 */
	static const struct xrt_pose imu_head = {{1, 0, 0, 0}, {0, 0, 0}};
	static const struct xrt_pose xr_world = {{-0.70710678118654752f, 0, 0, 0.70710678118654752f}, {0, 0, 0}};

	struct xrt_relation_chain xrc = {0};
	m_relation_chain_push_pose(&xrc, &imu_head);
	m_relation_chain_push_relation(&xrc, world_imu);
	m_relation_chain_push_pose(&xrc, &xr_world);
	m_relation_chain_resolve(&xrc, out_head);
}
