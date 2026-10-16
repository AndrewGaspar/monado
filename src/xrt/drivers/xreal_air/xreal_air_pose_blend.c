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
	b->align_from.orientation = identity_quat;
	b->align_to.orientation = identity_quat;
	b->fusion_full = identity_quat;
	b->fusion_yaw = identity_quat;
	b->last_out.orientation = identity_quat;
}

//! The world-frame rotation taking @p from onto @p to.
static void
correction_between(const struct xrt_quat *to, const struct xrt_quat *from, struct xrt_quat *out)
{
	struct xrt_quat inv_from;
	math_quat_invert(from, &inv_from);
	math_quat_rotate(to, &inv_from, out); // out * from = to
	math_quat_normalize(out);
}

static float
fade(int64_t at_ns, int64_t start_ns, int64_t duration_ns)
{
	const int64_t dt = at_ns - start_ns;
	if (dt <= 0) {
		return 0.0f;
	}
	if (dt >= duration_ns) {
		return 1.0f;
	}
	return (float)dt / (float)duration_ns;
}

static void
current_align(const struct xreal_air_pose_blend *b, int64_t at_ns, struct xrt_pose *out)
{
	const float t = fade(at_ns, b->align_start_ns, b->align_ns);
	if (t >= 1.0f) {
		*out = b->align_to;
		return;
	}
	math_pose_interpolate(&b->align_from, &b->align_to, t, out);
	math_quat_normalize(&out->orientation);
}

static void
current_fusion_correction(const struct xreal_air_pose_blend *b, int64_t at_ns, struct xrt_quat *out)
{
	const float t = fade(at_ns, b->fusion_switch_ns, XREAL_AIR_POSE_BLEND_TILT_FADE_NS);
	if (t >= 1.0f) {
		*out = b->fusion_yaw;
		return;
	}
	math_quat_slerp(&b->fusion_full, &b->fusion_yaw, t, out);
	math_quat_normalize(out);
}

//! align = (rotation, translation) such that rotation * slam_pos + translation = out_pos.
static void
anchored_align(const struct xrt_quat *rotation,
               const struct xrt_vec3 *slam_pos,
               const struct xrt_vec3 *out_pos,
               struct xrt_pose *out)
{
	struct xrt_vec3 rotated;
	math_quat_rotate_vec3(rotation, slam_pos, &rotated);
	out->orientation = *rotation;
	out->position.x = out_pos->x - rotated.x;
	out->position.y = out_pos->y - rotated.y;
	out->position.z = out_pos->z - rotated.z;
}

static void
acquire(struct xreal_air_pose_blend *b,
        int64_t at_ns,
        const struct xrt_space_relation *slam,
        uint32_t world_id,
        const struct xrt_quat *fusion_q)
{
	// What we output right now, which the first SLAM output must equal.
	struct xrt_quat correction, current;
	current_fusion_correction(b, at_ns, &correction);
	math_quat_rotate(&correction, fusion_q, &current);

	struct xrt_quat slam_q = slam->pose.orientation;
	math_quat_normalize(&slam_q);

	struct xrt_quat full;
	correction_between(&current, &slam_q, &full);
	anchored_align(&full, &slam->pose.position, &b->held_position, &b->align_from);

	if (b->have_world && b->world_id == world_id) {
		// Same world, it only stalled: glide back to where the room was.
		b->align_ns = XREAL_AIR_POSE_BLEND_GLIDE_NS;
	} else {
		// A new world: keep the yaw and the position of the anchoring, fade the tilt out.
		struct xrt_quat yaw;
		xreal_air_pose_blend_yaw_of(&full, &yaw);
		anchored_align(&yaw, &slam->pose.position, &b->held_position, &b->align_to);
		b->align_ns = XREAL_AIR_POSE_BLEND_TILT_FADE_NS;
		b->have_world = true;
		b->world_id = world_id;
	}
	b->align_start_ns = at_ns;
	b->slam_active = true;
	b->slam_acquired++;
}

//! Lost: keep the orientation where SLAM left it, hold the position.
static void
lose(struct xreal_air_pose_blend *b, int64_t at_ns, const struct xrt_quat *fusion_q)
{
	b->held_position = b->last_out.position;
	correction_between(&b->last_out.orientation, fusion_q, &b->fusion_full);
	xreal_air_pose_blend_yaw_of(&b->fusion_full, &b->fusion_yaw);
	b->fusion_switch_ns = at_ns;
	b->slam_active = false;
	b->slam_lost++;
}

void
xreal_air_pose_blend_hold(struct xreal_air_pose_blend *b,
                          int64_t at_ns,
                          const struct xrt_space_relation *fusion,
                          const struct xrt_vec3 *position)
{
	if (b->slam_active) {
		struct xrt_quat fusion_q = fusion->pose.orientation;
		math_quat_normalize(&fusion_q);
		lose(b, at_ns, &fusion_q);
	}
	b->held_position = *position;
}

void
xreal_air_pose_blend_update(struct xreal_air_pose_blend *b,
                            int64_t at_ns,
                            const struct xrt_space_relation *slam,
                            uint32_t world_id,
                            const struct xrt_space_relation *fusion,
                            struct xrt_space_relation *out)
{
	struct xrt_quat fusion_q = fusion->pose.orientation;
	math_quat_normalize(&fusion_q);

	if (slam != NULL) {
		if (!b->slam_active || b->world_id != world_id) {
			if (b->slam_active) {
				// Re-initialised without a gap: re-anchor from where the output is.
				b->held_position = b->last_out.position;
				correction_between(&b->last_out.orientation, &fusion_q, &b->fusion_full);
				b->fusion_yaw = b->fusion_full;
				b->fusion_switch_ns = at_ns;
			}
			acquire(b, at_ns, slam, world_id, &fusion_q);
		}

		struct xrt_pose align;
		current_align(b, at_ns, &align);

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
		lose(b, at_ns, &fusion_q);
	}

	struct xrt_quat correction;
	current_fusion_correction(b, at_ns, &correction);

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
