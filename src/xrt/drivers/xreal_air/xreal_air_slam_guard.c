// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 6DoF: decide when the SLAM output can be trusted.
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_slam_guard.h"

#include "math/m_api.h"
#include "math/m_vec3.h"

#include <math.h>
#include <string.h>

#define NS_PER_MS ((int64_t)1000 * 1000)


/*
 *
 * IMU stillness.
 *
 */

void
xreal_air_still_init(struct xreal_air_still *st, float gyro_max, float accel_range)
{
	memset(st, 0, sizeof(*st));
	st->gyro_max = gyro_max;
	st->accel_range = accel_range;
}

void
xreal_air_still_reset(struct xreal_air_still *st)
{
	xreal_air_still_init(st, st->gyro_max, st->accel_range);
}

static bool
still_evaluate(const struct xreal_air_still *st)
{
	if (st->block_count < XREAL_AIR_STILL_WINDOW_BLOCKS) {
		return false;
	}
	float accel_min = st->block_accel[0];
	float accel_max = st->block_accel[0];
	for (uint32_t i = 0; i < XREAL_AIR_STILL_WINDOW_BLOCKS; i++) {
		if (st->block_gyro[i] >= st->gyro_max) {
			return false;
		}
		accel_min = fminf(accel_min, st->block_accel[i]);
		accel_max = fmaxf(accel_max, st->block_accel[i]);
	}
	return accel_max - accel_min < st->accel_range;
}

bool
xreal_air_still_push(struct xreal_air_still *st, int64_t ts, const struct xrt_vec3 *gyro, const struct xrt_vec3 *accel)
{
	// A gap of more than a few samples: the window no longer describes a continuous stretch.
	if (st->last_ts != 0 && (ts - st->last_ts > 50 * NS_PER_MS || ts < st->last_ts)) {
		xreal_air_still_reset(st);
	}
	st->last_ts = ts;

	st->gyro_sum = m_vec3_add(st->gyro_sum, *gyro);
	st->accel_sum = m_vec3_add(st->accel_sum, *accel);
	if (++st->samples < XREAL_AIR_STILL_BLOCK_SAMPLES) {
		return st->still;
	}

	const float inv = 1.0f / (float)st->samples;
	st->block_gyro[st->block_head] = m_vec3_len(m_vec3_mul_scalar(st->gyro_sum, inv));
	st->block_accel[st->block_head] = m_vec3_len(m_vec3_mul_scalar(st->accel_sum, inv));
	st->block_head = (st->block_head + 1) % XREAL_AIR_STILL_WINDOW_BLOCKS;
	if (st->block_count < XREAL_AIR_STILL_WINDOW_BLOCKS) {
		st->block_count++;
	}
	st->gyro_sum = (struct xrt_vec3){0, 0, 0};
	st->accel_sum = (struct xrt_vec3){0, 0, 0};
	st->samples = 0;

	st->still = still_evaluate(st);
	return st->still;
}


/*
 *
 * Wear state.
 *
 */

void
xreal_air_wear_init(struct xreal_air_wear *w)
{
	memset(w, 0, sizeof(*w));
	w->raw = true;
	w->worn = true;
}

void
xreal_air_wear_raw(struct xreal_air_wear *w, bool on, int64_t now_ns)
{
	if (w->known && w->raw == on) {
		return;
	}
	w->known = true;
	w->raw = on;
	w->raw_change_ns = now_ns;
}

bool
xreal_air_wear_update(struct xreal_air_wear *w, int64_t now_ns, int64_t debounce_ns)
{
	if (!w->known || w->raw == w->worn || now_ns - w->raw_change_ns < debounce_ns) {
		return false;
	}
	w->worn = w->raw;
	w->worn_change_ns = w->raw_change_ns;
	return true;
}


/*
 *
 * SLAM pose guard.
 *
 */

void
xreal_air_slam_guard_default_config(struct xreal_air_slam_guard_config *cfg)
{
	cfg->stale_ns = 300 * NS_PER_MS;
	cfg->fresh_ns = 200 * NS_PER_MS;
	cfg->fresh_hold_ns = 500 * NS_PER_MS;
	cfg->warmup_ns = 1500 * NS_PER_MS;
	cfg->jump_distance_m = 0.25f;
	cfg->jump_angle_deg = 25.0f; // Between two 30 Hz poses: 750 deg/s, faster than heads turn.
	cfg->jump_repeat_ns = 2000 * NS_PER_MS;
	cfg->max_speed_m_s = 3.0f;
	cfg->max_speed_ns = 200 * NS_PER_MS;
	cfg->still_speed_m_s = 0.10f; // Worn and at rest, Basalt says < 0.015 m/s (living-room-02).
	cfg->still_ns = 300 * NS_PER_MS;
}

const char *
xreal_air_slam_pose_verdict_str(enum xreal_air_slam_pose_verdict v)
{
	switch (v) {
	case XREAL_AIR_SLAM_POSE_SAME: return "same";
	case XREAL_AIR_SLAM_POSE_FIRST: return "first";
	case XREAL_AIR_SLAM_POSE_OK: return "ok";
	case XREAL_AIR_SLAM_POSE_SUSPECT: return "suspect";
	case XREAL_AIR_SLAM_POSE_RESUMED: return "resumed";
	case XREAL_AIR_SLAM_POSE_REINIT: return "reinit";
	case XREAL_AIR_SLAM_POSE_DIVERGED: return "diverged";
	}
	return "?";
}

void
xreal_air_slam_guard_init(struct xreal_air_slam_guard *g, const struct xreal_air_slam_guard_config *cfg)
{
	memset(g, 0, sizeof(*g));
	g->cfg = *cfg;
}

static void
clear_suspicion(struct xreal_air_slam_guard *g)
{
	g->fast_since_ts = 0;
	g->still_moving_since_ts = 0;
	g->suspect_since_ts = 0;
	g->reason = NULL;
}

void
xreal_air_slam_guard_restart(struct xreal_air_slam_guard *g)
{
	g->have_pose = false;
	g->last_jump_ts = 0;
	g->diverged = false;
	g->fresh = false;
	g->fresh_candidate_ns = 0;
	clear_suspicion(g);
	g->world_id++;
}

static void
record(struct xreal_air_slam_guard *g, int64_t ts, const struct xrt_pose *pose)
{
	g->last_ts = ts;
	g->last_pos = pose->position;
	g->last_rot = pose->orientation;
}

//! @p since_ts tracks how long @p cond has held (0 = not), true once it held for @p duration_ns.
static bool
sustained(bool cond, int64_t ts, int64_t *since_ts, int64_t duration_ns)
{
	if (!cond) {
		*since_ts = 0;
		return false;
	}
	if (*since_ts == 0) {
		*since_ts = ts;
	}
	return ts - *since_ts >= duration_ns;
}

enum xreal_air_slam_pose_verdict
xreal_air_slam_guard_push(struct xreal_air_slam_guard *g,
                          int64_t ts,
                          const struct xrt_pose *pose,
                          const struct xrt_vec3 *velocity,
                          bool imu_still)
{
	if (!g->have_pose) {
		g->have_pose = true;
		g->warm_start_ts = ts;
		record(g, ts, pose);
		return XREAL_AIR_SLAM_POSE_FIRST;
	}
	if (ts <= g->last_ts) {
		return XREAL_AIR_SLAM_POSE_SAME;
	}

	const int64_t dt = ts - g->last_ts;
	const struct xrt_vec3 d = m_vec3_sub(pose->position, g->last_pos);
	const float dist = m_vec3_len(d);
	const float dot = fabsf(math_quat_dot(&pose->orientation, &g->last_rot));
	const float angle_deg = 2.0f * acosf(dot > 1.0f ? 1.0f : dot) * 180.0f / (float)M_PI;
	const float speed = velocity != NULL ? m_vec3_len(*velocity) : dist / ((float)dt * 1e-9f);
	g->last_dist = dist;
	g->last_angle_deg = angle_deg;
	g->last_speed = speed;
	record(g, ts, pose);

	if (g->diverged) {
		// Latched until the tracker is restarted.
		return XREAL_AIR_SLAM_POSE_DIVERGED;
	}

	if (dt > g->cfg.stale_ns) {
		// Back from a stall: same world, but let it settle again before following it.
		g->warm_start_ts = ts;
		clear_suspicion(g);
		return XREAL_AIR_SLAM_POSE_RESUMED;
	}

	if (dist > g->cfg.jump_distance_m || angle_deg > g->cfg.jump_angle_deg) {
		if (g->last_jump_ts != 0 && ts - g->last_jump_ts < g->cfg.jump_repeat_ns) {
			// A tracker re-initialises once and then moves on smoothly; discontinuities in a row
			// are a tracker running away.
			if (g->suspect_since_ts == 0) {
				g->suspect_since_ts = g->last_jump_ts;
			}
			g->reason = "discontinuities in a row";
			g->diverged = true;
			return XREAL_AIR_SLAM_POSE_DIVERGED;
		}
		g->last_jump_ts = ts;
		g->world_id++;
		g->warm_start_ts = ts;
		clear_suspicion(g);
		return XREAL_AIR_SLAM_POSE_REINIT;
	}

	const bool too_fast = sustained(speed > g->cfg.max_speed_m_s, ts, &g->fast_since_ts, g->cfg.max_speed_ns);
	const bool moving_still =
	    sustained(imu_still && speed > g->cfg.still_speed_m_s, ts, &g->still_moving_since_ts, g->cfg.still_ns);

	if (g->fast_since_ts == 0 && g->still_moving_since_ts == 0) {
		g->suspect_since_ts = 0;
		return XREAL_AIR_SLAM_POSE_OK;
	}

	if (g->suspect_since_ts == 0) {
		g->suspect_since_ts = ts;
	}
	g->reason = g->fast_since_ts != 0 ? "faster than a head moves" : "moving while the IMU is at rest";
	if (too_fast || moving_still) {
		g->diverged = true;
		return XREAL_AIR_SLAM_POSE_DIVERGED;
	}
	return XREAL_AIR_SLAM_POSE_SUSPECT;
}

bool
xreal_air_slam_guard_usable(struct xreal_air_slam_guard *g, int64_t now_ns, int64_t latest_ts)
{
	if (!g->have_pose) {
		g->fresh = false;
		g->fresh_candidate_ns = 0;
		return false;
	}

	const int64_t lag = now_ns - latest_ts;
	g->last_lag_ns = lag;
	if (g->fresh) {
		if (lag > g->cfg.stale_ns) {
			g->fresh = false;
			g->fresh_candidate_ns = 0;
		}
	} else if (lag <= g->cfg.fresh_ns) {
		if (g->fresh_candidate_ns == 0) {
			g->fresh_candidate_ns = now_ns;
		}
		if (now_ns - g->fresh_candidate_ns >= g->cfg.fresh_hold_ns) {
			g->fresh = true;
		}
	} else {
		g->fresh_candidate_ns = 0;
	}

	if (!g->fresh || g->diverged || g->suspect_since_ts != 0) {
		return false;
	}
	return latest_ts - g->warm_start_ts >= g->cfg.warmup_ns;
}


/*
 *
 * Position history.
 *
 */

void
xreal_air_pos_history_init(struct xreal_air_pos_history *h, int64_t min_interval_ns)
{
	memset(h, 0, sizeof(*h));
	h->min_interval_ns = min_interval_ns;
}

void
xreal_air_pos_history_clear(struct xreal_air_pos_history *h)
{
	xreal_air_pos_history_init(h, h->min_interval_ns);
}

void
xreal_air_pos_history_push(struct xreal_air_pos_history *h, int64_t ts, const struct xrt_vec3 *pos)
{
	if (h->count > 0) {
		const uint32_t newest = (h->head + XREAL_AIR_POS_HISTORY_SIZE - 1) % XREAL_AIR_POS_HISTORY_SIZE;
		if (ts < h->ts[newest] + h->min_interval_ns) {
			return;
		}
	}
	h->ts[h->head] = ts;
	h->pos[h->head] = *pos;
	h->head = (h->head + 1) % XREAL_AIR_POS_HISTORY_SIZE;
	if (h->count < XREAL_AIR_POS_HISTORY_SIZE) {
		h->count++;
	}
}

bool
xreal_air_pos_history_at(const struct xreal_air_pos_history *h, int64_t ts, struct xrt_vec3 *out_pos)
{
	if (h->count == 0) {
		return false;
	}
	// Newest to oldest.
	uint32_t idx = h->head;
	for (uint32_t n = 0; n < h->count; n++) {
		idx = (idx + XREAL_AIR_POS_HISTORY_SIZE - 1) % XREAL_AIR_POS_HISTORY_SIZE;
		if (h->ts[idx] <= ts) {
			*out_pos = h->pos[idx];
			return true;
		}
	}
	*out_pos = h->pos[idx]; // The oldest.
	return true;
}
