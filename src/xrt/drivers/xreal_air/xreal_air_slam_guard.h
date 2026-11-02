// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 6DoF: decide when the SLAM output can be trusted.
 *
 * Pure bookkeeping (no threads, no clocks of its own), so that it can be unit tested and run on
 * recordings:
 *
 *  - @ref xreal_air_still: is the IMU at rest (glasses lying on a desk)?
 *  - @ref xreal_air_wear: the debounced wear state from the glasses' display on/off (proximity)
 *    edges. Unknown until the first edge, and then assumed worn.
 *  - @ref xreal_air_presence: the same display on/off edges, debounced into the user presence the
 *    HMD hands out (XR_EXT_user_presence), seeded at start since the glasses only report changes.
 *  - @ref xreal_air_slam_guard: looks at every new SLAM pose. Tells a stall (pose gap, same world)
 *    from a re-initialisation (one discontinuity, new world) from a divergence (implausible motion:
 *    too fast for a head, or moving while the IMU is at rest, or discontinuities in a row), and
 *    keeps the freshness of the pose stream with hysteresis so that a pose latency hovering around
 *    the stale threshold does not flap tracking on and off.
 *  - @ref xreal_air_pos_history: recent output positions, to hold the position from before the
 *    glasses were taken off or before a divergence started instead of where it ended up.
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


/*
 *
 * IMU stillness.
 *
 */

//! Samples averaged into one block (1 kHz IMU: 10 ms), which takes the sensor noise out.
#define XREAL_AIR_STILL_BLOCK_SAMPLES 10
//! Blocks in the window (0.3 s).
#define XREAL_AIR_STILL_WINDOW_BLOCKS 30

struct xreal_air_still
{
	//! At rest: every block mean |gyro| below this (rad/s) ...
	float gyro_max;
	//! ... and the block mean |accel| within this range (m/s^2) over the window.
	float accel_range;

	// The block being summed.
	struct xrt_vec3 gyro_sum, accel_sum;
	uint32_t samples;

	// Finished blocks, a ring.
	float block_gyro[XREAL_AIR_STILL_WINDOW_BLOCKS];
	float block_accel[XREAL_AIR_STILL_WINDOW_BLOCKS];
	uint32_t block_head;
	uint32_t block_count;

	bool still;
	int64_t last_ts;
};

void
xreal_air_still_init(struct xreal_air_still *st, float gyro_max, float accel_range);

//! Feed one IMU sample, returns whether the IMU is at rest over the window ending with it.
bool
xreal_air_still_push(struct xreal_air_still *st, int64_t ts, const struct xrt_vec3 *gyro, const struct xrt_vec3 *accel);

//! Forget everything (after a gap in the samples).
void
xreal_air_still_reset(struct xreal_air_still *st);


/*
 *
 * Wear state.
 *
 */

struct xreal_air_wear
{
	bool known;             //!< Any edge seen yet (the glasses only report changes).
	bool raw;               //!< The last raw state.
	int64_t raw_change_ns;  //!< When it last changed.
	bool worn;              //!< Debounced, true until known otherwise.
	int64_t worn_change_ns; //!< The raw edge the current @ref worn came from.
};

void
xreal_air_wear_init(struct xreal_air_wear *w);

//! A raw display on/off edge (the glasses' wear sensor) at @p now_ns.
void
xreal_air_wear_raw(struct xreal_air_wear *w, bool on, int64_t now_ns);

//! Debounce: true when @ref xreal_air_wear::worn changed in this call.
bool
xreal_air_wear_update(struct xreal_air_wear *w, int64_t now_ns, int64_t debounce_ns);


/*
 *
 * User presence.
 *
 */

/*!
 * The glasses push their display state only when it changes (0x6C04 DISPLAY_TOGGLED, the display
 * toggle button, the OLED text log); there is no known request that reads it. So the state at start
 * is an assumption: the HID side of the glasses is only enumerated while they are (recently) worn,
 * so they start present, and the first real edge corrects it.
 */
struct xreal_air_presence
{
	bool display_on;   //!< The last raw display state.
	int64_t change_ns; //!< When @ref display_on last changed, 0 for the seeded state.
	bool reported;     //!< The debounced presence last handed out.
};

//! Start with @p initial as the settled state: handed out from the first query, no debounce.
void
xreal_air_presence_init(struct xreal_air_presence *p, bool initial);

//! A raw display state at @p now_ns, true when it is a real edge.
bool
xreal_air_presence_set(struct xreal_air_presence *p, bool on, int64_t now_ns);

//! The presence to hand out: a raw edge once it has held for @p debounce_ns.
bool
xreal_air_presence_get(struct xreal_air_presence *p, int64_t now_ns, int64_t debounce_ns);


/*
 *
 * SLAM pose guard.
 *
 */

struct xreal_air_slam_guard_config
{
	int64_t stale_ns;      //!< Lose tracking when the newest pose is older than this (and a gap this
	                       //!< long between poses is a stall).
	int64_t fresh_ns;      //!< Only (re)gain tracking once the newest pose is younger than this ...
	int64_t fresh_hold_ns; //!< ... for this long without interruption.
	int64_t warmup_ns;     //!< Follow a (re)started pose stream only after this long.

	float jump_distance_m; //!< Between consecutive poses (no stall): a re-initialisation.
	float jump_angle_deg;
	int64_t jump_repeat_ns; //!< A second discontinuity within this of the last one: diverged.

	float max_speed_m_s;   //!< Faster than any head: suspicious, ...
	int64_t max_speed_ns;  //!< ... diverged when it lasts this long.
	float still_speed_m_s; //!< Moving this fast while the IMU is at rest: suspicious, ...
	int64_t still_ns;      //!< ... diverged when it lasts this long.
};

void
xreal_air_slam_guard_default_config(struct xreal_air_slam_guard_config *cfg);

enum xreal_air_slam_pose_verdict
{
	XREAL_AIR_SLAM_POSE_SAME,     //!< Not a new pose.
	XREAL_AIR_SLAM_POSE_FIRST,    //!< The first pose of this tracker (warming up).
	XREAL_AIR_SLAM_POSE_OK,       //!< Continuous with the last one.
	XREAL_AIR_SLAM_POSE_SUSPECT,  //!< Implausible motion, not confirmed yet: do not follow it.
	XREAL_AIR_SLAM_POSE_RESUMED,  //!< After a stall, same world (warming up again).
	XREAL_AIR_SLAM_POSE_REINIT,   //!< One discontinuity: the tracker re-initialised, a new world.
	XREAL_AIR_SLAM_POSE_DIVERGED, //!< Confirmed implausible: reset the tracker.
};

const char *
xreal_air_slam_pose_verdict_str(enum xreal_air_slam_pose_verdict v);

struct xreal_air_slam_guard
{
	struct xreal_air_slam_guard_config cfg;

	bool have_pose;
	int64_t last_ts;
	struct xrt_vec3 last_pos;
	struct xrt_quat last_rot;
	int64_t warm_start_ts;

	//! Bumped on every re-initialisation and restart, the blend's world id.
	uint32_t world_id;

	int64_t last_jump_ts;          //!< 0: none since the last restart.
	int64_t fast_since_ts;         //!< 0: not too fast.
	int64_t still_moving_since_ts; //!< 0: not moving while still.
	//! When the current suspicion (or the divergence it became) started, pose time.
	int64_t suspect_since_ts;
	bool diverged;      //!< Latched until @ref xreal_air_slam_guard_restart.
	const char *reason; //!< Why it is suspect / diverged, for logs.

	//! Freshness with hysteresis.
	bool fresh;
	int64_t fresh_candidate_ns; //!< 0: not a candidate.

	// For logs.
	float last_speed;
	float last_dist;
	float last_angle_deg;
	int64_t last_lag_ns;
};

void
xreal_air_slam_guard_init(struct xreal_air_slam_guard *g, const struct xreal_air_slam_guard_config *cfg);

/*!
 * The tracker was reset / recreated: its next pose starts a new world.
 */
void
xreal_air_slam_guard_restart(struct xreal_air_slam_guard *g);

/*!
 * Look at the tracker's newest pose.
 *
 * @param ts        Its timestamp.
 * @param pose      Its pose (any fixed world frame, only differences and speeds are used).
 * @param velocity  The tracker's own velocity estimate, NULL to use the finite difference.
 * @param imu_still The IMU is at rest right now.
 */
enum xreal_air_slam_pose_verdict
xreal_air_slam_guard_push(struct xreal_air_slam_guard *g,
                          int64_t ts,
                          const struct xrt_pose *pose,
                          const struct xrt_vec3 *velocity,
                          bool imu_still);

/*!
 * Whether the output may follow SLAM at @p now_ns, the newest pose being from @p latest_ts.
 * Updates the freshness hysteresis.
 */
bool
xreal_air_slam_guard_usable(struct xreal_air_slam_guard *g, int64_t now_ns, int64_t latest_ts);


/*
 *
 * Frame skipping while the tracker is behind.
 *
 */

/*!
 * Basalt queues every frame (10 deep, then the push blocks the camera thread), so a tracker short of
 * CPU turns a load spike into a third of a second of pose latency, and the guard into a loss. This
 * decides per stereo pair whether to leave it out instead: when @ref max_pending pairs pushed
 * earlier still have no pose (the tracker is busy with one and the next is queued), at most
 * @ref max_in_row pairs in a row (the tracker always gets some), and not while the newest pose is
 * older than the newest pushed pair by more than @ref ceiling_ns (the tracker is starting or
 * re-initialising, not behind).
 *
 * Counting pairs, not comparing the new pair's time with the newest pose: after a gap in the camera
 * stream (USB, the kernel dropping frames) the new pair is far newer than the newest pose although
 * the tracker has nothing left to do, and skipping it would only widen the gap.
 */
#define XREAL_AIR_BEHIND_RING 16

struct xreal_air_behind
{
	uint32_t max_pending; //!< 0: never skip.
	int64_t ceiling_ns;
	uint32_t max_in_row;

	int64_t pushed_ts[XREAL_AIR_BEHIND_RING]; //!< The last pairs pushed, a ring.
	uint32_t pushed_count;                    //!< Total pushed (index of the next slot).

	uint32_t in_row;      //!< Pairs skipped in a row so far.
	uint64_t skipped;     //!< Pairs skipped in total.
	uint32_t last_pending; //!< For logs.
};

void
xreal_air_behind_init(struct xreal_air_behind *b, uint32_t max_pending, int64_t ceiling_ns, uint32_t max_in_row);

/*!
 * Whether to skip the next pair. @p have_pose false when the tracker has no pose yet, else
 * @p newest_pose_ts is the newest one's timestamp.
 */
bool
xreal_air_behind_skip(struct xreal_air_behind *b, bool have_pose, int64_t newest_pose_ts);

//! A pair at @p pair_ts was pushed to the tracker.
void
xreal_air_behind_pushed(struct xreal_air_behind *b, int64_t pair_ts);

//! A new tracker: start counting afresh (the total stays).
void
xreal_air_behind_restart(struct xreal_air_behind *b);


/*
 *
 * Position history.
 *
 */

#define XREAL_AIR_POS_HISTORY_SIZE 128

struct xreal_air_pos_history
{
	int64_t ts[XREAL_AIR_POS_HISTORY_SIZE];
	struct xrt_vec3 pos[XREAL_AIR_POS_HISTORY_SIZE];
	uint32_t head; //!< Next write.
	uint32_t count;
	int64_t min_interval_ns; //!< Samples closer than this to the previous one are skipped.
};

void
xreal_air_pos_history_init(struct xreal_air_pos_history *h, int64_t min_interval_ns);

void
xreal_air_pos_history_push(struct xreal_air_pos_history *h, int64_t ts, const struct xrt_vec3 *pos);

/*!
 * The newest position at or before @p ts, false when the history is empty. When everything is
 * newer than @p ts, the oldest one.
 */
bool
xreal_air_pos_history_at(const struct xreal_air_pos_history *h, int64_t ts, struct xrt_vec3 *out_pos);

void
xreal_air_pos_history_clear(struct xreal_air_pos_history *h);

#ifdef __cplusplus
}
#endif
