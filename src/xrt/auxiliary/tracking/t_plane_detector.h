// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Plane detection worker: stereo frames, sparse points and SLAM poses in, a plane snapshot out.
 *
 * A low priority thread (nice +10) that owns a @ref t_plane_map. Inputs never block:
 *
 *  - @ref t_plane_detector_push_stereo: stereo pairs, at most @ref t_plane_detector_config::rate_hz of
 *    them are kept (the newest one waits, an older one waiting is dropped). They go through a
 *    @ref t_plane_depth_source (dense SGBM at half resolution, @ref t_plane_stereo.h; swappable) and
 *    every pixel with depth becomes a point.
 *  - @ref t_plane_detector_push_points: sparse points in the IMU frame (e.g. the SLAM system's own
 *    landmarks), queued.
 *
 * Both are placed in the world with the SLAM pose at their timestamp, asked from the driver through
 * the @ref t_plane_pose_fn callback on the worker thread (never while a detector lock is held): not
 * there yet (retried for @ref t_plane_detector_config::pose_wait_ns), or not to be trusted (dropped).
 * A pose from a different SLAM world (world id) resets the map. Point height uncertainty:
 * sigma_h = |point z - camera z| * rel_sigma, rel_sigma being the depth source's confidence.
 *
 * Every @ref t_plane_detector_config::extract_interval_ns the map is updated and the reportable
 * planes published as a snapshot (@ref t_plane_detector_get_planes), tagged with its SLAM world and a
 * generation that is bumped on every reset.
 *
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"

#include "t_plane_depth.h"
#include "t_plane_map.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum t_plane_pose_status
{
	T_PLANE_POSE_READY,    //!< The pose is filled in.
	T_PLANE_POSE_NOT_YET,  //!< Ask again later.
	T_PLANE_POSE_UNUSABLE, //!< Drop the data (not tracking, old world, ...).
};

/*!
 * The SLAM pose of the IMU (body) in the world (z up, gravity aligned) at @p timestamp_ns, and the
 * id of that world. Called on the detector's thread.
 */
typedef enum t_plane_pose_status (*t_plane_pose_fn)(void *user,
                                                    int64_t timestamp_ns,
                                                    uint32_t *out_world_id,
                                                    struct xrt_pose *out_T_world_imu);

//! A sparse point for @ref t_plane_detector_push_points.
struct t_plane_body_point
{
	struct xrt_vec3 p_imu;
	float rel_sigma; //!< sigma_h = |point z - camera z| * rel_sigma.
};

struct t_plane_detector_config
{
	float rate_hz;               //!< Stereo pairs processed per second (default 4).
	int64_t extract_interval_ns; //!< Default 1 s.
	int64_t pose_wait_ns;        //!< Default 500 ms.
	float sparse_weight;         //!< Weight scale of sparse points (default 1).
	int nice;                    //!< Worker thread nice value (default 10).
	/*!
	 * Debugging: append every batch of map points to this file (NULL = off), for re-running the map
	 * offline with other settings (monado-cli plane-map-replay). Format, little endian, per record:
	 * int64 time_ns, float head[3], float weight_scale, uint32 count, count x float[4] (x y z sigma_h);
	 * count 0xffffffff marks a reset.
	 */
	const char *dump_path;
	struct t_plane_map_config map;
};

struct t_plane_detector_stats
{
	uint64_t pairs_pushed;
	uint64_t pairs_processed;
	uint64_t pairs_dropped; //!< Replaced while waiting, or no usable pose.
	uint64_t sparse_batches;
	uint64_t sparse_points;
	uint64_t resets;
	uint64_t extractions;
	uint32_t world_id;
	uint64_t generation;
	uint32_t planes; //!< Reportable ones in the last snapshot.

	// Per stage, mean and max milliseconds.
	float prepare_ms, prepare_max_ms;     //!< Depth source, e.g. SGBM's rectification.
	float match_ms, match_max_ms;         //!< Depth source, e.g. SGBM itself.
	float points_ms, points_max_ms;       //!< Depth image -> world points.
	float integrate_ms, integrate_max_ms; //!< World transform + map (dense).
	float sparse_ms, sparse_max_ms;
	float extract_ms, extract_max_ms;
	float dense_points_mean; //!< Points per pair passed to the map.

	struct t_plane_map_stats map;
};

struct t_plane_detector;

void
t_plane_detector_default_config(struct t_plane_detector_config *cfg);

/*!
 * @param depth The dense path's depth source (taken over, destroyed with the detector), NULL for
 *              sparse points only.
 * @return 0 on success.
 */
int
t_plane_detector_create(const struct t_plane_detector_config *cfg,
                        struct t_plane_depth_source *depth,
                        t_plane_pose_fn pose_fn,
                        void *user,
                        struct t_plane_detector **out);

//! Stops the thread.
void
t_plane_detector_destroy(struct t_plane_detector **d_ptr);

/*!
 * A stereo pair (L8), @p timestamp_ns as the SLAM poses are stamped. Keeps references, never blocks.
 */
void
t_plane_detector_push_stereo(struct t_plane_detector *d,
                             int64_t timestamp_ns,
                             struct xrt_frame *left,
                             struct xrt_frame *right);

//! Sparse points seen at @p timestamp_ns (copied), never blocks.
void
t_plane_detector_push_points(struct t_plane_detector *d,
                             int64_t timestamp_ns,
                             const struct t_plane_body_point *points,
                             uint32_t count);

//! Drop the map (the driver knows the world is gone).
void
t_plane_detector_reset(struct t_plane_detector *d);

/*!
 * The newest snapshot of planes (world frame, z up), largest first: the reportable ones, or with
 * @p reportable_only false every tracked one (for tools).
 *
 * @return How many were written to @p out (at most @p max).
 */
uint32_t
t_plane_detector_get_planes(struct t_plane_detector *d,
                            bool reportable_only,
                            uint32_t *out_world_id,
                            uint64_t *out_generation,
                            struct t_plane *out,
                            uint32_t max);

void
t_plane_detector_get_stats(struct t_plane_detector *d, struct t_plane_detector_stats *out);

#ifdef __cplusplus
}
#endif
