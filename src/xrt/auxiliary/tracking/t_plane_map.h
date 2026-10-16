// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Horizontal surface map: world points in, stable horizontal planes out.
 *
 * Pure bookkeeping (no OpenCV, no threads, the caller serialises the calls), so other drivers can
 * feed it and it can be unit tested.
 *
 * The world is gravity aligned with **z up** (Basalt's VIO world). Points go into gravity-aligned
 * columns (@ref t_plane_map_config::cell_m, 5 cm) each holding a height histogram
 * (@ref t_plane_map_config::bin_m, 1 cm bins) weighted by the point's height uncertainty
 * (min(1, (sigma_ref / sigma_h)^2)). Extraction (@ref t_plane_map_update, ~1 Hz):
 *
 *  1. Per column, the height peak: the histogram mode (smoothed with a 1-bin gaussian) holding at
 *     least @ref t_plane_map_config::peak_fraction of the mass within +-20 cm in +-3.5 cm, and at
 *     least @ref t_plane_map_config::min_peak_weight. With @ref t_plane_map_config::upper_mode the
 *     highest such mode wins over a bigger one below it (glossy tops: their mirror images land below
 *     the surface), tested against the mass above it only.
 *  2. Region growing over neighbouring peaky columns (< 1.5 cm from the neighbour, < 4 cm from the
 *     seed, bridging gaps of 2 cells), then merging regions at the same height (+-1.5 cm) that are
 *     close to each other. The normal is gravity; the height is the weighted mean of the column
 *     centroids, the outline the convex hull of the cells.
 *  3. Tracking: a region overlapping a known plane at about the same height keeps its id. A plane
 *     is reportable once it has existed for @ref t_plane_map_config::stable_ns with enough cells, a
 *     minimum width and a small height spread, and stays so unless it shrinks under half that size; one not seen again is dropped after @ref t_plane_map_config::grace_ns.
 *  4. Labels: the lowest large plane well below the head (and others at its height) is FLOOR; one
 *     0.25..1.25 m above the floor, or 0.25..0.8 m below the head within reach, is PLATFORM
 *     (sticky, but only while it is at least 0.25 m below the head).
 *
 * Point filters (@ref t_plane_map_add_points): nearer than @ref t_plane_map_config::near_m to the
 * head (hands), under the head within @ref t_plane_map_config::lap_radius_m and more than
 * @ref t_plane_map_config::lap_below_m below it (lap, laptop), more than
 * @ref t_plane_map_config::max_above_head_m above the head (ceiling), farther than
 * @ref t_plane_map_config::max_range_m, or a height uncertainty above
 * @ref t_plane_map_config::max_sigma_h_m.
 *
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_plane_detector.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Most hull vertices a plane carries (the hull is simplified down to this).
#define T_PLANE_MAX_VERTICES 32

struct t_plane_map;

struct t_plane_map_config
{
	// Grid.
	float cell_m; //!< Column size (default 0.05).
	float bin_m;  //!< Height bin (default 0.01).
	float z_min;  //!< Lowest world height mapped (default -3).
	float z_max;  //!< Highest world height mapped (default 2).

	// Point filters, see the file comment.
	float near_m;           //!< Default 0.6.
	float lap_radius_m;     //!< Default 0.45.
	float lap_below_m;      //!< Default 0.25.
	float max_above_head_m; //!< Default 0.6.
	float max_range_m;      //!< Default 4.
	float max_sigma_h_m;    //!< Default 0.03.
	float sigma_ref_m;      //!< Weight 1 at or below this height sigma (default 0.01).

	// Column peaks.
	float min_peak_weight; //!< Default 3.
	float peak_fraction;   //!< Default 0.5.
	int peak_half_bins;    //!< Default 3 (7 bins).
	int wide_half_bins;    //!< Default 20.
	bool upper_mode;       //!< Default false.
	float upper_fraction;  //!< An upper mode needs this much of the biggest one's mass (default 0.35).
	float upper_max_m;     //!< ... and may be at most this far above it (default 0.3).

	// Regions.
	float grow_step_m;  //!< Default 0.015.
	float grow_total_m; //!< Default 0.04 (8 cm chained a coffee table to the blanket 2.5 cm above it).
	int grow_gap_cells; //!< Default 2.
	int min_cells;      //!< A region needs this many columns (default 8).
	float merge_dh_m;   //!< Default 0.015.
	float merge_gap_m;  //!< Default 0.3.

	// Tracking and reporting.
	float match_dh_m;     //!< Default 0.04.
	int64_t stable_ns;    //!< Default 5 s.
	int64_t grace_ns;     //!< Default 3 s.
	int min_report_cells; //!< Default 20 (0.05 m^2).
	float min_width_m;    //!< Narrower planes (oriented box) are not reported (default 0.15).
	float max_spread_m;   //!< Planes whose column heights spread more are not reported (default 0.035).

	// Labels.
	float floor_min_area_m2;          //!< Default 0.1.
	float floor_min_below_head_m;     //!< Default 0.8.
	float platform_above_floor_min_m; //!< Default 0.25.
	float platform_above_floor_max_m; //!< Default 1.25.
	float platform_below_head_min_m;  //!< Default 0.25.
	float platform_below_head_max_m;  //!< Default 0.8.
	float platform_reach_m;           //!< Default 1.5.
};

//! A point for @ref t_plane_map_add_points.
struct t_plane_map_point
{
	struct xrt_vec3 position; //!< World, z up.
	float sigma_h;            //!< Height uncertainty, meters.
};

/*!
 * A plane, world frame (z up). Its own frame: origin at (center, height), +z up, +x at @ref yaw
 * from the world x axis; @ref extents and @ref vertices are in that frame.
 */
struct t_plane
{
	uint64_t id; //!< Never reused within a map's lifetime, also not across resets.
	enum xrt_plane_detector_semantic_type_ext label;
	float height;
	struct xrt_vec2 center;  //!< Oriented box center, world x y.
	float yaw;               //!< Radians about +z.
	struct xrt_vec2 extents; //!< Oriented box size along the plane's x and y.
	float area_m2;           //!< Cells times cell area.
	uint32_t cell_count;
	float spread_m; //!< Weighted std of the column heights around @ref height.
	int64_t first_seen_ns;
	int64_t last_seen_ns;
	bool reportable;
	uint32_t vertex_count;
	struct xrt_vec2 vertices[T_PLANE_MAX_VERTICES]; //!< Convex outline, counter-clockwise about +z.
};

struct t_plane_map_stats
{
	uint64_t points_in;
	uint64_t points_used;
	uint32_t columns;
	uint32_t peaky_columns; //!< At the last update.
	uint32_t regions;       //!< At the last update.
	uint32_t tracks;
	float head_z;  //!< Smoothed head height used for the labels.
	float floor_z; //!< NAN while unknown.
};

void
t_plane_map_default_config(struct t_plane_map_config *cfg);

int
t_plane_map_create(const struct t_plane_map_config *cfg, struct t_plane_map **out_map);

void
t_plane_map_destroy(struct t_plane_map **map_ptr);

//! Forget everything (a new world). Plane ids keep counting up.
void
t_plane_map_reset(struct t_plane_map *map);

/*!
 * Integrate points seen from @p head (world).
 *
 * @param weight_scale Multiplies every point's weight (e.g. to balance sparse against dense).
 * @return How many points passed the filters.
 */
uint32_t
t_plane_map_add_points(struct t_plane_map *map,
                       const struct xrt_vec3 *head,
                       const struct t_plane_map_point *points,
                       uint32_t count,
                       float weight_scale);

//! Extract regions, track and label planes. @p now_ns drives stability and expiry.
void
t_plane_map_update(struct t_plane_map *map, int64_t now_ns);

/*!
 * The tracked planes, largest first.
 *
 * @return How many were written (at most @p max).
 */
uint32_t
t_plane_map_get_planes(const struct t_plane_map *map, bool reportable_only, struct t_plane *out, uint32_t max);

void
t_plane_map_get_stats(const struct t_plane_map *map, struct t_plane_map_stats *out);


/*
 *
 * Geometry helpers, exposed for tests.
 *
 */

/*!
 * Convex hull (Andrew's monotone chain) of @p count points into @p out (counter-clockwise, no
 * collinear points), simplified to at most @p max vertices by repeatedly dropping the vertex
 * spanning the smallest triangle (the result stays inside the hull).
 *
 * @return The vertex count.
 */
uint32_t
t_plane_convex_hull(const struct xrt_vec2 *points, uint32_t count, struct xrt_vec2 *out, uint32_t max);

#ifdef __cplusplus
}
#endif
