// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Answer @ref xrt_device plane detection calls (XR_EXT_plane_detection) from a list of planes.
 *
 * For a device that keeps ONE plane map and answers every detection (every session, every
 * detector) from it: @ref t_plane_requests remembers each detection's query (the begin info), and
 * @ref t_plane_detections_fill filters a plane list by it and lays the result out the way
 * @ref xrt_plane_detections_ext wants it.
 *
 * Plane convention (XR_EXT_plane_detection): the pose's +Z is the plane normal, @p extents and the
 * polygon vertices are in the pose's XY plane, the polygon counter-clockwise seen from +Z. Planes
 * are given in the device's tracking-origin space, which is what the state tracker expects (it
 * relates them to the base space through the device's space, see oxr_xrGetPlaneDetectionsEXT).
 *
 * Query filters: orientations and semantic types (each list empty = any), @p min_area (the polygon
 * area, else extents), @p max_planes (largest first), and the bounding box (the plane center or a
 * polygon vertex inside it; a box with a non-positive extent is taken as no box at all).
 *
 * Pure, no locking: the caller serialises.
 *
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_plane_detector.h"
#include "xrt/xrt_results.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Most concurrent detections tracked (one per XrPlaneDetectorEXT in flight).
#define T_PLANE_REQUESTS_MAX 64

//! A plane to report, in the device's tracking-origin space.
struct t_plane_location
{
	uint64_t id;
	struct xrt_pose pose; //!< +Z is the normal.
	enum xrt_space_relation_flags relation_flags;
	struct xrt_vec2 extents;
	enum xrt_plane_detector_orientation_ext orientation;
	enum xrt_plane_detector_semantic_type_ext semantic_type;
	uint32_t vertex_count;
	const struct xrt_vec2 *vertices; //!< Outline in the pose's XY plane, may be NULL if @p vertex_count is 0.
};

struct t_plane_requests
{
	uint64_t next_id;
	uint32_t count;
	uint64_t ids[T_PLANE_REQUESTS_MAX];
	struct xrt_plane_detector_begin_info_ext queries[T_PLANE_REQUESTS_MAX];
};

void
t_plane_requests_init(struct t_plane_requests *r);

/*!
 * Remember a new detection's query.
 *
 * @return XRT_ERROR_ALLOCATION when @ref T_PLANE_REQUESTS_MAX are in flight.
 */
xrt_result_t
t_plane_requests_begin(struct t_plane_requests *r,
                       const struct xrt_plane_detector_begin_info_ext *query,
                       uint64_t *out_id);

//! Forget a detection, false if unknown.
bool
t_plane_requests_destroy(struct t_plane_requests *r, uint64_t id);

//! The query of a detection, false if unknown.
bool
t_plane_requests_get(const struct t_plane_requests *r,
                     uint64_t id,
                     struct xrt_plane_detector_begin_info_ext *out_query);

//! The area of a plane: its polygon's if it has one, else its extents'.
float
t_plane_location_area(const struct t_plane_location *plane);

//! Whether @p plane passes the filters of @p query (all but max_planes).
bool
t_plane_location_matches(const struct xrt_plane_detector_begin_info_ext *query, const struct t_plane_location *plane);

/*!
 * Filter @p planes by @p query and write the result into @p out, growing its arrays as needed
 * (sizes left at exactly the counts when they had to grow, as the IPC server expects). Polygons are
 * only written for detectors created with the contour flag.
 */
xrt_result_t
t_plane_detections_fill(const struct xrt_plane_detector_begin_info_ext *query,
                        const struct t_plane_location *planes,
                        uint32_t count,
                        struct xrt_plane_detections_ext *out);

#ifdef __cplusplus
}
#endif
