// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Answer @ref xrt_device plane detection calls (XR_EXT_plane_detection) from a list of planes.
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#include "t_plane_detections.h"

#include "math/m_api.h"
#include "util/u_misc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void
t_plane_requests_init(struct t_plane_requests *r)
{
	memset(r, 0, sizeof(*r));
	r->next_id = 1;
}

xrt_result_t
t_plane_requests_begin(struct t_plane_requests *r,
                       const struct xrt_plane_detector_begin_info_ext *query,
                       uint64_t *out_id)
{
	if (r->count >= T_PLANE_REQUESTS_MAX) {
		return XRT_ERROR_ALLOCATION;
	}
	const uint64_t id = r->next_id++;
	r->ids[r->count] = id;
	r->queries[r->count] = *query;
	r->count++;
	*out_id = id;
	return XRT_SUCCESS;
}

bool
t_plane_requests_destroy(struct t_plane_requests *r, uint64_t id)
{
	for (uint32_t i = 0; i < r->count; i++) {
		if (r->ids[i] != id) {
			continue;
		}
		r->count--;
		r->ids[i] = r->ids[r->count];
		r->queries[i] = r->queries[r->count];
		return true;
	}
	return false;
}

bool
t_plane_requests_get(const struct t_plane_requests *r, uint64_t id, struct xrt_plane_detector_begin_info_ext *out_query)
{
	for (uint32_t i = 0; i < r->count; i++) {
		if (r->ids[i] == id) {
			*out_query = r->queries[i];
			return true;
		}
	}
	return false;
}

float
t_plane_location_area(const struct t_plane_location *plane)
{
	if (plane->vertex_count >= 3 && plane->vertices != NULL) {
		double a = 0;
		for (uint32_t i = 0; i < plane->vertex_count; i++) {
			const struct xrt_vec2 *p = &plane->vertices[i];
			const struct xrt_vec2 *q = &plane->vertices[(i + 1) % plane->vertex_count];
			a += (double)p->x * q->y - (double)q->x * p->y;
		}
		return (float)fabs(0.5 * a);
	}
	return plane->extents.x * plane->extents.y;
}

static bool
in_box(const struct xrt_pose *box_pose, const struct xrt_vec3 *half, const struct xrt_vec3 *p)
{
	struct xrt_pose inv;
	math_pose_invert(box_pose, &inv);
	struct xrt_vec3 l;
	math_pose_transform_point(&inv, p, &l);
	return fabsf(l.x) <= half->x && fabsf(l.y) <= half->y && fabsf(l.z) <= half->z;
}

bool
t_plane_location_matches(const struct xrt_plane_detector_begin_info_ext *query, const struct t_plane_location *plane)
{
	if (query->orientation_count > 0) {
		bool found = false;
		for (uint32_t i = 0; i < query->orientation_count && i < XRT_MAX_PLANE_ORIENTATIONS_EXT; i++) {
			found = found || query->orientations[i] == plane->orientation;
		}
		if (!found) {
			return false;
		}
	}

	if (query->semantic_type_count > 0) {
		bool found = false;
		for (uint32_t i = 0; i < query->semantic_type_count && i < XRT_MAX_PLANE_SEMANTIC_TYPE_EXT; i++) {
			found = found || query->semantic_types[i] == plane->semantic_type;
		}
		if (!found) {
			return false;
		}
	}

	if (t_plane_location_area(plane) < query->min_area) {
		return false;
	}

	const struct xrt_vec3 half = {
	    0.5f * query->bounding_box_extent.x,
	    0.5f * query->bounding_box_extent.y,
	    0.5f * query->bounding_box_extent.z,
	};
	if (half.x <= 0.0f || half.y <= 0.0f || half.z <= 0.0f) {
		return true; // No box.
	}
	if (in_box(&query->bounding_box_pose, &half, &plane->pose.position)) {
		return true;
	}
	for (uint32_t i = 0; i < plane->vertex_count && plane->vertices != NULL; i++) {
		const struct xrt_vec3 local = {plane->vertices[i].x, plane->vertices[i].y, 0.0f};
		struct xrt_vec3 p;
		math_pose_transform_point(&plane->pose, &local, &p);
		if (in_box(&query->bounding_box_pose, &half, &p)) {
			return true;
		}
	}
	return false;
}

struct ranked
{
	uint32_t index;
	float area;
};

static int
by_area_desc(const void *a, const void *b)
{
	const float x = ((const struct ranked *)a)->area;
	const float y = ((const struct ranked *)b)->area;
	return x < y ? 1 : (x > y ? -1 : 0);
}

xrt_result_t
t_plane_detections_fill(const struct xrt_plane_detector_begin_info_ext *query,
                        const struct t_plane_location *planes,
                        uint32_t count,
                        struct xrt_plane_detections_ext *out)
{
	const bool contour = (query->detector_flags & XRT_PLANE_DETECTOR_FLAGS_CONTOUR_EXT) != 0;

	struct ranked *picked = count > 0 ? U_TYPED_ARRAY_CALLOC(struct ranked, count) : NULL;
	if (count > 0 && picked == NULL) {
		return XRT_ERROR_ALLOCATION;
	}
	uint32_t n = 0;
	for (uint32_t i = 0; i < count; i++) {
		if (t_plane_location_matches(query, &planes[i])) {
			picked[n].index = i;
			picked[n].area = t_plane_location_area(&planes[i]);
			n++;
		}
	}
	if (n > 1) {
		qsort(picked, n, sizeof(*picked), by_area_desc);
	}
	if (query->max_planes > 0 && n > query->max_planes) {
		n = query->max_planes;
	}

	uint32_t polygon_count = 0, vertex_count = 0;
	for (uint32_t i = 0; i < n && contour; i++) {
		const struct t_plane_location *p = &planes[picked[i].index];
		if (p->vertex_count >= 3 && p->vertices != NULL) {
			polygon_count++;
			vertex_count += p->vertex_count;
		}
	}

	if (out->location_size < n) {
		U_ARRAY_REALLOC_OR_FREE(out->locations, struct xrt_plane_detector_location_ext, n);
		U_ARRAY_REALLOC_OR_FREE(out->polygon_info_start_index, uint32_t, n);
		out->location_size = n;
	}
	if (out->polygon_info_size < polygon_count) {
		U_ARRAY_REALLOC_OR_FREE(out->polygon_infos, struct xrt_plane_polygon_info_ext, polygon_count);
		out->polygon_info_size = polygon_count;
	}
	if (out->vertex_size < vertex_count) {
		U_ARRAY_REALLOC_OR_FREE(out->vertices, struct xrt_vec2, vertex_count);
		out->vertex_size = vertex_count;
	}
	if ((n > 0 && (out->locations == NULL || out->polygon_info_start_index == NULL)) ||
	    (polygon_count > 0 && out->polygon_infos == NULL) || (vertex_count > 0 && out->vertices == NULL)) {
		free(picked);
		xrt_plane_detections_ext_clear(out);
		return XRT_ERROR_ALLOCATION;
	}

	uint32_t poly = 0, vert = 0;
	for (uint32_t i = 0; i < n; i++) {
		const struct t_plane_location *p = &planes[picked[i].index];
		struct xrt_plane_detector_location_ext *l = &out->locations[i];
		memset(l, 0, sizeof(*l));
		l->planeId = p->id;
		l->relation.pose = p->pose;
		l->relation.relation_flags = p->relation_flags;
		l->extents = p->extents;
		l->orientation = p->orientation;
		l->semantic_type = p->semantic_type;
		out->polygon_info_start_index[i] = poly;

		if (contour && p->vertex_count >= 3 && p->vertices != NULL) {
			l->polygon_buffer_count = 1;
			out->polygon_infos[poly].vertex_count = p->vertex_count;
			out->polygon_infos[poly].vertices_start_index = vert;
			memcpy(&out->vertices[vert], p->vertices, sizeof(struct xrt_vec2) * p->vertex_count);
			poly++;
			vert += p->vertex_count;
		}
	}
	out->location_count = n;

	free(picked);
	return XRT_SUCCESS;
}
