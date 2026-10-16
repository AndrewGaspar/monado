// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Re-run the plane map on recorded input points (XREAL_AIR_PLANES_DUMP) with other settings.
 *
 * The points are the map's input as integrated live or in xreal-slam-replay (world, Z up, with their
 * height uncertainty), so map and extraction settings can be tried in seconds. Extraction runs every
 * second of recorded time, as in the detector.
 *
 * Usage: monado-cli plane-map-replay <dump> [key=value ...], keys are the t_plane_map_config fields
 * (stable_ns as stable_s, grace_ns as grace_s; booleans 0/1); all=1 lists every tracked plane, not only
 * the reportable ones; until=<s> stops there; track=<x>,<y> follows the plane under that world point at every update
 * (id, height, area) and sums up how often its id changed and how much its height moved.
 *
 * @author Andrew Gaspar
 */

#include "tracking/t_plane_map.h"

#include "cli_common.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define P(...) fprintf(stderr, __VA_ARGS__)

enum field_type
{
	F_FLOAT,
	F_INT,
	F_BOOL,
	F_SECONDS,
};

struct field
{
	const char *name;
	size_t offset;
	enum field_type type;
};

#define FIELD(name, type) {#name, offsetof(struct t_plane_map_config, name), type}

static const struct field fields[] = {
    FIELD(cell_m, F_FLOAT),
    FIELD(bin_m, F_FLOAT),
    FIELD(near_m, F_FLOAT),
    FIELD(lap_radius_m, F_FLOAT),
    FIELD(lap_below_m, F_FLOAT),
    FIELD(max_above_head_m, F_FLOAT),
    FIELD(max_range_m, F_FLOAT),
    FIELD(max_sigma_h_m, F_FLOAT),
    FIELD(sigma_ref_m, F_FLOAT),
    FIELD(min_peak_weight, F_FLOAT),
    FIELD(peak_fraction, F_FLOAT),
    FIELD(peak_half_bins, F_INT),
    FIELD(wide_half_bins, F_INT),
    FIELD(upper_mode, F_BOOL),
    FIELD(upper_fraction, F_FLOAT),
    FIELD(upper_max_m, F_FLOAT),
    FIELD(grow_step_m, F_FLOAT),
    FIELD(grow_total_m, F_FLOAT),
    FIELD(grow_gap_cells, F_INT),
    FIELD(min_cells, F_INT),
    FIELD(merge_dh_m, F_FLOAT),
    FIELD(merge_gap_m, F_FLOAT),
    FIELD(match_dh_m, F_FLOAT),
    {"stable_s", offsetof(struct t_plane_map_config, stable_ns), F_SECONDS},
    {"grace_s", offsetof(struct t_plane_map_config, grace_ns), F_SECONDS},
    FIELD(min_report_cells, F_INT),
    FIELD(min_width_m, F_FLOAT),
    FIELD(max_spread_m, F_FLOAT),
    FIELD(floor_min_area_m2, F_FLOAT),
    FIELD(floor_min_below_head_m, F_FLOAT),
    FIELD(platform_above_floor_min_m, F_FLOAT),
    FIELD(platform_above_floor_max_m, F_FLOAT),
    FIELD(platform_below_head_min_m, F_FLOAT),
    FIELD(platform_below_head_max_m, F_FLOAT),
    FIELD(platform_reach_m, F_FLOAT),
};

static bool
set_field(struct t_plane_map_config *cfg, const char *arg)
{
	const char *eq = strchr(arg, '=');
	if (eq == NULL) {
		return false;
	}
	for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
		if (strlen(fields[i].name) != (size_t)(eq - arg) ||
		    strncmp(fields[i].name, arg, (size_t)(eq - arg)) != 0) {
			continue;
		}
		char *ptr = (char *)cfg + fields[i].offset;
		const double v = atof(eq + 1);
		switch (fields[i].type) {
		case F_FLOAT: *(float *)ptr = (float)v; break;
		case F_INT: *(int *)ptr = (int)v; break;
		case F_BOOL: *(bool *)ptr = v != 0; break;
		case F_SECONDS: *(int64_t *)ptr = (int64_t)(v * 1e9); break;
		}
		return true;
	}
	return false;
}

//! The plane (any) whose outline holds world (x, y), the largest one; NULL if none.
static const struct t_plane *
plane_at(const struct t_plane *planes, uint32_t n, float x, float y)
{
	for (uint32_t i = 0; i < n; i++) {
		const struct t_plane *p = &planes[i];
		const float dx = x - p->center.x, dy = y - p->center.y;
		const float c = cosf(p->yaw), s = sinf(p->yaw);
		const float u = c * dx + s * dy, v = -s * dx + c * dy;
		bool inside = p->vertex_count >= 3;
		for (uint32_t k = 0; k < p->vertex_count && inside; k++) {
			const struct xrt_vec2 *a = &p->vertices[k];
			const struct xrt_vec2 *b = &p->vertices[(k + 1) % p->vertex_count];
			inside = (b->x - a->x) * (v - a->y) - (b->y - a->y) * (u - a->x) >= 0.0f; // Left of a CCW edge.
		}
		if (inside) {
			return p;
		}
	}
	return NULL;
}

struct trace
{
	bool on;
	float x, y;
	uint64_t last_id;
	uint32_t id_changes, updates, seen, reportable;
	double first_reportable_s;
	double sum, sum2; //!< Heights of reportable sightings after 30 s.
	uint32_t count;
};

static void
trace_update(struct t_plane_map *map, struct trace *tr, double t_s)
{
	if (!tr->on) {
		return;
	}
	struct t_plane planes[128];
	const uint32_t n = t_plane_map_get_planes(map, false, planes, 128);
	const struct t_plane *p = plane_at(planes, n, tr->x, tr->y);
	tr->updates++;
	if (p == NULL) {
		P("  trace %5.1f s: -\n", t_s);
		return;
	}
	tr->seen++;
	if (tr->last_id != 0 && p->id != tr->last_id) {
		tr->id_changes++;
	}
	tr->last_id = p->id;
	if (p->reportable) {
		if (tr->reportable++ == 0) {
			tr->first_reportable_s = t_s;
		}
		if (t_s >= 30.0) {
			tr->sum += p->height;
			tr->sum2 += (double)p->height * p->height;
			tr->count++;
		}
	}
	P("  trace %5.1f s: #%-4" PRIu64 " %c h %+.3f area %.2f\n", t_s, p->id, p->reportable ? '*' : ' ', p->height,
	  p->area_m2);
}

static const char *
label_str(enum xrt_plane_detector_semantic_type_ext l)
{
	switch (l) {
	case XRT_PLANE_DETECTOR_SEMANTIC_TYPE_FLOOR_EXT: return "FLOOR";
	case XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT: return "PLATFORM";
	default: return "-";
	}
}

int
cli_cmd_plane_map_replay(int argc, const char **argv)
{
	if (argc < 3) {
		P("Re-run the plane map on points dumped with XREAL_AIR_PLANES_DUMP.\n");
		P("Usage: %s %s <dump> [key=value ...]\n", argv[0], argv[1]);
		return EXIT_FAILURE;
	}

	struct t_plane_map_config cfg;
	t_plane_map_default_config(&cfg);
	bool all = false;
	double until_s = INFINITY;
	struct trace tr = {0};
	for (int i = 3; i < argc; i++) {
		if (strcmp(argv[i], "all=1") == 0) {
			all = true;
			continue;
		}
		if (sscanf(argv[i], "until=%lf", &until_s) == 1) {
			continue;
		}
		if (sscanf(argv[i], "track=%f,%f", &tr.x, &tr.y) == 2) {
			tr.on = true;
			continue;
		}
		if (!set_field(&cfg, argv[i])) {
			P("Unknown setting '%s'\n", argv[i]);
			return EXIT_FAILURE;
		}
	}

	FILE *f = fopen(argv[2], "rb");
	if (f == NULL) {
		P("Could not open '%s'\n", argv[2]);
		return EXIT_FAILURE;
	}

	struct t_plane_map *map = NULL;
	if (t_plane_map_create(&cfg, &map) != 0) {
		fclose(f);
		return EXIT_FAILURE;
	}

	struct t_plane_map_point *pts = NULL;
	uint32_t cap = 0;
	int64_t first = 0, next_update = 0, t = 0;
	uint64_t batches = 0;
	while (true) {
		float h[4];
		uint32_t n;
		if (fread(&t, sizeof(t), 1, f) != 1 || fread(h, sizeof(h), 1, f) != 1 ||
		    fread(&n, sizeof(n), 1, f) != 1) {
			break;
		}
		if (first == 0) {
			first = t;
			next_update = t + 1000000000;
		}
		if ((double)(t - first) / 1e9 > until_s) {
			break;
		}
		while (t >= next_update) {
			t_plane_map_update(map, next_update);
			trace_update(map, &tr, (double)(next_update - first) / 1e9);
			next_update += 1000000000;
		}
		if (n == 0xffffffffu) {
			P("Reset at %.1f s\n", (double)(t - first) / 1e9);
			t_plane_map_reset(map);
			continue;
		}
		if (n > cap) {
			cap = n;
			pts = realloc(pts, sizeof(*pts) * cap);
		}
		for (uint32_t i = 0; i < n; i++) {
			float q[4];
			if (fread(q, sizeof(q), 1, f) != 1) {
				n = i;
				break;
			}
			pts[i].position = (struct xrt_vec3){q[0], q[1], q[2]};
			pts[i].sigma_h = q[3];
		}
		const struct xrt_vec3 head = {h[0], h[1], h[2]};
		t_plane_map_add_points(map, &head, pts, n, h[3]);
		batches++;
	}
	fclose(f);
	free(pts);
	t_plane_map_update(map, next_update);

	struct t_plane planes[128];
	const uint32_t n = t_plane_map_get_planes(map, !all, planes, 128);
	struct t_plane_map_stats st;
	t_plane_map_get_stats(map, &st);
	P("%" PRIu64 " batches over %.1f s, %" PRIu64 " / %" PRIu64
	  " points used, %u columns (%u peaky), %u regions, "
	  "head z %+.3f, floor %+.3f; %u %s planes:\n",
	  batches, (double)(t - first) / 1e9, st.points_used, st.points_in, st.columns, st.peaky_columns, st.regions,
	  st.head_z, st.floor_z, n, all ? "tracked" : "reportable");
	for (uint32_t i = 0; i < n; i++) {
		const struct t_plane *p = &planes[i];
		P("  %c #%-4" PRIu64
		  " %-8s h %+.3f  area %.2f m2 (%3u cells)  center (%+.2f, %+.2f)  %.2f x %.2f m  spread "
		  "%2.0f mm  age %3.0f s\n",
		  p->reportable ? '*' : ' ', p->id, label_str(p->label), p->height, p->area_m2, p->cell_count,
		  p->center.x, p->center.y, p->extents.x, p->extents.y, p->spread_m * 1000.0f,
		  (double)(p->last_seen_ns - p->first_seen_ns) / 1e9);
	}

	if (tr.on) {
		const double mean = tr.count > 0 ? tr.sum / tr.count : NAN;
		const double sd = tr.count > 1 ? sqrt(fmax(0.0, tr.sum2 / tr.count - mean * mean)) : NAN;
		P("Trace (%.2f, %.2f): seen %u / %u updates, reportable %u (first at %.0f s), %u id changes, height "
		  "after "
		  "30 s %+.3f std %.1f mm\n",
		  tr.x, tr.y, tr.seen, tr.updates, tr.reportable, tr.first_reportable_s, tr.id_changes, mean,
		  sd * 1000.0);
	}

	t_plane_map_destroy(&map);
	return EXIT_SUCCESS;
}
