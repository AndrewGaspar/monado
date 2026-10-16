// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Plane detection: the horizontal surface map (columns, peaks, regions, tracking, labels),
 *         the convex hull, and answering XR_EXT_plane_detection queries.
 * @author Andrew Gaspar
 */

#include "tracking/t_plane_map.h"
#include "tracking/t_plane_detections.h"

#include "math/m_api.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

using Catch::Approx;

static constexpr int64_t kSec = 1000 * 1000 * 1000;

//! Repeatable gaussian noise.
struct Rng
{
	uint64_t state = 0x1234abcd;

	double
	uniform()
	{
		state = state * 6364136223846793005ULL + 1442695040888963407ULL;
		return ((double)(state >> 11) + 0.5) / (double)(1ULL << 53);
	}
	double
	gauss(double sigma)
	{
		return std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * M_PI * uniform()) * sigma;
	}
};

//! Points on a horizontal rectangle [x0, x1] x [y0, y1] at height z, @p per_m2 of them.
static void
add_rect(std::vector<t_plane_map_point> &pts,
         Rng &rng,
         float x0,
         float x1,
         float y0,
         float y1,
         float z,
         int per_m2,
         float noise = 0.004f,
         float sigma = 0.005f)
{
	const int n = (int)((x1 - x0) * (y1 - y0) * (float)per_m2);
	for (int i = 0; i < n; i++) {
		t_plane_map_point p;
		p.position.x = x0 + (float)rng.uniform() * (x1 - x0);
		p.position.y = y0 + (float)rng.uniform() * (y1 - y0);
		p.position.z = z + (float)rng.gauss(noise);
		p.sigma_h = sigma;
		pts.push_back(p);
	}
}

static const xrt_vec3 kHead = {0, 0, 0};

static t_plane_map *
make_map(t_plane_map_config *cfg_out = nullptr)
{
	t_plane_map_config cfg;
	t_plane_map_default_config(&cfg);
	if (cfg_out != nullptr) {
		*cfg_out = cfg;
	}
	t_plane_map *m = nullptr;
	REQUIRE(t_plane_map_create(&cfg, &m) == 0);
	return m;
}

static t_plane_map *
make_map_with(const t_plane_map_config &cfg)
{
	t_plane_map *m = nullptr;
	REQUIRE(t_plane_map_create(&cfg, &m) == 0);
	return m;
}

static std::vector<t_plane>
planes_of(t_plane_map *m, bool reportable_only)
{
	std::vector<t_plane> v(64);
	v.resize(t_plane_map_get_planes(m, reportable_only, v.data(), (uint32_t)v.size()));
	return v;
}

//! A living room: floor 1.1 m below the head, a coffee table 0.55 m below it in front.
static std::vector<t_plane_map_point>
living_room(Rng &rng)
{
	std::vector<t_plane_map_point> pts;
	add_rect(pts, rng, -1.0f, 1.0f, 1.6f, 3.0f, -1.1f, 4000);  // Floor beyond the table.
	add_rect(pts, rng, -0.4f, 0.4f, 0.8f, 1.3f, -0.55f, 4000); // Table.
	return pts;
}


TEST_CASE("t_plane_convex_hull")
{
	SECTION("square with interior and collinear points")
	{
		const xrt_vec2 pts[] = {{0, 0},       {1, 0},    {1, 1},       {0, 1},
		                        {0.5f, 0.5f}, {0.5f, 0}, {0.2f, 0.7f}, {1, 0.5f}};
		xrt_vec2 out[8];
		const uint32_t n = t_plane_convex_hull(pts, 8, out, 8);
		REQUIRE(n == 4);
		// Counter-clockwise: positive signed area.
		double a = 0;
		for (uint32_t i = 0; i < n; i++) {
			a += (double)out[i].x * out[(i + 1) % n].y - (double)out[(i + 1) % n].x * out[i].y;
		}
		CHECK(0.5 * a == Approx(1.0));
	}

	SECTION("simplified to the limit, inside the original")
	{
		std::vector<xrt_vec2> circle;
		for (int i = 0; i < 100; i++) {
			const float t = (float)i / 100.0f * 2.0f * (float)M_PI;
			circle.push_back({std::cos(t), std::sin(t)});
		}
		xrt_vec2 out[16];
		const uint32_t n = t_plane_convex_hull(circle.data(), (uint32_t)circle.size(), out, 16);
		REQUIRE(n == 16);
		for (uint32_t i = 0; i < n; i++) {
			CHECK(std::hypot(out[i].x, out[i].y) == Approx(1.0f).margin(1e-5));
		}
	}

	SECTION("degenerate")
	{
		const xrt_vec2 pts[] = {{0, 0}, {0, 0}};
		xrt_vec2 out[4];
		CHECK(t_plane_convex_hull(pts, 2, out, 4) == 1);
		CHECK(t_plane_convex_hull(pts, 0, out, 4) == 0);
	}
}

TEST_CASE("t_plane_map finds a table and the floor")
{
	Rng rng;
	t_plane_map *m = make_map();
	const std::vector<t_plane_map_point> pts = living_room(rng);
	CHECK(t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f) == pts.size());

	// Not reportable until stable.
	t_plane_map_update(m, 1 * kSec);
	CHECK(planes_of(m, true).empty());
	const std::vector<t_plane> early = planes_of(m, false);
	REQUIRE(early.size() == 2);

	t_plane_map_update(m, 7 * kSec);
	const std::vector<t_plane> planes = planes_of(m, true);
	REQUIRE(planes.size() == 2);

	// Largest first: the floor (2 x 1.4 m), then the table (0.8 x 0.5 m).
	const t_plane &floor = planes[0];
	const t_plane &table = planes[1];
	CHECK(floor.height == Approx(-1.1f).margin(0.003f));
	CHECK(table.height == Approx(-0.55f).margin(0.003f));
	CHECK(floor.label == XRT_PLANE_DETECTOR_SEMANTIC_TYPE_FLOOR_EXT);
	CHECK(table.label == XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT);
	CHECK(table.area_m2 == Approx(0.4f).margin(0.08f));
	CHECK(floor.area_m2 == Approx(2.8f).margin(0.3f));

	// Ids survive updates.
	CHECK(floor.id == early[0].id);
	CHECK(table.id == early[1].id);

	// The table's oriented box and outline.
	const float ex = std::max(table.extents.x, table.extents.y);
	const float ey = std::min(table.extents.x, table.extents.y);
	CHECK(ex == Approx(0.8f).margin(0.1f));
	CHECK(ey == Approx(0.5f).margin(0.1f));
	CHECK(table.center.x == Approx(0.0f).margin(0.05f));
	CHECK(table.center.y == Approx(1.05f).margin(0.05f));
	CHECK(table.vertex_count >= 4);
	for (uint32_t i = 0; i < table.vertex_count; i++) {
		CHECK(std::fabs(table.vertices[i].x) <= 0.5f * table.extents.x + 1e-4f);
		CHECK(std::fabs(table.vertices[i].y) <= 0.5f * table.extents.y + 1e-4f);
	}

	t_plane_map_stats stats;
	t_plane_map_get_stats(m, &stats);
	CHECK(stats.floor_z == Approx(-1.1f).margin(0.005f));

	// A reset forgets everything, new ids.
	t_plane_map_reset(m);
	t_plane_map_update(m, 8 * kSec);
	CHECK(planes_of(m, false).empty());
	CHECK(t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f) == pts.size());
	t_plane_map_update(m, 9 * kSec);
	const std::vector<t_plane> again = planes_of(m, false);
	REQUIRE(again.size() == 2);
	CHECK(again[0].id > table.id);
	CHECK(again[1].id > table.id);

	t_plane_map_destroy(&m);
	CHECK(m == nullptr);
}

TEST_CASE("t_plane_map keeps a vanished plane for the grace time")
{
	Rng rng;
	t_plane_map *m = make_map();
	std::vector<t_plane_map_point> pts;
	add_rect(pts, rng, -0.4f, 0.4f, 0.8f, 1.3f, -0.55f, 4000);
	t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(m, 0);
	t_plane_map_update(m, 6 * kSec);
	REQUIRE(planes_of(m, true).size() == 1);
	const uint64_t id = planes_of(m, true)[0].id;

	// Something heavier 20 cm above takes the columns over: the table is no longer extracted.
	pts.clear();
	add_rect(pts, rng, -0.4f, 0.4f, 0.8f, 1.3f, -0.35f, 12000);
	t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(m, 7 * kSec);
	std::vector<t_plane> all = planes_of(m, false);
	REQUIRE(all.size() == 2);
	std::vector<t_plane> shown = planes_of(m, true);
	REQUIRE(shown.size() == 1);
	CHECK(shown[0].id == id); // Still reported during the grace time.
	CHECK(shown[0].height == Approx(-0.55f).margin(0.005f));

	t_plane_map_update(m, 10 * kSec);
	all = planes_of(m, false);
	REQUIRE(all.size() == 1);
	CHECK(all[0].id != id);
	CHECK(all[0].height == Approx(-0.35f).margin(0.005f));
	CHECK(planes_of(m, true).empty()); // New, not stable yet.

	t_plane_map_destroy(&m);
}

TEST_CASE("t_plane_map point filters")
{
	Rng rng;
	t_plane_map *m = make_map();
	std::vector<t_plane_map_point> pts;
	add_rect(pts, rng, -0.15f, 0.15f, 0.25f, 0.4f, -0.6f, 4000);             // Lap: under the head (but not near).
	add_rect(pts, rng, -0.2f, 0.2f, 1.0f, 1.2f, 0.8f, 2000);                 // Above the head (ceiling-ish).
	add_rect(pts, rng, -0.2f, 0.2f, 4.5f, 4.7f, -1.0f, 2000);                // Too far.
	add_rect(pts, rng, -0.2f, 0.2f, 1.0f, 1.2f, -0.6f, 2000, 0.004f, 0.05f); // Too uncertain.
	add_rect(pts, rng, -0.2f, 0.2f, 0.3f, 0.4f, -0.2f, 2000);                // Near (hands).
	CHECK(t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f) == 0);

	t_plane_map_update(m, 10 * kSec);
	CHECK(planes_of(m, false).empty());
	t_plane_map_destroy(&m);
}

TEST_CASE("t_plane_map ignores walls and noise")
{
	Rng rng;
	t_plane_map *m = make_map();
	std::vector<t_plane_map_point> pts;
	// A wall at y = 2: x in [-1, 1], z in [-1.1, 0.5].
	for (int i = 0; i < 40000; i++) {
		t_plane_map_point p;
		p.position = {(float)(rng.uniform() * 2.0 - 1.0), 2.0f + (float)rng.gauss(0.01),
		              (float)(-1.1 + rng.uniform() * 1.6)};
		p.sigma_h = 0.005f;
		pts.push_back(p);
	}
	// Scattered clutter.
	for (int i = 0; i < 20000; i++) {
		t_plane_map_point p;
		p.position = {(float)(rng.uniform() * 2.0 - 1.0), (float)(1.0 + rng.uniform()),
		              (float)(-1.1 + rng.uniform() * 1.0)};
		p.sigma_h = 0.005f;
		pts.push_back(p);
	}
	t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(m, 10 * kSec);
	CHECK(planes_of(m, false).empty());
	t_plane_map_destroy(&m);
}

TEST_CASE("t_plane_map merges patches at one height only when close")
{
	Rng rng;
	t_plane_map *m = make_map();
	std::vector<t_plane_map_point> pts;
	// Two floor patches 20 cm apart (more than the 2-cell growth gap), one 1.5 m away.
	add_rect(pts, rng, -1.0f, -0.3f, 1.5f, 2.2f, -1.1f, 4000);
	add_rect(pts, rng, -0.1f, 0.6f, 1.5f, 2.2f, -1.105f, 4000);
	add_rect(pts, rng, 2.0f, 2.6f, 1.5f, 2.2f, -1.1f, 4000);
	t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(m, 0);
	const std::vector<t_plane> planes = planes_of(m, false);
	REQUIRE(planes.size() == 2);
	CHECK(planes[0].area_m2 == Approx(0.98f).margin(0.15f));
	CHECK(planes[1].area_m2 == Approx(0.42f).margin(0.1f));
	t_plane_map_destroy(&m);
}

TEST_CASE("t_plane_map upper mode for glossy tops")
{
	// Per column: 35 % of the points on the top at -0.55, 65 % mirrored ~11 cm below it.
	Rng rng;
	std::vector<t_plane_map_point> pts;
	const float x0 = -0.4f, x1 = 0.4f, y0 = 0.8f, y1 = 1.3f;
	for (int i = 0; i < 6000; i++) {
		t_plane_map_point p;
		p.position.x = x0 + (float)rng.uniform() * (x1 - x0);
		p.position.y = y0 + (float)rng.uniform() * (y1 - y0);
		const bool top = rng.uniform() < 0.35;
		p.position.z = top ? -0.55f + (float)rng.gauss(0.004) : -0.66f + (float)rng.gauss(0.012);
		p.sigma_h = 0.005f;
		pts.push_back(p);
	}

	t_plane_map_config cfg;
	t_plane_map_default_config(&cfg);

	cfg.upper_mode = false;
	t_plane_map *plain = make_map_with(cfg);
	t_plane_map_add_points(plain, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(plain, 0);
	const std::vector<t_plane> a = planes_of(plain, false);
	REQUIRE(a.size() >= 1);
	CHECK(a[0].height < -0.6f); // The mirror image wins.

	cfg.upper_mode = true;
	t_plane_map *upper = make_map_with(cfg);
	t_plane_map_add_points(upper, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(upper, 0);
	const std::vector<t_plane> b = planes_of(upper, false);
	REQUIRE(b.size() >= 1);
	CHECK(b[0].height == Approx(-0.55f).margin(0.005f));

	t_plane_map_destroy(&plain);
	t_plane_map_destroy(&upper);
}

TEST_CASE("t_plane_map does not report narrow ridges")
{
	Rng rng;
	t_plane_map *m = make_map();
	std::vector<t_plane_map_point> pts;
	add_rect(pts, rng, -0.8f, 0.8f, 1.0f, 1.08f, -0.3f, 30000); // A sofa back's top edge, 8 cm wide.
	t_plane_map_add_points(m, &kHead, pts.data(), (uint32_t)pts.size(), 1.0f);
	t_plane_map_update(m, 0);
	t_plane_map_update(m, 6 * kSec);
	CHECK(planes_of(m, false).size() == 1);
	CHECK(planes_of(m, true).empty());
	t_plane_map_destroy(&m);
}


/*
 *
 * Queries.
 *
 */

static t_plane_location
location(uint64_t id, float x, float y, float z, float w, float h, enum xrt_plane_detector_semantic_type_ext label)
{
	t_plane_location l;
	std::memset(&l, 0, sizeof(l));
	l.id = id;
	l.pose.orientation = {-0.70710678f, 0, 0, 0.70710678f}; // +Z up.
	l.pose.position = {x, y, z};
	l.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                   XRT_SPACE_RELATION_POSITION_VALID_BIT);
	l.extents = {w, h};
	l.orientation = XRT_PLANE_DETECTOR_ORIENTATION_HORIZONTAL_UPWARD_EXT;
	l.semantic_type = label;
	return l;
}

static xrt_plane_detector_begin_info_ext
any_query()
{
	xrt_plane_detector_begin_info_ext q;
	std::memset(&q, 0, sizeof(q));
	q.max_planes = 16;
	q.bounding_box_pose.orientation.w = 1;
	q.bounding_box_extent = {100, 100, 100};
	return q;
}

TEST_CASE("t_plane_requests")
{
	t_plane_requests r;
	t_plane_requests_init(&r);
	xrt_plane_detector_begin_info_ext q = any_query();
	uint64_t a = 0, b = 0;
	REQUIRE(t_plane_requests_begin(&r, &q, &a) == XRT_SUCCESS);
	q.max_planes = 3;
	REQUIRE(t_plane_requests_begin(&r, &q, &b) == XRT_SUCCESS);
	CHECK(a != 0);
	CHECK(b != a);

	xrt_plane_detector_begin_info_ext got;
	REQUIRE(t_plane_requests_get(&r, b, &got));
	CHECK(got.max_planes == 3);
	CHECK(t_plane_requests_destroy(&r, a));
	CHECK_FALSE(t_plane_requests_destroy(&r, a));
	CHECK_FALSE(t_plane_requests_get(&r, a, &got));
	CHECK(t_plane_requests_get(&r, b, &got));

	for (int i = 0; i < T_PLANE_REQUESTS_MAX - 1; i++) {
		uint64_t id = 0;
		CHECK(t_plane_requests_begin(&r, &q, &id) == XRT_SUCCESS);
	}
	uint64_t id = 0;
	CHECK(t_plane_requests_begin(&r, &q, &id) == XRT_ERROR_ALLOCATION);
}

TEST_CASE("t_plane_detections_fill")
{
	const xrt_vec2 square[4] = {{-0.4f, -0.25f}, {0.4f, -0.25f}, {0.4f, 0.25f}, {-0.4f, 0.25f}};
	t_plane_location planes[3] = {
	    location(10, 0, -0.55f, -1.0f, 0.8f, 0.5f, XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT),
	    location(11, 0, -1.1f, -2.0f, 2.0f, 1.5f, XRT_PLANE_DETECTOR_SEMANTIC_TYPE_FLOOR_EXT),
	    location(12, 3, -0.3f, 0.0f, 0.4f, 0.4f, XRT_PLANE_DETECTOR_SEMANTIC_TYPE_UNDEFINED_EXT),
	};
	planes[0].vertex_count = 4;
	planes[0].vertices = square;

	xrt_plane_detections_ext out;
	std::memset(&out, 0, sizeof(out));

	SECTION("everything, largest first, no contour")
	{
		xrt_plane_detector_begin_info_ext q = any_query();
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		REQUIRE(out.location_count == 3);
		CHECK(out.locations[0].planeId == 11);
		CHECK(out.locations[1].planeId == 10);
		CHECK(out.locations[2].planeId == 12);
		CHECK(out.locations[1].polygon_buffer_count == 0);
		CHECK(out.locations[1].semantic_type == XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT);
		CHECK(out.locations[1].relation.pose.position.y == Approx(-0.55f));
		// Sized exactly, as the IPC server sends them.
		CHECK(out.location_size == 3);
		CHECK(out.polygon_info_size == 0);
		CHECK(out.vertex_size == 0);
	}

	SECTION("contour, max planes")
	{
		xrt_plane_detector_begin_info_ext q = any_query();
		q.detector_flags = XRT_PLANE_DETECTOR_FLAGS_CONTOUR_EXT;
		q.max_planes = 2;
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		REQUIRE(out.location_count == 2);
		CHECK(out.locations[1].planeId == 10);
		REQUIRE(out.locations[1].polygon_buffer_count == 1);
		const xrt_plane_polygon_info_ext &info = out.polygon_infos[out.polygon_info_start_index[1]];
		CHECK(info.vertex_count == 4);
		CHECK(out.vertices[info.vertices_start_index + 2].x == Approx(0.4f));
		CHECK(out.polygon_info_size == 1);
		CHECK(out.vertex_size == 4);
	}

	SECTION("semantic and orientation filters")
	{
		xrt_plane_detector_begin_info_ext q = any_query();
		q.semantic_type_count = 1;
		q.semantic_types[0] = XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT;
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		REQUIRE(out.location_count == 1);
		CHECK(out.locations[0].planeId == 10);

		q = any_query();
		q.orientation_count = 1;
		q.orientations[0] = XRT_PLANE_DETECTOR_ORIENTATION_VERTICAL_EXT;
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		CHECK(out.location_count == 0);

		q.orientation_count = 2;
		q.orientations[1] = XRT_PLANE_DETECTOR_ORIENTATION_HORIZONTAL_UPWARD_EXT;
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		CHECK(out.location_count == 3);
	}

	SECTION("min area uses the polygon")
	{
		xrt_plane_detector_begin_info_ext q = any_query();
		q.min_area = 0.3f;
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		REQUIRE(out.location_count == 2); // 3 m^2 floor and the 0.4 m^2 table, not the 0.16 m^2 one.
		CHECK(t_plane_location_area(&planes[0]) == Approx(0.4f));
	}

	SECTION("bounding box")
	{
		xrt_plane_detector_begin_info_ext q = any_query();
		q.bounding_box_pose.position = {0, -0.5f, -1.0f};
		q.bounding_box_extent = {1.0f, 0.4f, 1.0f};
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		REQUIRE(out.location_count == 1);
		CHECK(out.locations[0].planeId == 10);

		// A box only touching the table's outline still finds it.
		q.bounding_box_pose.position = {0.5f, -0.55f, -1.0f};
		q.bounding_box_extent = {0.25f, 0.1f, 0.6f};
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		REQUIRE(out.location_count == 1);

		// A zero box is no box.
		q.bounding_box_extent = {0, 0, 0};
		REQUIRE(t_plane_detections_fill(&q, planes, 3, &out) == XRT_SUCCESS);
		CHECK(out.location_count == 3);
	}

	SECTION("nothing")
	{
		xrt_plane_detector_begin_info_ext q = any_query();
		REQUIRE(t_plane_detections_fill(&q, planes, 0, &out) == XRT_SUCCESS);
		CHECK(out.location_count == 0);
	}

	xrt_plane_detections_ext_clear(&out);
}
