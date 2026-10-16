// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Horizontal surface map: world points in, stable horizontal planes out.
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#include "t_plane_map.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

using Key = int64_t;

inline Key
make_key(int32_t ix, int32_t iy)
{
	return (int64_t)((uint64_t)(uint32_t)ix << 32 | (uint64_t)(uint32_t)iy);
}

inline int32_t
key_x(Key k)
{
	return (int32_t)(uint32_t)((uint64_t)k >> 32);
}

inline int32_t
key_y(Key k)
{
	return (int32_t)(uint32_t)((uint64_t)k & 0xffffffffu);
}

//! One gravity-aligned column: a height histogram over [base, base + bins.size()) global bins.
struct Column
{
	int32_t base = 0;
	std::vector<float> bins;

	void
	add(int32_t bin, float w)
	{
		if (bins.empty()) {
			base = bin;
			bins.push_back(w);
			return;
		}
		if (bin < base) {
			bins.insert(bins.begin(), (size_t)(base - bin), 0.0f);
			base = bin;
		} else if (bin >= base + (int32_t)bins.size()) {
			bins.resize((size_t)(bin - base + 1), 0.0f);
		}
		bins[(size_t)(bin - base)] += w;
	}
};

//! What a column's histogram says.
struct ColumnPeak
{
	float level;    //!< Center of the peak bin (quantised), for region growing.
	float centroid; //!< Weighted mean height within the peak window.
	float weight;   //!< Mass within the peak window.
};

struct Region
{
	std::vector<Key> cells;
	std::vector<float> heights; //!< Column centroids.
	std::vector<float> weights;
	float height = 0;
	float weight = 0;
	float spread = 0;
	int32_t min_x = 0, max_x = 0, min_y = 0, max_y = 0;

	void
	finish()
	{
		double sw = 0, swh = 0;
		min_x = min_y = INT32_MAX;
		max_x = max_y = INT32_MIN;
		for (size_t i = 0; i < cells.size(); i++) {
			sw += weights[i];
			swh += weights[i] * heights[i];
			min_x = std::min(min_x, key_x(cells[i]));
			max_x = std::max(max_x, key_x(cells[i]));
			min_y = std::min(min_y, key_y(cells[i]));
			max_y = std::max(max_y, key_y(cells[i]));
		}
		weight = (float)sw;
		height = sw > 0 ? (float)(swh / sw) : 0.0f;
		double sv = 0;
		for (size_t i = 0; i < cells.size(); i++) {
			const double d = heights[i] - height;
			sv += weights[i] * d * d;
		}
		spread = sw > 0 ? (float)std::sqrt(sv / sw) : 0.0f;
	}
};

struct Track
{
	t_plane plane;
	std::unordered_set<Key> cells;
	bool platform = false; //!< Sticky.
	bool stable = false;   //!< Has met the reporting criteria once.
	bool matched = false;  //!< At the current update.
};

} // namespace

struct t_plane_map
{
	t_plane_map_config cfg;
	std::unordered_map<Key, Column> columns;
	std::vector<Track> tracks;
	uint64_t next_id = 1;

	bool have_head = false;
	xrt_vec3 head = {};
	float head_z = 0;
	float floor_z = NAN;

	t_plane_map_stats stats = {};
};


/*
 *
 * Geometry.
 *
 */

static double
cross(const xrt_vec2 &o, const xrt_vec2 &a, const xrt_vec2 &b)
{
	return ((double)a.x - o.x) * ((double)b.y - o.y) - ((double)a.y - o.y) * ((double)b.x - o.x);
}

extern "C" uint32_t
t_plane_convex_hull(const struct xrt_vec2 *points, uint32_t count, struct xrt_vec2 *out, uint32_t max)
{
	if (count == 0 || max == 0) {
		return 0;
	}

	std::vector<xrt_vec2> p(points, points + count);
	std::sort(p.begin(), p.end(),
	          [](const xrt_vec2 &a, const xrt_vec2 &b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
	p.erase(std::unique(p.begin(), p.end(),
	                    [](const xrt_vec2 &a, const xrt_vec2 &b) { return a.x == b.x && a.y == b.y; }),
	        p.end());
	if (p.size() < 3) {
		const uint32_t n = std::min((uint32_t)p.size(), max);
		std::copy(p.begin(), p.begin() + n, out);
		return n;
	}

	std::vector<xrt_vec2> h(2 * p.size());
	size_t k = 0;
	for (size_t i = 0; i < p.size(); i++) {
		while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) {
			k--;
		}
		h[k++] = p[i];
	}
	for (size_t i = p.size() - 1, t = k + 1; i > 0; i--) {
		while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0) {
			k--;
		}
		h[k++] = p[i - 1];
	}
	h.resize(k - 1); // The last point is the first one again.

	// Drop the vertex spanning the smallest triangle until it fits.
	while (h.size() > max && h.size() > 3) {
		size_t best = 0;
		double best_area = INFINITY;
		for (size_t i = 0; i < h.size(); i++) {
			const xrt_vec2 &a = h[(i + h.size() - 1) % h.size()];
			const xrt_vec2 &b = h[(i + 1) % h.size()];
			const double area = std::fabs(cross(a, h[i], b));
			if (area < best_area) {
				best_area = area;
				best = i;
			}
		}
		h.erase(h.begin() + (std::ptrdiff_t)best);
	}

	const uint32_t n = std::min((uint32_t)h.size(), max);
	std::copy(h.begin(), h.begin() + n, out);
	return n;
}


/*
 *
 * Columns.
 *
 */

//! Gaussian, sigma 1 bin, truncated at 4 sigma (as scipy's gaussian_filter1d).
static const float kGauss[9] = {0.00013383f, 0.00443185f, 0.05399097f, 0.24197072f, 0.39894228f,
                                0.24197072f, 0.05399097f, 0.00443185f, 0.00013383f};

static bool
column_peak(const t_plane_map_config &cfg, const Column &c, ColumnPeak *out)
{
	const int n = (int)c.bins.size();
	if (n == 0) {
		return false;
	}

	// Prefix sums: mass(lo, hi) = sum of bins j with lo < j <= hi.
	thread_local std::vector<double> prefix;
	thread_local std::vector<float> smooth;
	prefix.assign((size_t)n + 1, 0.0);
	for (int i = 0; i < n; i++) {
		prefix[(size_t)i + 1] = prefix[(size_t)i] + c.bins[(size_t)i];
	}
	auto mass = [&](int lo, int hi) -> double {
		lo = std::clamp(lo + 1, 0, n);
		hi = std::clamp(hi + 1, 0, n);
		return hi > lo ? prefix[(size_t)hi] - prefix[(size_t)lo] : 0.0;
	};
	if (prefix[(size_t)n] < cfg.min_peak_weight) {
		return false;
	}

	smooth.assign((size_t)n, 0.0f);
	for (int i = 0; i < n; i++) {
		float s = 0;
		for (int j = -4; j <= 4; j++) {
			const int q = i + j;
			if (q >= 0 && q < n) {
				s += kGauss[j + 4] * c.bins[(size_t)q];
			}
		}
		smooth[(size_t)i] = s;
	}

	int k1 = 0;
	for (int i = 1; i < n; i++) {
		if (smooth[(size_t)i] > smooth[(size_t)k1]) {
			k1 = i;
		}
	}

	const int nr = cfg.peak_half_bins;
	const int wr = cfg.wide_half_bins;
	auto near_mass = [&](int k) { return mass(k - nr - 1, k + nr); };
	auto wide_mass = [&](int k) { return mass(k - wr - 1, k + wr); };
	auto above_mass = [&](int k) { return mass(k + nr, k + wr); };
	auto below_mass = [&](int k) { return mass(k - wr - 1, k - nr - 1); };
	auto symmetric_ok = [&](int k) {
		const double nm = near_mass(k);
		return nm >= cfg.min_peak_weight && nm >= cfg.peak_fraction * wide_mass(k);
	};
	// Nothing much above it: a top, with whatever below (mirror images, its own front face).
	auto top_ok = [&](int k) {
		const double nm = near_mass(k);
		return nm >= cfg.min_peak_weight && nm >= cfg.peak_fraction * (nm + 2.0 * above_mass(k)) &&
		       below_mass(k) >= above_mass(k);
	};

	int k = -1;
	if (cfg.upper_mode) {
		const double nm1 = near_mass(k1);
		const int upper_bins = (int)std::lround(cfg.upper_max_m / cfg.bin_m);
		for (int i = std::min(n - 1, k1 + upper_bins); i > k1; i--) {
			const float s = smooth[(size_t)i];
			const bool local_max =
			    s >= smooth[(size_t)(i - 1)] && (i + 1 >= n || s >= smooth[(size_t)(i + 1)]);
			if (!local_max ||
			    near_mass(i) < std::max((double)cfg.min_peak_weight, cfg.upper_fraction * nm1)) {
				continue;
			}
			if (top_ok(i) || symmetric_ok(i)) {
				k = i;
				break;
			}
		}
		if (k < 0 && (symmetric_ok(k1) || top_ok(k1))) {
			k = k1;
		}
	} else if (symmetric_ok(k1)) {
		k = k1;
	}
	if (k < 0) {
		return false;
	}

	double sw = 0, swh = 0;
	for (int i = std::max(0, k - nr); i <= std::min(n - 1, k + nr); i++) {
		const double z = cfg.z_min + (c.base + i + 0.5) * cfg.bin_m;
		sw += c.bins[(size_t)i];
		swh += c.bins[(size_t)i] * z;
	}
	out->level = cfg.z_min + (float)(c.base + k + 0.5) * cfg.bin_m;
	out->centroid = sw > 0 ? (float)(swh / sw) : out->level;
	out->weight = (float)sw;
	return true;
}


/*
 *
 * Extraction.
 *
 */

namespace {

struct Grid
{
	int32_t x0 = 0, y0 = 0, w = 0, h = 0;
	std::vector<uint8_t> good;
	std::vector<ColumnPeak> peak;
	std::vector<int32_t> label;

	size_t
	at(int32_t x, int32_t y) const
	{
		return (size_t)(y - y0) * (size_t)w + (size_t)(x - x0);
	}
	bool
	inside(int32_t x, int32_t y) const
	{
		return x >= x0 && x < x0 + w && y >= y0 && y < y0 + h;
	}
};

} // namespace

static std::vector<Region>
extract_regions(t_plane_map &m)
{
	const t_plane_map_config &cfg = m.cfg;
	std::vector<std::pair<Key, ColumnPeak>> peaky;
	peaky.reserve(m.columns.size() / 4);
	for (const auto &kv : m.columns) {
		ColumnPeak p;
		if (column_peak(cfg, kv.second, &p)) {
			peaky.emplace_back(kv.first, p);
		}
	}
	m.stats.peaky_columns = (uint32_t)peaky.size();
	if (peaky.empty()) {
		return {};
	}

	Grid g;
	int32_t min_x = INT32_MAX, max_x = INT32_MIN, min_y = INT32_MAX, max_y = INT32_MIN;
	for (const auto &p : peaky) {
		min_x = std::min(min_x, key_x(p.first));
		max_x = std::max(max_x, key_x(p.first));
		min_y = std::min(min_y, key_y(p.first));
		max_y = std::max(max_y, key_y(p.first));
	}
	g.x0 = min_x;
	g.y0 = min_y;
	g.w = max_x - min_x + 1;
	g.h = max_y - min_y + 1;
	g.good.assign((size_t)g.w * (size_t)g.h, 0);
	g.peak.assign((size_t)g.w * (size_t)g.h, ColumnPeak{});
	g.label.assign((size_t)g.w * (size_t)g.h, 0);
	for (const auto &p : peaky) {
		const size_t i = g.at(key_x(p.first), key_y(p.first));
		g.good[i] = 1;
		g.peak[i] = p.second;
	}

	// Seeds: the heaviest peaks first.
	std::sort(peaky.begin(), peaky.end(),
	          [](const auto &a, const auto &b) { return a.second.weight > b.second.weight; });

	const float eps = 1e-4f;
	const int gap = cfg.grow_gap_cells;
	std::vector<Region> regions;
	std::vector<std::pair<int32_t, int32_t>> stack;
	int32_t next_label = 0;
	for (const auto &seed : peaky) {
		const int32_t sx = key_x(seed.first), sy = key_y(seed.first);
		if (g.label[g.at(sx, sy)] != 0) {
			continue;
		}
		next_label++;
		const float h0 = seed.second.level;
		Region r;
		g.label[g.at(sx, sy)] = next_label;
		stack.clear();
		stack.emplace_back(sx, sy);
		while (!stack.empty()) {
			const auto [ax, ay] = stack.back();
			stack.pop_back();
			const ColumnPeak &pa = g.peak[g.at(ax, ay)];
			r.cells.push_back(make_key(ax, ay));
			r.heights.push_back(pa.centroid);
			r.weights.push_back(pa.weight);
			for (int dy = -gap; dy <= gap; dy++) {
				for (int dx = -gap; dx <= gap; dx++) {
					const int32_t x = ax + dx, y = ay + dy;
					if (!g.inside(x, y)) {
						continue;
					}
					const size_t i = g.at(x, y);
					if (!g.good[i] || g.label[i] != 0) {
						continue;
					}
					const float hl = g.peak[i].level;
					if (std::fabs(hl - pa.level) > cfg.grow_step_m + eps ||
					    std::fabs(hl - h0) > cfg.grow_total_m + eps) {
						continue;
					}
					g.label[i] = next_label;
					stack.emplace_back(x, y);
				}
			}
		}
		if ((int)r.cells.size() >= cfg.min_cells) {
			r.finish();
			regions.push_back(std::move(r));
		}
	}

	// Merge regions at the same height that are close to each other.
	std::sort(regions.begin(), regions.end(),
	          [](const Region &a, const Region &b) { return a.cells.size() > b.cells.size(); });
	const float gap_cells = cfg.merge_gap_m / cfg.cell_m + 1.0f; // Center to center.
	bool merged = true;
	while (merged) {
		merged = false;
		for (size_t i = 0; i < regions.size() && !merged; i++) {
			for (size_t j = i + 1; j < regions.size() && !merged; j++) {
				Region &a = regions[i];
				Region &b = regions[j];
				if (std::fabs(a.height - b.height) > cfg.merge_dh_m) {
					continue;
				}
				const float bx = (float)std::max({0, b.min_x - a.max_x, a.min_x - b.max_x});
				const float by = (float)std::max({0, b.min_y - a.max_y, a.min_y - b.max_y});
				if (bx * bx + by * by > gap_cells * gap_cells) {
					continue;
				}
				bool close = false;
				for (size_t p = 0; p < a.cells.size() && !close; p++) {
					const float ax = (float)key_x(a.cells[p]), ay = (float)key_y(a.cells[p]);
					for (size_t q = 0; q < b.cells.size(); q++) {
						const float dx = ax - (float)key_x(b.cells[q]);
						const float dy = ay - (float)key_y(b.cells[q]);
						if (dx * dx + dy * dy <= gap_cells * gap_cells) {
							close = true;
							break;
						}
					}
				}
				if (!close) {
					continue;
				}
				a.cells.insert(a.cells.end(), b.cells.begin(), b.cells.end());
				a.heights.insert(a.heights.end(), b.heights.begin(), b.heights.end());
				a.weights.insert(a.weights.end(), b.weights.begin(), b.weights.end());
				a.finish();
				regions.erase(regions.begin() + (std::ptrdiff_t)j);
				merged = true;
			}
		}
	}

	return regions;
}

//! Fill the geometry of @p out (not id / times / label) from @p r.
static void
region_to_plane(const t_plane_map_config &cfg, const Region &r, t_plane *out)
{
	const double cell = cfg.cell_m;
	const size_t n = r.cells.size();

	// Orientation: principal axes of the cell centers.
	double mx = 0, my = 0;
	for (Key k : r.cells) {
		mx += (key_x(k) + 0.5) * cell;
		my += (key_y(k) + 0.5) * cell;
	}
	mx /= (double)n;
	my /= (double)n;
	double cxx = 0, cyy = 0, cxy = 0;
	for (Key k : r.cells) {
		const double dx = (key_x(k) + 0.5) * cell - mx;
		const double dy = (key_y(k) + 0.5) * cell - my;
		cxx += dx * dx;
		cyy += dy * dy;
		cxy += dx * dy;
	}
	const double yaw = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
	const double c = std::cos(yaw), s = std::sin(yaw);

	// Cell corners in a frame at the mean, rotated by yaw.
	std::vector<xrt_vec2> corners;
	corners.reserve(n * 4);
	double lo_u = INFINITY, hi_u = -INFINITY, lo_v = INFINITY, hi_v = -INFINITY;
	for (Key k : r.cells) {
		for (int corner = 0; corner < 4; corner++) {
			const double wx = (key_x(k) + (corner & 1)) * cell - mx;
			const double wy = (key_y(k) + (corner >> 1)) * cell - my;
			const double u = c * wx + s * wy;
			const double v = -s * wx + c * wy;
			lo_u = std::min(lo_u, u);
			hi_u = std::max(hi_u, u);
			lo_v = std::min(lo_v, v);
			hi_v = std::max(hi_v, v);
			corners.push_back({(float)u, (float)v});
		}
	}

	// Re-center on the oriented box.
	const double cu = 0.5 * (lo_u + hi_u), cv = 0.5 * (lo_v + hi_v);
	for (xrt_vec2 &p : corners) {
		p.x -= (float)cu;
		p.y -= (float)cv;
	}

	out->height = r.height;
	out->center.x = (float)(mx + c * cu - s * cv);
	out->center.y = (float)(my + s * cu + c * cv);
	out->yaw = (float)yaw;
	out->extents.x = (float)(hi_u - lo_u);
	out->extents.y = (float)(hi_v - lo_v);
	out->area_m2 = (float)((double)n * cell * cell);
	out->cell_count = (uint32_t)n;
	out->spread_m = r.spread;
	out->vertex_count =
	    t_plane_convex_hull(corners.data(), (uint32_t)corners.size(), out->vertices, T_PLANE_MAX_VERTICES);
}

//! Horizontal distance from @p p (world xy) to the plane's oriented box.
static float
box_distance(const t_plane &pl, float px, float py)
{
	const float dx = px - pl.center.x, dy = py - pl.center.y;
	const float c = std::cos(pl.yaw), s = std::sin(pl.yaw);
	const float u = std::fabs(c * dx + s * dy) - 0.5f * pl.extents.x;
	const float v = std::fabs(-s * dx + c * dy) - 0.5f * pl.extents.y;
	const float ou = std::max(u, 0.0f), ov = std::max(v, 0.0f);
	return std::sqrt(ou * ou + ov * ov);
}

static void
track_regions(t_plane_map &m, std::vector<Region> &regions, int64_t now_ns)
{
	const t_plane_map_config &cfg = m.cfg;
	for (Track &t : m.tracks) {
		t.matched = false;
	}

	// Biggest regions pick first.
	for (const Region &r : regions) {
		int best = -1;
		size_t best_overlap = 0;
		for (size_t ti = 0; ti < m.tracks.size(); ti++) {
			Track &t = m.tracks[ti];
			if (t.matched || std::fabs(t.plane.height - r.height) > cfg.match_dh_m) {
				continue;
			}
			size_t overlap = 0;
			for (Key k : r.cells) {
				overlap += t.cells.count(k);
			}
			if (overlap > best_overlap) {
				best_overlap = overlap;
				best = (int)ti;
			}
		}

		if (best < 0) {
			Track t;
			std::memset(&t.plane, 0, sizeof(t.plane));
			t.plane.id = m.next_id++;
			t.plane.first_seen_ns = now_ns;
			t.plane.label = XRT_PLANE_DETECTOR_SEMANTIC_TYPE_UNDEFINED_EXT;
			m.tracks.push_back(std::move(t));
			best = (int)m.tracks.size() - 1;
		}

		Track &t = m.tracks[(size_t)best];
		t.matched = true;
		t.plane.last_seen_ns = now_ns;
		region_to_plane(cfg, r, &t.plane);
		t.cells.clear();
		t.cells.insert(r.cells.begin(), r.cells.end());
		if (!t.stable && now_ns - t.plane.first_seen_ns >= cfg.stable_ns &&
		    (int)t.plane.cell_count >= cfg.min_report_cells &&
		    std::min(t.plane.extents.x, t.plane.extents.y) >= cfg.min_width_m &&
		    t.plane.spread_m <= cfg.max_spread_m) {
			t.stable = true;
		}
		// Once stable it stays reported unless it shrinks to under half the size it needed.
		t.plane.reportable = t.stable && 2 * (int)t.plane.cell_count >= cfg.min_report_cells;
	}

	m.tracks.erase(
	    std::remove_if(m.tracks.begin(), m.tracks.end(),
	                   [&](const Track &t) { return !t.matched && now_ns - t.plane.last_seen_ns > cfg.grace_ns; }),
	    m.tracks.end());
}

static void
label_tracks(t_plane_map &m)
{
	const t_plane_map_config &cfg = m.cfg;
	if (!m.have_head) {
		return;
	}

	float floor_z = NAN;
	for (const Track &t : m.tracks) {
		if (t.plane.area_m2 < cfg.floor_min_area_m2 || t.plane.height > m.head_z - cfg.floor_min_below_head_m) {
			continue;
		}
		if (std::isnan(floor_z) || t.plane.height < floor_z) {
			floor_z = t.plane.height;
		}
	}
	m.floor_z = floor_z;

	for (Track &t : m.tracks) {
		const float h = t.plane.height;
		const bool is_floor = !std::isnan(floor_z) && std::fabs(h - floor_z) <= 0.05f &&
		                      h <= m.head_z - cfg.floor_min_below_head_m;
		if (is_floor) {
			t.plane.label = XRT_PLANE_DETECTOR_SEMANTIC_TYPE_FLOOR_EXT;
			continue;
		}

		const bool above_floor = !std::isnan(floor_z) && h - floor_z >= cfg.platform_above_floor_min_m &&
		                         h - floor_z <= cfg.platform_above_floor_max_m;
		const float below_head = m.head_z - h;
		const bool reachable = below_head >= cfg.platform_below_head_min_m &&
		                       below_head <= cfg.platform_below_head_max_m &&
		                       box_distance(t.plane, m.head.x, m.head.y) <= cfg.platform_reach_m;
		if (above_floor || reachable) {
			t.platform = true;
		}
		// Sticky (the head moves around), but never at or above eye level.
		const bool below = below_head >= cfg.platform_below_head_min_m;
		t.plane.label = t.platform && below ? XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT
		                                    : XRT_PLANE_DETECTOR_SEMANTIC_TYPE_UNDEFINED_EXT;
	}
}


/*
 *
 * Exported functions.
 *
 */

extern "C" void
t_plane_map_default_config(struct t_plane_map_config *cfg)
{
	std::memset(cfg, 0, sizeof(*cfg));
	cfg->cell_m = 0.05f;
	cfg->bin_m = 0.01f;
	cfg->z_min = -3.0f;
	cfg->z_max = 2.0f;

	cfg->near_m = 0.6f;
	cfg->lap_radius_m = 0.45f;
	cfg->lap_below_m = 0.25f;
	cfg->max_above_head_m = 0.6f;
	cfg->max_range_m = 4.0f;
	cfg->max_sigma_h_m = 0.03f;
	cfg->sigma_ref_m = 0.01f;

	cfg->min_peak_weight = 3.0f;
	cfg->peak_fraction = 0.5f;
	cfg->peak_half_bins = 3;
	cfg->wide_half_bins = 20;
	cfg->upper_mode = false;
	cfg->upper_fraction = 0.35f;
	cfg->upper_max_m = 0.3f;

	cfg->grow_step_m = 0.015f;
	cfg->grow_total_m = 0.04f;
	cfg->grow_gap_cells = 2;
	cfg->min_cells = 8;
	cfg->merge_dh_m = 0.015f;
	cfg->merge_gap_m = 0.3f;

	cfg->match_dh_m = 0.04f;
	cfg->stable_ns = (int64_t)5 * 1000 * 1000 * 1000;
	cfg->grace_ns = (int64_t)3 * 1000 * 1000 * 1000;
	cfg->min_report_cells = 20;
	cfg->min_width_m = 0.15f;
	cfg->max_spread_m = 0.035f;

	cfg->floor_min_area_m2 = 0.1f;
	cfg->floor_min_below_head_m = 0.8f;
	cfg->platform_above_floor_min_m = 0.25f;
	cfg->platform_above_floor_max_m = 1.25f;
	cfg->platform_below_head_min_m = 0.25f;
	cfg->platform_below_head_max_m = 0.8f;
	cfg->platform_reach_m = 1.5f;
}

extern "C" int
t_plane_map_create(const struct t_plane_map_config *cfg, struct t_plane_map **out_map)
{
	if (cfg->cell_m <= 0 || cfg->bin_m <= 0 || cfg->z_max <= cfg->z_min) {
		return -1;
	}
	t_plane_map *m = new t_plane_map();
	m->cfg = *cfg;
	m->stats.floor_z = NAN;
	*out_map = m;
	return 0;
}

extern "C" void
t_plane_map_destroy(struct t_plane_map **map_ptr)
{
	delete *map_ptr;
	*map_ptr = nullptr;
}

extern "C" void
t_plane_map_reset(struct t_plane_map *map)
{
	map->columns.clear();
	map->tracks.clear();
	map->have_head = false;
	map->floor_z = NAN;
	const uint64_t points_in = map->stats.points_in, points_used = map->stats.points_used;
	map->stats = {};
	map->stats.points_in = points_in;
	map->stats.points_used = points_used;
	map->stats.floor_z = NAN;
}

extern "C" uint32_t
t_plane_map_add_points(struct t_plane_map *map,
                       const struct xrt_vec3 *head,
                       const struct t_plane_map_point *points,
                       uint32_t count,
                       float weight_scale)
{
	t_plane_map &m = *map;
	const t_plane_map_config &cfg = m.cfg;

	// The head height for the labels, smoothed over ~10 s of updates at a few Hz.
	if (!m.have_head) {
		m.head_z = head->z;
		m.have_head = true;
	} else {
		m.head_z += 0.02f * (head->z - m.head_z);
	}
	m.head = *head;

	const float near2 = cfg.near_m * cfg.near_m;
	const float lap2 = cfg.lap_radius_m * cfg.lap_radius_m;
	const float range2 = cfg.max_range_m * cfg.max_range_m;
	const int32_t nz = (int32_t)std::floor((cfg.z_max - cfg.z_min) / cfg.bin_m);
	const float ref = cfg.sigma_ref_m;

	uint32_t used = 0;
	for (uint32_t i = 0; i < count; i++) {
		const xrt_vec3 &p = points[i].position;
		const float sigma = points[i].sigma_h;
		if (!(sigma <= cfg.max_sigma_h_m) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
		    !std::isfinite(p.z)) {
			continue;
		}
		const float dx = p.x - head->x, dy = p.y - head->y, dz = p.z - head->z;
		const float horiz2 = dx * dx + dy * dy;
		const float d2 = horiz2 + dz * dz;
		if (d2 < near2 || d2 > range2 || dz > cfg.max_above_head_m) {
			continue;
		}
		if (horiz2 < lap2 && dz < -cfg.lap_below_m) {
			continue;
		}
		const int32_t bin = (int32_t)std::floor((p.z - cfg.z_min) / cfg.bin_m);
		if (bin < 0 || bin >= nz) {
			continue;
		}
		const int32_t ix = (int32_t)std::floor(p.x / cfg.cell_m);
		const int32_t iy = (int32_t)std::floor(p.y / cfg.cell_m);
		const float r = ref / std::max(sigma, 0.5f * ref);
		const float w = weight_scale * std::min(1.0f, r * r);
		m.columns[make_key(ix, iy)].add(bin, w);
		used++;
	}

	m.stats.points_in += count;
	m.stats.points_used += used;
	return used;
}

extern "C" void
t_plane_map_update(struct t_plane_map *map, int64_t now_ns)
{
	std::vector<Region> regions = extract_regions(*map);
	map->stats.regions = (uint32_t)regions.size();
	track_regions(*map, regions, now_ns);
	label_tracks(*map);
}

extern "C" uint32_t
t_plane_map_get_planes(const struct t_plane_map *map, bool reportable_only, struct t_plane *out, uint32_t max)
{
	std::vector<const t_plane *> v;
	for (const Track &t : map->tracks) {
		if (!reportable_only || t.plane.reportable) {
			v.push_back(&t.plane);
		}
	}
	std::sort(v.begin(), v.end(), [](const t_plane *a, const t_plane *b) { return a->area_m2 > b->area_m2; });
	const uint32_t n = std::min((uint32_t)v.size(), max);
	for (uint32_t i = 0; i < n; i++) {
		out[i] = *v[i];
	}
	return n;
}

extern "C" void
t_plane_map_get_stats(const struct t_plane_map *map, struct t_plane_map_stats *out)
{
	*out = map->stats;
	out->columns = (uint32_t)map->columns.size();
	out->tracks = (uint32_t)map->tracks.size();
	out->head_z = map->head_z;
	out->floor_z = map->floor_z;
}
