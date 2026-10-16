// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Plane detection worker: stereo frames, sparse points and SLAM poses in, a plane snapshot out.
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#include "t_plane_detector.h"

#include "math/m_api.h"
#include "os/os_time.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <string>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace {

//! Most sparse batches waiting (a few seconds of SLAM poses).
constexpr size_t kMaxSparseBatches = 256;
//! Most planes in a snapshot.
constexpr uint32_t kMaxPlanes = 64;
//! Retry interval while a pose is not there yet.
constexpr int64_t kRetryNs = 15 * 1000 * 1000;

struct SparseBatch
{
	int64_t ts = 0;
	int64_t arrival_ns = 0;
	std::vector<t_plane_body_point> points;
};

struct Pair
{
	int64_t ts = 0;
	int64_t arrival_ns = 0;
	xrt_frame *left = nullptr;
	xrt_frame *right = nullptr;

	bool
	valid() const
	{
		return left != nullptr;
	}
	void
	release()
	{
		xrt_frame_reference(&left, nullptr);
		xrt_frame_reference(&right, nullptr);
	}
};

struct Stage
{
	double sum_ms = 0;
	float max_ms = 0;
	uint64_t count = 0;

	void
	add(float ms)
	{
		sum_ms += ms;
		max_ms = std::max(max_ms, ms);
		count++;
	}
	float
	mean() const
	{
		return count > 0 ? (float)(sum_ms / (double)count) : 0.0f;
	}
};

int64_t
now_ns()
{
	return (int64_t)os_monotonic_get_ns();
}

float
ms_since(int64_t t0)
{
	return (float)((double)(now_ns() - t0) / 1e6);
}

} // namespace

struct t_plane_detector
{
	t_plane_detector_config cfg;
	t_plane_pose_fn pose_fn = nullptr;
	void *user = nullptr;

	t_plane_depth_source *depth = nullptr;
	t_plane_map *map = nullptr;

	// Inputs, under mtx.
	std::mutex mtx;
	std::condition_variable cv;
	bool running = true;
	bool reset_requested = false;
	Pair pending;
	int64_t last_accepted_ts = INT64_MIN;
	std::deque<SparseBatch> sparse;

	// Snapshot and stats, under snap_mtx.
	std::mutex snap_mtx;
	std::vector<t_plane> snapshot;
	uint32_t snap_world = 0;
	uint64_t snap_generation = 0;
	t_plane_detector_stats stats = {};
	Stage st_prepare, st_match, st_points, st_integrate, st_sparse, st_extract;
	double dense_points_sum = 0;

	// Worker thread only.
	std::thread thread;
	bool have_world = false;
	uint32_t world_id = 0;
	uint64_t generation = 0;
	int64_t last_extract_ns = 0;
	Pair retry;
	std::deque<SparseBatch> sparse_retry;
	std::vector<t_plane_map_point> map_points;
	std::string dump_path;
	FILE *dump = nullptr;
};


/*
 *
 * Worker.
 *
 */

static void
publish_locked(t_plane_detector &d)
{
	d.snapshot.resize(kMaxPlanes);
	const uint32_t n = t_plane_map_get_planes(d.map, false, d.snapshot.data(), kMaxPlanes);
	d.snapshot.resize(n);
	uint32_t reportable = 0;
	for (const t_plane &p : d.snapshot) {
		reportable += p.reportable ? 1 : 0;
	}
	d.snap_world = d.world_id;
	d.snap_generation = d.generation;
	d.stats.planes = reportable;
	d.stats.world_id = d.world_id;
	d.stats.generation = d.generation;
	t_plane_map_get_stats(d.map, &d.stats.map);
}

//! Integrate (and optionally dump) a batch of world points.
static uint32_t
add_points(t_plane_detector &d, const xrt_vec3 &head, const t_plane_map_point *pts, uint32_t n, float weight_scale)
{
	if (d.dump != nullptr) {
		const int64_t t = now_ns();
		const float h[4] = {head.x, head.y, head.z, weight_scale};
		fwrite(&t, sizeof(t), 1, d.dump);
		fwrite(h, sizeof(h), 1, d.dump);
		fwrite(&n, sizeof(n), 1, d.dump);
		for (uint32_t i = 0; i < n; i++) {
			const float q[4] = {pts[i].position.x, pts[i].position.y, pts[i].position.z, pts[i].sigma_h};
			fwrite(q, sizeof(q), 1, d.dump);
		}
	}
	return t_plane_map_add_points(d.map, &head, pts, n, weight_scale);
}

static void
do_reset(t_plane_detector &d)
{
	if (d.dump != nullptr) {
		const int64_t t = now_ns();
		const float h[4] = {0, 0, 0, 0};
		const uint32_t marker = 0xffffffffu;
		fwrite(&t, sizeof(t), 1, d.dump);
		fwrite(h, sizeof(h), 1, d.dump);
		fwrite(&marker, sizeof(marker), 1, d.dump);
	}
	t_plane_map_reset(d.map);
	d.generation++;
	std::lock_guard<std::mutex> lock(d.snap_mtx);
	d.stats.resets++;
	publish_locked(d);
}

//! A pose came in with @p world_id: a different world than the map's drops the map.
static void
check_world(t_plane_detector &d, uint32_t world_id)
{
	if (d.have_world && world_id == d.world_id) {
		return;
	}
	const bool had = d.have_world;
	d.have_world = true;
	d.world_id = world_id;
	if (had) {
		do_reset(d);
	}
}

static void
transform(const xrt_pose &T, const xrt_matrix_3x3 &R, const xrt_vec3 &p, xrt_vec3 *out)
{
	out->x = R.v[0] * p.x + R.v[1] * p.y + R.v[2] * p.z + T.position.x;
	out->y = R.v[3] * p.x + R.v[4] * p.y + R.v[5] * p.z + T.position.y;
	out->z = R.v[6] * p.x + R.v[7] * p.y + R.v[8] * p.z + T.position.z;
}

//! False if the pose is not there yet (keep the pair for later).
static bool
process_pair(t_plane_detector &d, Pair &pair)
{
	uint32_t world_id = 0;
	xrt_pose T_world_imu = XRT_POSE_IDENTITY;
	const t_plane_pose_status status = d.pose_fn(d.user, pair.ts, &world_id, &T_world_imu);
	if (status == T_PLANE_POSE_NOT_YET && now_ns() - pair.arrival_ns < d.cfg.pose_wait_ns) {
		return false;
	}
	if (status != T_PLANE_POSE_READY) {
		std::lock_guard<std::mutex> lock(d.snap_mtx);
		d.stats.pairs_dropped++;
		pair.release();
		return true;
	}
	check_world(d, world_id);

	t_plane_depth_image img = {};
	t_plane_depth_timing timing = {};
	const bool have_depth = d.depth->compute(d.depth, pair.left, pair.right, &img, &timing);
	pair.release();
	if (!have_depth) {
		std::lock_guard<std::mutex> lock(d.snap_mtx);
		d.stats.pairs_dropped++;
		return true;
	}

	// Depth image -> world points.
	int64_t t0 = now_ns();
	math_quat_normalize(&T_world_imu.orientation);
	xrt_pose T_world_cam;
	math_pose_transform(&T_world_imu, &img.T_imu_cam, &T_world_cam);
	xrt_matrix_3x3 R;
	math_matrix_3x3_from_quat(&T_world_cam.orientation, &R);
	const float cam_z = T_world_cam.position.z;
	d.map_points.resize((size_t)img.width * img.height);
	uint32_t n = 0;
	for (uint32_t v = 0; v < img.height; v++) {
		const float *depth = &img.depth[(size_t)v * img.width];
		const float *sigma = &img.rel_sigma[(size_t)v * img.width];
		const float y = ((float)v - img.cy) / img.fy;
		for (uint32_t u = 0; u < img.width; u++) {
			const float z = depth[u];
			if (!(z > 0.0f)) {
				continue;
			}
			const xrt_vec3 p = {((float)u - img.cx) / img.fx * z, y * z, z};
			t_plane_map_point &mp = d.map_points[n++];
			transform(T_world_cam, R, p, &mp.position);
			mp.sigma_h = std::fabs(mp.position.z - cam_z) * sigma[u];
		}
	}
	const float points_ms = ms_since(t0);

	t0 = now_ns();
	const uint32_t used = add_points(d, T_world_imu.position, d.map_points.data(), n, 1.0f);
	const float integrate_ms = ms_since(t0);

	std::lock_guard<std::mutex> lock(d.snap_mtx);
	d.stats.pairs_processed++;
	d.st_prepare.add(timing.prepare_ms);
	d.st_match.add(timing.match_ms);
	d.st_points.add(points_ms);
	d.st_integrate.add(integrate_ms);
	d.dense_points_sum += used;
	return true;
}

//! False if the pose is not there yet.
static bool
process_sparse(t_plane_detector &d, SparseBatch &b)
{
	uint32_t world_id = 0;
	xrt_pose T_world_imu = XRT_POSE_IDENTITY;
	const t_plane_pose_status status = d.pose_fn(d.user, b.ts, &world_id, &T_world_imu);
	if (status == T_PLANE_POSE_NOT_YET && now_ns() - b.arrival_ns < d.cfg.pose_wait_ns) {
		return false;
	}
	if (status != T_PLANE_POSE_READY) {
		return true;
	}
	check_world(d, world_id);

	const int64_t t0 = now_ns();
	math_quat_normalize(&T_world_imu.orientation);
	xrt_matrix_3x3 R;
	math_matrix_3x3_from_quat(&T_world_imu.orientation, &R);
	const float head_z = T_world_imu.position.z;
	d.map_points.resize(b.points.size());
	for (size_t i = 0; i < b.points.size(); i++) {
		t_plane_map_point &mp = d.map_points[i];
		transform(T_world_imu, R, b.points[i].p_imu, &mp.position);
		mp.sigma_h = std::fabs(mp.position.z - head_z) * b.points[i].rel_sigma;
	}
	const uint32_t used =
	    add_points(d, T_world_imu.position, d.map_points.data(), (uint32_t)b.points.size(), d.cfg.sparse_weight);

	std::lock_guard<std::mutex> lock(d.snap_mtx);
	d.stats.sparse_batches++;
	d.stats.sparse_points += used;
	d.st_sparse.add(ms_since(t0));
	return true;
}

static void
worker(t_plane_detector *dp)
{
	t_plane_detector &d = *dp;
	pthread_setname_np(pthread_self(), "Plane detector");
	// Per-thread nice on Linux.
	(void)setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), d.cfg.nice);

	d.last_extract_ns = now_ns();
	std::unique_lock<std::mutex> lock(d.mtx);
	while (d.running) {
		const bool retrying = d.retry.valid() || !d.sparse_retry.empty();
		int64_t wake = d.last_extract_ns + d.cfg.extract_interval_ns;
		if (retrying) {
			wake = std::min(wake, now_ns() + kRetryNs);
		}
		const int64_t wait_ns = std::max<int64_t>(0, wake - now_ns());
		d.cv.wait_for(lock, std::chrono::nanoseconds(wait_ns), [&] {
			return !d.running || d.reset_requested || d.pending.valid() || !d.sparse.empty();
		});
		if (!d.running) {
			break;
		}

		const bool reset = d.reset_requested;
		d.reset_requested = false;
		Pair pair = d.pending;
		d.pending = Pair{};
		std::deque<SparseBatch> batches;
		batches.swap(d.sparse);
		lock.unlock();

		if (reset) {
			d.retry.release();
			d.sparse_retry.clear();
			batches.clear();
			do_reset(d);
		}

		// Sparse first (their poses are older), oldest first.
		for (SparseBatch &b : batches) {
			d.sparse_retry.push_back(std::move(b));
		}
		while (!d.sparse_retry.empty()) {
			if (!process_sparse(d, d.sparse_retry.front())) {
				break; // Later ones are newer still.
			}
			d.sparse_retry.pop_front();
		}

		if (d.depth != nullptr) {
			if (pair.valid()) {
				// A newer pair replaces one still waiting for its pose.
				if (d.retry.valid()) {
					d.retry.release();
					std::lock_guard<std::mutex> slock(d.snap_mtx);
					d.stats.pairs_dropped++;
				}
				d.retry = pair;
				pair = Pair{};
			}
			if (d.retry.valid() && process_pair(d, d.retry)) {
				d.retry = Pair{};
			}
		} else {
			pair.release();
		}

		if (now_ns() - d.last_extract_ns >= d.cfg.extract_interval_ns) {
			d.last_extract_ns = now_ns();
			const int64_t t0 = now_ns();
			t_plane_map_update(d.map, d.last_extract_ns);
			const float ms = ms_since(t0);
			std::lock_guard<std::mutex> slock(d.snap_mtx);
			d.stats.extractions++;
			d.st_extract.add(ms);
			publish_locked(d);
		}

		lock.lock();
	}
	lock.unlock();

	d.retry.release();
}


/*
 *
 * Exported functions.
 *
 */

extern "C" void
t_plane_detector_default_config(struct t_plane_detector_config *cfg)
{
	std::memset(cfg, 0, sizeof(*cfg));
	cfg->rate_hz = 4.0f;
	cfg->extract_interval_ns = (int64_t)1000 * 1000 * 1000;
	cfg->pose_wait_ns = (int64_t)500 * 1000 * 1000;
	cfg->sparse_weight = 1.0f;
	cfg->nice = 10;
	t_plane_map_default_config(&cfg->map);
}

extern "C" int
t_plane_detector_create(const struct t_plane_detector_config *cfg,
                        struct t_plane_depth_source *depth,
                        t_plane_pose_fn pose_fn,
                        void *user,
                        struct t_plane_detector **out)
{
	t_plane_detector *d = new t_plane_detector();
	d->cfg = *cfg;
	d->pose_fn = pose_fn;
	d->user = user;

	d->depth = depth;
	if (t_plane_map_create(&cfg->map, &d->map) != 0) {
		t_plane_depth_source_destroy(&d->depth);
		delete d;
		return -1;
	}

	if (cfg->dump_path != nullptr && cfg->dump_path[0] != '\0') {
		d->dump_path = cfg->dump_path;
		d->dump = fopen(cfg->dump_path, "wb");
	}
	d->cfg.dump_path = nullptr;

	d->thread = std::thread(worker, d);
	*out = d;
	return 0;
}

extern "C" void
t_plane_detector_destroy(struct t_plane_detector **d_ptr)
{
	t_plane_detector *d = *d_ptr;
	if (d == nullptr) {
		return;
	}
	*d_ptr = nullptr;

	{
		std::lock_guard<std::mutex> lock(d->mtx);
		d->running = false;
	}
	d->cv.notify_all();
	d->thread.join();

	d->pending.release();
	if (d->dump != nullptr) {
		fclose(d->dump);
	}
	t_plane_depth_source_destroy(&d->depth);
	t_plane_map_destroy(&d->map);
	delete d;
}

extern "C" void
t_plane_detector_push_stereo(struct t_plane_detector *d,
                             int64_t timestamp_ns,
                             struct xrt_frame *left,
                             struct xrt_frame *right)
{
	if (d->depth == nullptr) {
		return;
	}
	const int64_t min_interval = d->cfg.rate_hz > 0 ? (int64_t)(1e9 / d->cfg.rate_hz) : 0;

	std::unique_lock<std::mutex> lock(d->mtx);
	if (d->last_accepted_ts != INT64_MIN && timestamp_ns - d->last_accepted_ts < min_interval &&
	    timestamp_ns >= d->last_accepted_ts) {
		return;
	}
	d->last_accepted_ts = timestamp_ns;

	bool replaced = d->pending.valid();
	d->pending.release();
	xrt_frame_reference(&d->pending.left, left);
	xrt_frame_reference(&d->pending.right, right);
	d->pending.ts = timestamp_ns;
	d->pending.arrival_ns = now_ns();
	lock.unlock();
	d->cv.notify_one();

	std::lock_guard<std::mutex> slock(d->snap_mtx);
	d->stats.pairs_pushed++;
	if (replaced) {
		d->stats.pairs_dropped++;
	}
}

extern "C" void
t_plane_detector_push_points(struct t_plane_detector *d,
                             int64_t timestamp_ns,
                             const struct t_plane_body_point *points,
                             uint32_t count)
{
	if (count == 0) {
		return;
	}
	SparseBatch b;
	b.ts = timestamp_ns;
	b.arrival_ns = now_ns();
	b.points.assign(points, points + count);

	{
		std::lock_guard<std::mutex> lock(d->mtx);
		if (d->sparse.size() >= kMaxSparseBatches) {
			d->sparse.pop_front();
		}
		d->sparse.push_back(std::move(b));
	}
	d->cv.notify_one();
}

extern "C" void
t_plane_detector_reset(struct t_plane_detector *d)
{
	{
		std::lock_guard<std::mutex> lock(d->mtx);
		d->reset_requested = true;
	}
	d->cv.notify_one();
}

extern "C" uint32_t
t_plane_detector_get_planes(struct t_plane_detector *d,
                            bool reportable_only,
                            uint32_t *out_world_id,
                            uint64_t *out_generation,
                            struct t_plane *out,
                            uint32_t max)
{
	std::lock_guard<std::mutex> lock(d->snap_mtx);
	uint32_t n = 0;
	for (const t_plane &p : d->snapshot) {
		if (n >= max) {
			break;
		}
		if (!reportable_only || p.reportable) {
			out[n++] = p;
		}
	}
	*out_world_id = d->snap_world;
	*out_generation = d->snap_generation;
	return n;
}

extern "C" void
t_plane_detector_get_stats(struct t_plane_detector *d, struct t_plane_detector_stats *out)
{
	std::lock_guard<std::mutex> lock(d->snap_mtx);
	*out = d->stats;
	out->prepare_ms = d->st_prepare.mean();
	out->prepare_max_ms = d->st_prepare.max_ms;
	out->match_ms = d->st_match.mean();
	out->match_max_ms = d->st_match.max_ms;
	out->points_ms = d->st_points.mean();
	out->points_max_ms = d->st_points.max_ms;
	out->integrate_ms = d->st_integrate.mean();
	out->integrate_max_ms = d->st_integrate.max_ms;
	out->sparse_ms = d->st_sparse.mean();
	out->sparse_max_ms = d->st_sparse.max_ms;
	out->extract_ms = d->st_extract.mean();
	out->extract_max_ms = d->st_extract.max_ms;
	out->dense_points_mean =
	    d->st_integrate.count > 0 ? (float)(d->dense_points_sum / (double)d->st_integrate.count) : 0.0f;
}
