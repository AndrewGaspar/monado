// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Replay an XREAL Air EuRoC recording through the xreal_air driver's 6DoF path.
 *
 * Exercises exactly what the driver does live, minus the USB devices: the factory calibration
 * (xreal/factory_calibration.json) goes through the driver's parser and the SLAM module's
 * conversion (fisheye624 -> KB4 fit, JPL -> Hamilton extrinsics, IMU noise), the IMU samples and
 * stereo pairs go through xreal_air_slam_push_imu / _push_stereo (mid-exposure stamping from the
 * recorded telemetry exposure), and the head pose comes out of xreal_air_slam_get_head_pose blended
 * with a 3DoF fusion run on the same IMU, in real time, at a compositor-like rate.
 *
 * The recording's timestamps are shifted onto the current monotonic clock and played back at 1x
 * with the live arrival latencies, so the stale / warm-up logic sees what it would live.
 *
 * Simulated trouble, each <start_s>,<length_s> in seconds from the first frame:
 *  - XREAL_SLAM_REPLAY_GAP    drop the camera frames (a camera stall: 3DoF fallback, re-acquisition).
 *  - XREAL_SLAM_REPLAY_BLACK  replace the frames with black ones (no features: the tracker goes blind
 *                             but keeps getting frames, like glasses lying face down).
 *  - XREAL_SLAM_REPLAY_STILL  freeze the IMU (gravity where it was plus sensor noise, no rotation) and
 *                             keep showing the last frame, like glasses lying on a desk.
 *  - XREAL_SLAM_REPLAY_DOFF   report the glasses taken off for the window (display off / on edges).
 *
 * Writes <out_dir>/head_poses.csv (one line per pose query) and, through t_slam's CSV writers,
 * <out_dir>/tracking.csv (raw SLAM poses, Basalt frames, shifted clock).
 *
 * @author Andrew Gaspar
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_config_build.h"

#include "cli_common.h"

#include <stdio.h>
#include <stdlib.h>

#if defined(XRT_BUILD_DRIVER_XREAL_AIR) && defined(XRT_FEATURE_SLAM)

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "xreal_air/xreal_air_hmd.h"
#include "xreal_air/xreal_air_slam.h"

#include "cli_xreal_image.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#define P(...) fprintf(stderr, __VA_ARGS__)

//! Live arrival latencies after the sample time (IMU over HID, camera after exposure start).
#define IMU_ARRIVAL_NS (1 * U_TIME_1MS_IN_NS)
#define CAM_ARRIVAL_NS (35 * U_TIME_1MS_IN_NS)
//! Pose queries like a 90 Hz compositor predicting 25 ms ahead.
#define QUERY_INTERVAL_NS (11111111)
#define QUERY_PREDICT_NS (25 * U_TIME_1MS_IN_NS)

struct imu_row
{
	int64_t ts;
	struct xrt_vec3 gyro, accel;
};

struct cam_row
{
	int64_t ts;
	char name[64];
	uint32_t exposure;
};

static char *
read_file(const char *path, size_t *out_size)
{
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);
	char *buf = malloc((size_t)size + 1);
	if (buf == NULL || fread(buf, 1, (size_t)size, f) != (size_t)size) {
		free(buf);
		fclose(f);
		return NULL;
	}
	buf[size] = '\0';
	fclose(f);
	*out_size = (size_t)size;
	return buf;
}

static struct imu_row *
read_imu(const char *dir, size_t *out_count)
{
	char path[1024];
	snprintf(path, sizeof(path), "%s/mav0/imu0/data.csv", dir);
	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return NULL;
	}
	size_t cap = 1 << 17, n = 0;
	struct imu_row *rows = malloc(cap * sizeof(*rows));
	char line[512];
	while (fgets(line, sizeof(line), f) != NULL) {
		if (line[0] == '#') {
			continue;
		}
		double w[3], a[3];
		long long ts;
		if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &ts, &w[0], &w[1], &w[2], &a[0], &a[1], &a[2]) != 7) {
			continue;
		}
		if (n == cap) {
			cap *= 2;
			rows = realloc(rows, cap * sizeof(*rows));
		}
		rows[n++] = (struct imu_row){
		    ts, {(float)w[0], (float)w[1], (float)w[2]}, {(float)a[0], (float)a[1], (float)a[2]}};
	}
	fclose(f);
	*out_count = n;
	return rows;
}

static struct cam_row *
read_cams(const char *dir, size_t *out_count)
{
	char path[1024];
	snprintf(path, sizeof(path), "%s/mav0/cam0/data.csv", dir);
	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return NULL;
	}
	size_t cap = 1 << 13, n = 0;
	struct cam_row *rows = malloc(cap * sizeof(*rows));
	char line[512];
	while (fgets(line, sizeof(line), f) != NULL) {
		if (line[0] == '#') {
			continue;
		}
		long long ts;
		char name[64];
		if (sscanf(line, "%lld,%63s", &ts, name) != 2) {
			continue;
		}
		if (n == cap) {
			cap *= 2;
			rows = realloc(rows, cap * sizeof(*rows));
		}
		rows[n] = (struct cam_row){.ts = ts};
		snprintf(rows[n].name, sizeof(rows[n].name), "%s", name);
		n++;
	}
	fclose(f);

	// The telemetry exposure of each recorded left frame, by its EuRoC timestamp.
	snprintf(path, sizeof(path), "%s/xreal/camera_frames.csv", dir);
	f = fopen(path, "r");
	size_t found = 0;
	if (f != NULL) {
		char big[4096];
		size_t j = 0;
		while (fgets(big, sizeof(big), f) != NULL) {
			int cam;
			long long v4l2_seq, kernel_ns, dequeue_ns, device_ns, counter, exposure, gain, id58, id59,
			    pair_ts, phase, mapped_ns, imu_clock_ns, euroc_ns;
			if (sscanf(big, "%d,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld",
			           &cam, &v4l2_seq, &kernel_ns, &dequeue_ns, &device_ns, &counter, &exposure, &gain,
			           &id58, &id59, &pair_ts, &phase, &mapped_ns, &imu_clock_ns, &euroc_ns) != 15 ||
			    cam != 0 || euroc_ns <= 0) {
				continue;
			}
			while (j < n && rows[j].ts < euroc_ns) {
				j++;
			}
			if (j < n && rows[j].ts == euroc_ns) {
				rows[j].exposure = (uint32_t)exposure;
				found++;
			}
		}
		fclose(f);
	}
	P("cam0: %zu frames, telemetry exposure found for %zu (others use the 16 ms default)\n", n, found);

	*out_count = n;
	return rows;
}

static struct xrt_frame *
load_frame(const char *dir, int cam, const char *name)
{
	char path[1024];
	snprintf(path, sizeof(path), "%s/mav0/cam%d/data/%s", dir, cam, name);
	struct xrt_frame *xf = NULL;
	if (!cli_xreal_load_gray(path, &xf)) {
		P("Could not load '%s'\n", path);
		return NULL;
	}
	return xf;
}

//! A <start_s>,<length_s> window from the environment, inactive when unset.
struct window
{
	double start_s, len_s;
};

static struct window
window_from_env(const char *name)
{
	struct window w = {-1, 0};
	const char *v = getenv(name);
	if (v != NULL && sscanf(v, "%lf,%lf", &w.start_s, &w.len_s) == 2) {
		P("%s: from %.1f s for %.1f s\n", name, w.start_s, w.len_s);
	} else {
		w.start_s = -1;
		w.len_s = 0;
	}
	return w;
}

static bool
in_window(const struct window *w, double rel_s)
{
	return w->len_s > 0 && rel_s >= w->start_s && rel_s < w->start_s + w->len_s;
}

//! Gaussian noise, Box-Muller on a fixed-seed generator (repeatable runs).
static float
noise(unsigned int *seed, float std)
{
	const double u1 = ((double)rand_r(seed) + 1.0) / ((double)RAND_MAX + 2.0);
	const double u2 = ((double)rand_r(seed) + 1.0) / ((double)RAND_MAX + 2.0);
	return (float)(sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2)) * std;
}

static void
sleep_until(int64_t t)
{
	int64_t now = (int64_t)os_monotonic_get_ns();
	if (t > now) {
		os_nanosleep(t - now);
	}
}

int
cli_cmd_xreal_slam_replay(int argc, const char **argv)
{
	if (argc < 4) {
		P("Replay an XREAL Air EuRoC recording through the driver's 6DoF path in real time.\n");
		P("Usage: %s %s <dataset_dir> <out_dir>\n", argv[0], argv[1]);
		P("Needs VIT_SYSTEM_LIBRARY_PATH (libbasalt.so), honours the XREAL_AIR_SLAM_* and SLAM_* options.\n");
		return EXIT_FAILURE;
	}
	const char *dir = argv[2];
	const char *out_dir = argv[3];

	setenv("XREAL_AIR_SLAM", "1", 1);
	setenv("SLAM_WRITE_CSVS", "1", 0);
	setenv("SLAM_CSV_PATH", out_dir, 0);
	setenv("SLAM_TIMING_STAT", "0", 0);
	setenv("SLAM_FEATURES_STAT", "0", 0);

	// The calibration, through the driver's own parser.
	char path[1024];
	snprintf(path, sizeof(path), "%s/xreal/factory_calibration.json", dir);
	size_t calib_size = 0;
	char *calib_buf = read_file(path, &calib_size);
	struct xreal_air_parsed_calibration calib;
	if (calib_buf == NULL || !xreal_air_parse_calibration_buffer(&calib, calib_buf, calib_size)) {
		P("Could not read/parse '%s'\n", path);
		return EXIT_FAILURE;
	}
	free(calib_buf);

	size_t imu_count = 0, cam_count = 0;
	struct imu_row *imu = read_imu(dir, &imu_count);
	struct cam_row *cams = read_cams(dir, &cam_count);
	if (imu == NULL || cams == NULL || imu_count == 0 || cam_count == 0) {
		P("Could not read the IMU / camera CSVs\n");
		return EXIT_FAILURE;
	}

	struct xreal_air_slam *s = xreal_air_slam_create(U_LOGGING_INFO);
	if (s == NULL) {
		P("Could not create the SLAM module\n");
		return EXIT_FAILURE;
	}
	xreal_air_slam_set_calibration(s, &calib);

	bool ready = false, tracking = false, failed = false;
	for (int i = 0; i < 1000 && !ready && !failed; i++) {
		os_nanosleep(10 * U_TIME_1MS_IN_NS);
		xreal_air_slam_get_status(s, &ready, &tracking, &failed);
	}
	if (!ready) {
		P("The tracker did not come up\n");
		xreal_air_slam_destroy(&s);
		return EXIT_FAILURE;
	}

	snprintf(path, sizeof(path), "%s/head_poses.csv", out_dir);
	FILE *out = fopen(path, "w");
	if (out == NULL) {
		P("Could not write '%s'\n", path);
		xreal_air_slam_destroy(&s);
		return EXIT_FAILURE;
	}
	fprintf(out, "#t_dataset_ns,px,py,pz,qx,qy,qz,qw,slam,fusion_qx,fusion_qy,fusion_qz,fusion_qw,flags\n");

	struct m_imu_3dof fusion;
	m_imu_3dof_init(&fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	const int64_t first = imu[0].ts < cams[0].ts ? imu[0].ts : cams[0].ts;
	const int64_t offset = (int64_t)os_monotonic_get_ns() + 200 * U_TIME_1MS_IN_NS - first;
	const int64_t end = cams[cam_count - 1].ts + offset + U_TIME_1S_IN_NS;

	const struct window gap = window_from_env("XREAL_SLAM_REPLAY_GAP");
	const struct window black = window_from_env("XREAL_SLAM_REPLAY_BLACK");
	const struct window still = window_from_env("XREAL_SLAM_REPLAY_STILL");
	const struct window doff = window_from_env("XREAL_SLAM_REPLAY_DOFF");
	bool doffed = false;
	unsigned int seed = 1234;
	struct xrt_vec3 accel_ema = {0, 0, 0}; // Where gravity is, for the frozen IMU.
	bool was_still = false;
	struct xrt_vec3 frozen_accel = {0, 0, 0};
	char frozen_frame[64] = "";

	size_t ii = 0, ci = 0;
	int64_t next_query = first + offset;
	uint64_t queries = 0, queries_tracking = 0;
	P("Replaying %zu IMU samples and %zu stereo pairs, %.1f s\n", imu_count, cam_count,
	  (double)(cams[cam_count - 1].ts - first) / 1e9);

	while (true) {
		const int64_t t_imu = ii < imu_count ? imu[ii].ts + offset + IMU_ARRIVAL_NS : INT64_MAX;
		const int64_t t_cam = ci < cam_count ? cams[ci].ts + offset + CAM_ARRIVAL_NS : INT64_MAX;
		const int64_t t_next = MIN(MIN(t_imu, t_cam), next_query);
		if (t_next > end) {
			break;
		}
		sleep_until(t_next);

		if (t_next == t_imu) {
			const struct imu_row *r = &imu[ii++];
			const double rel_s = (double)(r->ts - cams[0].ts) / 1e9;
			struct xrt_vec3 gyro = r->gyro, accel = r->accel;
			if (in_window(&still, rel_s)) {
				if (!was_still) {
					frozen_accel = accel_ema;
					was_still = true;
				}
				// The measured sensor noise at 1 kHz (4e-5 rad/s/sqrt(Hz), 6.1e-4 m/s^2/sqrt(Hz)).
				gyro = (struct xrt_vec3){noise(&seed, 1e-3f), noise(&seed, 1e-3f), noise(&seed, 1e-3f)};
				accel = (struct xrt_vec3){frozen_accel.x + noise(&seed, 0.014f),
				                          frozen_accel.y + noise(&seed, 0.014f),
				                          frozen_accel.z + noise(&seed, 0.014f)};
			} else {
				was_still = false;
				accel_ema.x += 0.02f * (accel.x - accel_ema.x);
				accel_ema.y += 0.02f * (accel.y - accel_ema.y);
				accel_ema.z += 0.02f * (accel.z - accel_ema.z);
			}
			if (in_window(&doff, rel_s) != doffed) {
				doffed = !doffed;
				P("Replay: glasses %s at %.1f s\n", doffed ? "taken off" : "put on", rel_s);
				xreal_air_slam_set_display_on(s, !doffed);
			}
			xreal_air_slam_push_imu(s, r->ts + offset, &gyro, &accel);
			// The 3DoF fusion runs in the driver's frame (x right, y up, z back).
			const struct xrt_vec3 g = {gyro.x, -gyro.y, -gyro.z};
			const struct xrt_vec3 a = {accel.x, -accel.y, -accel.z};
			m_imu_3dof_update(&fusion, (uint64_t)(r->ts + offset), &a, &g);
		} else if (t_next == t_cam) {
			const struct cam_row *r = &cams[ci++];
			const double rel_s = (double)(r->ts - cams[0].ts) / 1e9;
			if (in_window(&gap, rel_s)) {
				continue; // Simulated camera stall.
			}
			// Lying on a desk: the view does not change.
			if (!in_window(&still, rel_s)) {
				snprintf(frozen_frame, sizeof(frozen_frame), "%s", r->name);
			}
			const char *name = frozen_frame[0] != '\0' ? frozen_frame : r->name;
			struct xrt_frame *l = load_frame(dir, 0, name);
			struct xrt_frame *rt = load_frame(dir, 1, name);
			if (l != NULL && rt != NULL && in_window(&black, rel_s)) {
				memset(l->data, 0, l->size);
				memset(rt->data, 0, rt->size);
			}
			if (l != NULL && rt != NULL) {
				xreal_air_slam_push_stereo(s, r->ts + offset, r->exposure, l, rt);
			}
			xrt_frame_reference(&l, NULL);
			xrt_frame_reference(&rt, NULL);
		} else {
			next_query += QUERY_INTERVAL_NS;
			const int64_t at = (int64_t)os_monotonic_get_ns() + QUERY_PREDICT_NS;

			struct xrt_space_relation f = XRT_SPACE_RELATION_ZERO;
			f.pose.orientation = fusion.rot;
			f.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
			                                                   XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
			struct xrt_space_relation o;
			xreal_air_slam_get_head_pose(s, at, &f, &o);
			xreal_air_slam_get_status(s, &ready, &tracking, &failed);

			queries++;
			queries_tracking += tracking ? 1 : 0;
			const struct xrt_pose *p = &o.pose;
			fprintf(out, "%" PRId64 ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%d,%.7f,%.7f,%.7f,%.7f,%u\n",
			        at - offset, p->position.x, p->position.y, p->position.z, p->orientation.x,
			        p->orientation.y, p->orientation.z, p->orientation.w, tracking ? 1 : 0, fusion.rot.x,
			        fusion.rot.y, fusion.rot.z, fusion.rot.w, (unsigned)o.relation_flags);
		}
	}

	fclose(out);
	P("%" PRIu64 " pose queries, %" PRIu64 " (%.1f%%) on SLAM\n", queries, queries_tracking,
	  queries > 0 ? 100.0 * (double)queries_tracking / (double)queries : 0.0);
	P("Shifted clock: dataset time = shifted time - %" PRId64 "\n", offset);
	snprintf(path, sizeof(path), "%s/offset_ns.txt", out_dir);
	FILE *of = fopen(path, "w");
	if (of != NULL) {
		fprintf(of, "%" PRId64 "\n", offset);
		fclose(of);
	}

	const int64_t t0 = (int64_t)os_monotonic_get_ns();
	xreal_air_slam_destroy(&s);
	P("Shutdown took %.0f ms\n", (double)((int64_t)os_monotonic_get_ns() - t0) / 1e6);

	free(imu);
	free(cams);
	return EXIT_SUCCESS;
}

#else

int
cli_cmd_xreal_slam_replay(int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	fprintf(stderr, "Needs the xreal_air driver and XRT_FEATURE_SLAM.\n");
	return EXIT_FAILURE;
}

#endif
