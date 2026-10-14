// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Replay a recorded IMU dump through m_imu_3dof, with and without gyro bias estimation.
 *
 * Reads the CSV written by the xreal_air driver's XREAL_AIR_IMU_DUMP and runs
 * the samples through two fresh fusions: (a) static factory calibration only and
 * (b) with M_IMU_3DOF_AUTO_GYRO_BIAS and the given tunables. Prints yaw versus
 * time for both (plus the yaw recorded live) as CSV and a drift summary, which
 * is what the estimator tunables get tuned against. With --seed-bias a third
 * fusion starts the estimator from a saved bias, as the driver does at startup.
 */

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_mathinclude.h"

#include "cli_common.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define P(...) fprintf(stderr, __VA_ARGS__)

static void
print_usage(const char *argv0)
{
	P("Usage: %s imu-replay <dump.csv> [options]\n", argv0);
	P("\n");
	P("Replays an XREAL_AIR_IMU_DUMP capture through m_imu_3dof twice: static factory\n");
	P("calibration only, and with automatic gyro bias estimation. Writes CSV to stdout:\n");
	P("  t_s,temp_c,yaw_static_deg,yaw_est_deg,yaw_live_deg,bias_x,bias_y,bias_z (deg/s),still\n");
	P("  [,yaw_seeded_deg,seeded_bias_x,seeded_bias_y,seeded_bias_z with --seed-bias]\n");
	P("and a drift summary to stderr.\n");
	P("\n");
	P("Options:\n");
	P("  --params k=v,...  estimator tunables (struct m_imu_3dof_gyro_bias_params field names)\n");
	P("  --every N         output every Nth sample (default 100, 10 Hz at 1 kHz)\n");
	P("  --tail S          also fit drift over the last S seconds (default 300)\n");
	P("  --head S          also report the yaw change over the first S seconds (default 100)\n");
	P("  --seed-bias x,y,z also replay with the estimator seeded from this bias (deg/s), like\n");
	P("                    the driver does from the saved one at startup\n");
	P("  --seed-sd S       uncertainty of the seed (deg/s, default seed_min_sd_deg_s)\n");
	P("  --device-clock    feed the fusion device timestamps instead of the host-anchored ones\n");
	P("                    the live fusion saw (shows what USB/scheduling jitter costs)\n");
	P("  --quiet           summary only, no CSV\n");
}

struct sample
{
	uint64_t device_ns;
	int64_t fusion_ns;
	int64_t host_ns;
	float temp_c;
	struct xrt_vec3 gyro, accel, mag, bias;
	struct xrt_quat rot;
	int still;
};

static bool
parse_line(const char *line, struct sample *s)
{
	int n = sscanf(line,
	               "%" SCNu64 ",%" SCNd64 ",%" SCNd64 ",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d", //
	               &s->device_ns, &s->fusion_ns, &s->host_ns, &s->temp_c,                                       //
	               &s->gyro.x, &s->gyro.y, &s->gyro.z,                                                          //
	               &s->accel.x, &s->accel.y, &s->accel.z,                                                       //
	               &s->mag.x, &s->mag.y, &s->mag.z,                                                             //
	               &s->bias.x, &s->bias.y, &s->bias.z,                                                          //
	               &s->rot.x, &s->rot.y, &s->rot.z, &s->rot.w,                                                  //
	               &s->still);
	return n == 21;
}

//! Heading about the fusion's up axis (+Y), in degrees.
static double
yaw_deg(const struct xrt_quat *q)
{
	struct xrt_vec3 fwd = {0, 0, -1};
	struct xrt_vec3 out;
	math_quat_rotate_vec3(q, &fwd, &out);
	return atan2(-out.x, -out.z) * 180.0 / M_PI;
}

//! Continuous (unwrapped) yaw track.
struct yaw_track
{
	bool valid;
	double last_raw;
	double value;
};

static double
yaw_track_push(struct yaw_track *t, double raw)
{
	if (!t->valid) {
		t->valid = true;
		t->value = raw;
	} else {
		double d = raw - t->last_raw;
		while (d > 180.0) {
			d -= 360.0;
		}
		while (d < -180.0) {
			d += 360.0;
		}
		t->value += d;
	}
	t->last_raw = raw;
	return t->value;
}

//! Least-squares slope accumulator.
struct fit
{
	double n, sx, sy, sxx, sxy;
};

static void
fit_add(struct fit *f, double x, double y)
{
	f->n += 1;
	f->sx += x;
	f->sy += y;
	f->sxx += x * x;
	f->sxy += x * y;
}

static double
fit_slope(const struct fit *f)
{
	double d = f->n * f->sxx - f->sx * f->sx;
	return d != 0.0 ? (f->n * f->sxy - f->sx * f->sy) / d : 0.0;
}

enum track
{
	TRACK_STATIC = 0,
	TRACK_EST = 1,
	TRACK_LIVE = 2,
	TRACK_SEEDED = 3,
	TRACK_COUNT = 4,
};

static const char *track_names[TRACK_COUNT] = {"static-cal only", "bias estimator", "recorded live", "seeded est."};

int
cli_cmd_imu_replay(int argc, const char **argv)
{
	const char *path = NULL;
	const char *params_str = NULL;
	long every = 100;
	double tail_s = 300.0;
	bool quiet = false;
	bool device_clock = false;
	double head_s = 100.0;
	bool seed = false;
	struct xrt_vec3 seed_bias_deg_s = XRT_VEC3_ZERO;
	double seed_sd_deg_s = -1.0;

	for (int i = 2; i < argc; i++) {
		if (strcmp(argv[i], "--params") == 0 && i + 1 < argc) {
			params_str = argv[++i];
		} else if (strcmp(argv[i], "--every") == 0 && i + 1 < argc) {
			every = strtol(argv[++i], NULL, 10);
		} else if (strcmp(argv[i], "--tail") == 0 && i + 1 < argc) {
			tail_s = strtod(argv[++i], NULL);
		} else if (strcmp(argv[i], "--head") == 0 && i + 1 < argc) {
			head_s = strtod(argv[++i], NULL);
		} else if (strcmp(argv[i], "--seed-bias") == 0 && i + 1 < argc) {
			struct xrt_vec3 *b = &seed_bias_deg_s;
			if (sscanf(argv[++i], "%f,%f,%f", &b->x, &b->y, &b->z) != 3) {
				P("Bad --seed-bias '%s', want x,y,z in deg/s\n", argv[i]);
				return 1;
			}
			seed = true;
		} else if (strcmp(argv[i], "--seed-sd") == 0 && i + 1 < argc) {
			seed_sd_deg_s = strtod(argv[++i], NULL);
		} else if (strcmp(argv[i], "--device-clock") == 0) {
			device_clock = true;
		} else if (strcmp(argv[i], "--quiet") == 0) {
			quiet = true;
		} else if (argv[i][0] != '-' && path == NULL) {
			path = argv[i];
		} else {
			print_usage(argv[0]);
			return 1;
		}
	}
	if (path == NULL || every < 1) {
		print_usage(argv[0]);
		return 1;
	}

	FILE *file = fopen(path, "r");
	if (file == NULL) {
		P("Could not open '%s'\n", path);
		return 1;
	}

	struct m_imu_3dof fusion_static;
	struct m_imu_3dof fusion_est;
	struct m_imu_3dof fusion_seeded;
	m_imu_3dof_init(&fusion_static, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	m_imu_3dof_init(&fusion_est, M_IMU_3DOF_USE_GRAVITY_DUR_20MS | M_IMU_3DOF_AUTO_GYRO_BIAS);
	m_imu_3dof_init(&fusion_seeded, M_IMU_3DOF_USE_GRAVITY_DUR_20MS | M_IMU_3DOF_AUTO_GYRO_BIAS);

	if (params_str != NULL && !m_imu_3dof_gyro_bias_params_parse(&fusion_est.bias_est.params, params_str)) {
		P("Bad --params '%s'\n", params_str);
		fclose(file);
		m_imu_3dof_close(&fusion_static);
		m_imu_3dof_close(&fusion_est);
		m_imu_3dof_close(&fusion_seeded);
		return 1;
	}
	fusion_seeded.bias_est.params = fusion_est.bias_est.params;

	if (seed) {
		if (seed_sd_deg_s <= 0.0) {
			seed_sd_deg_s = (double)fusion_seeded.bias_est.params.seed_min_sd_deg_s;
		}
		struct xrt_vec3 b = {
		    (float)DEG_TO_RAD((double)seed_bias_deg_s.x),
		    (float)DEG_TO_RAD((double)seed_bias_deg_s.y),
		    (float)DEG_TO_RAD((double)seed_bias_deg_s.z),
		};
		m_imu_3dof_gyro_bias_seed(&fusion_seeded, &b, (float)seed_sd_deg_s);
		P("seed:   [%.4f %.4f %.4f] deg/s, sd %.4f deg/s\n", (double)seed_bias_deg_s.x,
		  (double)seed_bias_deg_s.y, (double)seed_bias_deg_s.z, seed_sd_deg_s);
	}
	const int track_count = seed ? TRACK_COUNT : TRACK_SEEDED;

	char params_buf[512];
	m_imu_3dof_gyro_bias_params_print(&fusion_est.bias_est.params, params_buf, sizeof(params_buf));
	P("params: %s\n", params_buf);

	if (!quiet) {
		printf("t_s,temp_c,yaw_static_deg,yaw_est_deg,yaw_live_deg,bias_x,bias_y,bias_z,still%s\n",
		       seed ? ",yaw_seeded_deg,seeded_bias_x,seeded_bias_y,seeded_bias_z" : "");
	}

	struct yaw_track tracks[TRACK_COUNT] = {0};
	double yaw0[TRACK_COUNT] = {0};
	double yaw_end[TRACK_COUNT] = {0};
	struct fit fits[TRACK_COUNT] = {0};

	// Second pass data for the tail fit: keep it simple, store decimated points.
	size_t pts_cap = 1 << 16;
	size_t pts_len = 0;
	double (*pts)[1 + TRACK_COUNT] = calloc(pts_cap, sizeof(*pts));

	char line[1024];
	uint64_t count = 0;
	uint64_t bad_lines = 0;
	uint64_t still_count = 0;
	int64_t first_ns = 0;
	int64_t last_ns = 0;
	uint64_t first_dev_ns = 0;
	uint64_t last_dev_ns = 0;
	float temp_min = 1e9f;
	float temp_max = -1e9f;
	double max_gap_ms = 0.0;
	double max_gap_at_s = 0.0;
	uint64_t max_gap_index = 0;
	double first_learn_s = -1.0;
	double first_learn_seeded_s = -1.0;
	double yaw_head[TRACK_COUNT] = {0};

	while (fgets(line, sizeof(line), file) != NULL) {
		if (line[0] == '#') {
			if (strncmp(line, "# dropped", 9) == 0) {
				P("capture note: %s", line + 2);
			}
			continue;
		}
		if (strncmp(line, "device_ns", 9) == 0) {
			continue;
		}

		struct sample s;
		if (!parse_line(line, &s)) {
			bad_lines++;
			continue;
		}

		if (device_clock) {
			s.fusion_ns = (int64_t)s.device_ns;
		}

		// The fusion needs forward progress.
		if (count > 0 && s.fusion_ns < last_ns) {
			bad_lines++;
			continue;
		}

		if (count == 0) {
			first_ns = s.fusion_ns;
			first_dev_ns = s.device_ns;
		} else {
			double gap_ms = (double)(s.fusion_ns - last_ns) / 1e6;
			if (gap_ms > max_gap_ms) {
				max_gap_ms = gap_ms;
				max_gap_at_s = (double)(s.fusion_ns - first_ns) / 1e9;
				max_gap_index = count;
			}
		}
		last_ns = s.fusion_ns;
		last_dev_ns = s.device_ns;

		temp_min = s.temp_c < temp_min ? s.temp_c : temp_min;
		temp_max = s.temp_c > temp_max ? s.temp_c : temp_max;

		m_imu_3dof_update(&fusion_static, (uint64_t)s.fusion_ns, &s.accel, &s.gyro);
		m_imu_3dof_update(&fusion_est, (uint64_t)s.fusion_ns, &s.accel, &s.gyro);
		if (seed) {
			m_imu_3dof_update(&fusion_seeded, (uint64_t)s.fusion_ns, &s.accel, &s.gyro);
			if (first_learn_seeded_s < 0.0 && fusion_seeded.bias_est.commits > 0) {
				first_learn_seeded_s = (double)(s.fusion_ns - first_ns) / 1e9;
			}
		}
		still_count += fusion_est.bias_est.is_still ? 1 : 0;
		if (first_learn_s < 0.0 && fusion_est.bias_est.commits > 0) {
			first_learn_s = (double)(s.fusion_ns - first_ns) / 1e9;
		}

		double t = (double)(s.fusion_ns - first_ns) / 1e9;
		double yaw[TRACK_COUNT] = {
		    yaw_track_push(&tracks[TRACK_STATIC], yaw_deg(&fusion_static.rot)),
		    yaw_track_push(&tracks[TRACK_EST], yaw_deg(&fusion_est.rot)),
		    yaw_track_push(&tracks[TRACK_LIVE], yaw_deg(&s.rot)),
		    seed ? yaw_track_push(&tracks[TRACK_SEEDED], yaw_deg(&fusion_seeded.rot)) : 0.0,
		};

		if (count == 0) {
			memcpy(yaw0, yaw, sizeof(yaw0));
		}

		for (int k = 0; k < TRACK_COUNT; k++) {
			yaw_end[k] = yaw[k] - yaw0[k];
			if (t <= head_s) {
				yaw_head[k] = yaw_end[k];
			}
		}

		if (count % (uint64_t)every == 0) {
			for (int k = 0; k < TRACK_COUNT; k++) {
				fit_add(&fits[k], t, yaw_end[k]);
			}

			if (pts != NULL) {
				if (pts_len == pts_cap) {
					void *n = realloc(pts, pts_cap * 2 * sizeof(*pts));
					if (n != NULL) {
						pts = n;
						pts_cap *= 2;
					}
				}
				if (pts_len < pts_cap) {
					pts[pts_len][0] = t;
					for (int k = 0; k < TRACK_COUNT; k++) {
						pts[pts_len][1 + k] = yaw_end[k];
					}
					pts_len++;
				}
			}

			if (!quiet) {
				struct xrt_vec3 b = fusion_est.gyro_bias.value;
				printf("%.3f,%.2f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%d", t, (double)s.temp_c,
				       yaw_end[TRACK_STATIC], yaw_end[TRACK_EST], yaw_end[TRACK_LIVE],
				       RAD_TO_DEG((double)b.x), RAD_TO_DEG((double)b.y), RAD_TO_DEG((double)b.z),
				       fusion_est.bias_est.is_still ? 1 : 0);
				if (seed) {
					struct xrt_vec3 sb = fusion_seeded.gyro_bias.value;
					printf(",%.4f,%.5f,%.5f,%.5f", yaw_end[TRACK_SEEDED], RAD_TO_DEG((double)sb.x),
					       RAD_TO_DEG((double)sb.y), RAD_TO_DEG((double)sb.z));
				}
				printf("\n");
			}
		}

		count++;
	}
	fclose(file);

	if (count < 2) {
		P("No samples in '%s' (%" PRIu64 " malformed lines)\n", path, bad_lines);
		free(pts);
		m_imu_3dof_close(&fusion_static);
		m_imu_3dof_close(&fusion_est);
		m_imu_3dof_close(&fusion_seeded);
		return 1;
	}

	double dur_s = (double)(last_ns - first_ns) / 1e9;
	double dev_dur_s = (double)(last_dev_ns - first_dev_ns) / 1e9;

	P("\n");
	P("samples:     %" PRIu64 " over %.1f s -> %.1f Hz (fusion clock); device clock %.1f s -> %.1f Hz\n", count,
	  dur_s, (double)(count - 1) / dur_s, dev_dur_s, dev_dur_s > 0 ? (double)(count - 1) / dev_dur_s : 0.0);
	P("             max gap %.1f ms (sample %" PRIu64 ", t=%.3f s), %" PRIu64 " malformed lines\n", max_gap_ms,
	  max_gap_index, max_gap_at_s, bad_lines);
	P("temperature: %.2f .. %.2f C\n", (double)temp_min, (double)temp_max);
	P("estimator:   still %.1f%% of samples, %u blocks learned (%.0f s), final bias "
	  "[%.4f %.4f %.4f] deg/s\n",
	  100.0 * (double)still_count / (double)count, fusion_est.bias_est.commits, fusion_est.bias_est.learned_s,
	  RAD_TO_DEG((double)fusion_est.gyro_bias.value.x), RAD_TO_DEG((double)fusion_est.gyro_bias.value.y),
	  RAD_TO_DEG((double)fusion_est.gyro_bias.value.z));
	P("             first learned at %.1f s, sd %.4f deg/s, rejects: %u sample, %u block, %u yaw-window\n",
	  first_learn_s, (double)fusion_est.bias_est.sd_deg_s, fusion_est.bias_est.rejects_sample,
	  fusion_est.bias_est.rejects_block, fusion_est.bias_est.rejects_yaw);
	if (seed) {
		struct xrt_vec3 sb = fusion_seeded.gyro_bias.value;
		P("seeded:      %u blocks learned (%.0f s), final bias [%.4f %.4f %.4f] deg/s, first learned at "
		  "%.1f s, sd %.4f deg/s\n",
		  fusion_seeded.bias_est.commits, fusion_seeded.bias_est.learned_s, RAD_TO_DEG((double)sb.x),
		  RAD_TO_DEG((double)sb.y), RAD_TO_DEG((double)sb.z), first_learn_seeded_s,
		  (double)fusion_seeded.bias_est.sd_deg_s);
	}
	P("\n");
	P("yaw            | first %4.0f s (deg) | end-start (deg) | fit, whole (deg/min) | fit, last %.0f s "
	  "(deg/min)\n",
	  head_s, tail_s);

	for (int k = 0; k < track_count; k++) {
		struct fit tail = {0};
		for (size_t i = 0; i < pts_len; i++) {
			if (pts[i][0] >= dur_s - tail_s) {
				fit_add(&tail, pts[i][0], pts[i][1 + k]);
			}
		}
		P("%-15s| %18.3f | %15.3f | %20.3f | %20.3f\n", track_names[k], yaw_head[k], yaw_end[k],
		  fit_slope(&fits[k]) * 60.0, fit_slope(&tail) * 60.0);
	}
	P("\n(Only meaningful for captures where the glasses really did not turn: a worn capture's\n"
	  " yaw is the head's true yaw plus drift.)\n");

	free(pts);
	m_imu_3dof_close(&fusion_static);
	m_imu_3dof_close(&fusion_est);
	m_imu_3dof_close(&fusion_seeded);

	return 0;
}
