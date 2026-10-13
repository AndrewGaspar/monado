// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra camera + IMU dataset recorder (EuRoC layout + sidecars).
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_record.h"

#include "os/os_threading.h"
#include "util/u_misc.h"
#include "xrt/xrt_frame.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef XREAL_AIR_HAVE_ZLIB
#include <zlib.h>

/*!
 * zlib at its fastest level for stb's PNG deflate: ~3x faster than stb's own deflate and smaller
 * files, which is what keeps a single writer thread ahead of 60 images a second.
 */
static unsigned char *
zlib_compress_fast(unsigned char *data, int data_len, int *out_len, int quality)
{
	(void)quality;
	uLongf len = compressBound((uLong)data_len);
	unsigned char *out = malloc(len);
	if (out == NULL) {
		return NULL;
	}
	if (compress2(out, &len, data, (uLong)data_len, Z_BEST_SPEED) != Z_OK) {
		free(out);
		return NULL;
	}
	*out_len = (int)len;
	return out;
}
#define STBIW_ZLIB_COMPRESS zlib_compress_fast
#endif

// Compile a private copy, so this never clashes with the GUI's (gui_stb.c).
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define REC_INFO(r, ...) U_LOG_IFL_I((r)->log_level, "Xreal Air record: " __VA_ARGS__)
#define REC_WARN(r, ...) U_LOG_IFL_W((r)->log_level, "Xreal Air record: " __VA_ARGS__)
#define REC_ERROR(r, ...) U_LOG_IFL_E((r)->log_level, "Xreal Air record: " __VA_ARGS__)

//! Drop new stereo pairs beyond this many queued (four seconds), the disk is not keeping up.
#define MAX_QUEUED_PAIRS 120
//! Raw scrambled frames kept for offline decoder debugging.
#define MAX_RAW_FRAMES 16
//! Re-write the calibration files once this many IMU samples gave a rate estimate.
#define IMU_RATE_SAMPLES 2000

enum item_type
{
	ITEM_IMU,
	ITEM_CAM_FRAME,
	ITEM_STEREO,
	ITEM_RAW,
};

struct item
{
	struct item *next;
	enum item_type type;

	union {
		struct xreal_air_record_imu imu;
		struct xreal_air_record_cam_frame cam_frame;
		struct
		{
			int64_t euroc_ns;
			struct xrt_frame *frames[2];
		} stereo;
		struct
		{
			uint8_t *data;
			size_t size;
			uint32_t sequence;
		} raw;
	};
};

struct xreal_air_record
{
	enum u_logging_level log_level;
	char path[1024];

	//! Its mutex protects the queue and everything in the "shared" block.
	struct os_thread_helper oth;

	struct item *head;
	struct item *tail;
	uint32_t queued_pairs;

	struct
	{
		char *calib_document;
		size_t calib_document_len;
		struct xreal_air_parsed_calibration calib;
		bool calib_valid;
		bool calib_dirty;
		uint32_t raw_frames;
		uint64_t dropped_pairs;
	} shared;

	// Writer thread only.
	FILE *imu_csv;
	FILE *cam_csv[2];
	FILE *imu_raw_csv;
	FILE *cam_frames_csv;
	uint64_t pairs_written;
	uint64_t imu_written;
	int64_t imu_first_ns;
	int64_t imu_last_ns;
	double imu_rate_hz;
	bool imu_rate_final;
};


/*
 *
 * Small helpers.
 *
 */

static bool
mkdir_p(const char *path)
{
	char tmp[PATH_MAX];
	(void)snprintf(tmp, sizeof(tmp), "%s", path);

	for (char *p = tmp + 1; *p != '\0'; p++) {
		if (*p != '/') {
			continue;
		}
		*p = '\0';
		if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
			return false;
		}
		*p = '/';
	}

	return mkdir(tmp, 0755) == 0 || errno == EEXIST;
}

static bool
dir_is_missing_or_empty(const char *path)
{
	DIR *dir = opendir(path);
	if (dir == NULL) {
		return errno == ENOENT;
	}

	bool empty = true;
	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL) {
		if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
			empty = false;
			break;
		}
	}
	(void)closedir(dir);
	return empty;
}

static FILE *
open_file(struct xreal_air_record *r, const char *rel, const char *header)
{
	char path[PATH_MAX];
	(void)snprintf(path, sizeof(path), "%s/%s", r->path, rel);

	FILE *f = fopen(path, "w");
	if (f == NULL) {
		REC_ERROR(r, "Could not open '%s': %s", path, strerror(errno));
		return NULL;
	}
	if (header != NULL) {
		(void)fputs(header, f);
	}
	return f;
}

static void
close_file(FILE **f)
{
	if (*f != NULL) {
		(void)fclose(*f);
		*f = NULL;
	}
}


/*
 *
 * Calibration conversion.
 *
 */

//! Rigid transform, rotation as a Hamilton quaternion (x, y, z, w).
struct pose_d
{
	double q[4];
	double p[3];
};

static void
quat_to_matrix(const double q[4], double m[3][3])
{
	const double x = q[0], y = q[1], z = q[2], w = q[3];
	m[0][0] = 1 - 2 * (y * y + z * z);
	m[0][1] = 2 * (x * y - z * w);
	m[0][2] = 2 * (x * z + y * w);
	m[1][0] = 2 * (x * y + z * w);
	m[1][1] = 1 - 2 * (x * x + z * z);
	m[1][2] = 2 * (y * z - x * w);
	m[2][0] = 2 * (x * z - y * w);
	m[2][1] = 2 * (y * z + x * w);
	m[2][2] = 1 - 2 * (x * x + y * y);
}

static void
quat_mul(const double a[4], const double b[4], double out[4])
{
	out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
	out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
	out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
	out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

static void
pose_rotate(const struct pose_d *t, const double v[3], double out[3])
{
	double m[3][3];
	quat_to_matrix(t->q, m);
	for (int i = 0; i < 3; i++) {
		out[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
	}
}

static void
pose_inverse(const struct pose_d *t, struct pose_d *out)
{
	struct pose_d inv = {.q = {-t->q[0], -t->q[1], -t->q[2], t->q[3]}};
	double p[3];
	pose_rotate(&inv, t->p, p);
	for (int i = 0; i < 3; i++) {
		inv.p[i] = -p[i];
	}
	*out = inv;
}

//! out = a * b
static void
pose_compose(const struct pose_d *a, const struct pose_d *b, struct pose_d *out)
{
	struct pose_d res;
	quat_mul(a->q, b->q, res.q);
	pose_rotate(a, b->p, res.p);
	for (int i = 0; i < 3; i++) {
		res.p[i] += a->p[i];
	}
	*out = res;
}

/*!
 * T_imu_cam from the factory imu_p_cam / imu_q_cam.
 *
 * imu_q_cam is documented in the blob as a JPL quaternion. A JPL quaternion's rotation matrix is
 * the transpose of the Hamilton one with the same components, so the Hamilton rotation taking
 * camera vectors into the IMU frame is the conjugate. This reading makes both cameras pitch ~8
 * degrees down and toe out ~6 degrees. Verified live on decoded stereo pairs: projecting far
 * points from the left image into the right one through this T_imu_cam and the factory
 * intrinsics lands within a few pixels vertically (and on the expected side horizontally,
 * by the finite-depth disparity), while the plain Hamilton reading is off by ~100 px.
 */
static void
factory_T_imu_cam(const struct xreal_air_slam_camera_calibration *cam, struct pose_d *out)
{
	const double *q = cam->imu_q_cam;
	double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n < 1e-9) {
		n = 1.0;
	}

	out->q[0] = -q[0] / n;
	out->q[1] = -q[1] / n;
	out->q[2] = -q[2] / n;
	out->q[3] = q[3] / n;
	for (int i = 0; i < 3; i++) {
		out->p[i] = cam->imu_p_cam[i];
	}
}

static void
write_matrix_yaml(FILE *f, const char *indent, const struct pose_d *t)
{
	double m[3][3];
	quat_to_matrix(t->q, m);
	for (int i = 0; i < 3; i++) {
		(void)fprintf(f, "%s- [%.12f, %.12f, %.12f, %.12f]\n", indent, m[i][0], m[i][1], m[i][2], t->p[i]);
	}
	(void)fprintf(f, "%s- [0.0, 0.0, 0.0, 1.0]\n", indent);
}

static const char *const fisheye624_names[XREAL_AIR_SLAM_CAMERA_MAX_KC] = {
    "k0", "k1", "k2", "k3", "k4", "k5", "p0", "p1", "s0", "s1", "s2", "s3",
};

// Generic Basalt EuRoC style IMU noise values, the factory imu_noises are kept verbatim in the
// README as their meaning is unknown.
#define DEFAULT_ACCEL_NOISE_STD 0.016
#define DEFAULT_GYRO_NOISE_STD 0.000282
#define DEFAULT_ACCEL_BIAS_STD 0.001
#define DEFAULT_GYRO_BIAS_STD 0.0001

static void
write_basalt_calibration(struct xreal_air_record *r, const struct xreal_air_parsed_calibration *calib, double imu_rate)
{
	FILE *f = open_file(r, "xreal/calibration_basalt.json", NULL);
	if (f == NULL) {
		return;
	}

	(void)fprintf(f, "{\n  \"value0\": {\n    \"T_imu_cam\": [\n");
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		struct pose_d t;
		factory_T_imu_cam(&calib->slam_camera[i], &t);
		(void)fprintf(f,
		              "      {\"px\": %.12f, \"py\": %.12f, \"pz\": %.12f, "
		              "\"qx\": %.12f, \"qy\": %.12f, \"qz\": %.12f, \"qw\": %.12f}%s\n",
		              t.p[0], t.p[1], t.p[2], t.q[0], t.q[1], t.q[2], t.q[3],
		              i + 1 < XREAL_AIR_SLAM_CAMERA_COUNT ? "," : "");
	}

	(void)fprintf(f, "    ],\n    \"intrinsics\": [\n");
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		const struct xreal_air_slam_camera_calibration *c = &calib->slam_camera[i];
		(void)fprintf(f,
		              "      {\"camera_type\": \"%s\", \"intrinsics\": {\"fx\": %.12f, \"fy\": %.12f, "
		              "\"cx\": %.12f, \"cy\": %.12f",
		              c->model[0] != '\0' ? c->model : "fisheye624", c->fc[0], c->fc[1], c->cc[0], c->cc[1]);
		for (int k = 0; k < c->kc_count; k++) {
			const bool named = c->kc_count == XREAL_AIR_SLAM_CAMERA_MAX_KC;
			if (named) {
				(void)fprintf(f, ", \"%s\": %.12g", fisheye624_names[k], c->kc[k]);
			} else {
				(void)fprintf(f, ", \"kc%d\": %.12g", k, c->kc[k]);
			}
		}
		(void)fprintf(f, "}}%s\n", i + 1 < XREAL_AIR_SLAM_CAMERA_COUNT ? "," : "");
	}

	(void)fprintf(f, "    ],\n    \"resolution\": [");
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		(void)fprintf(f, "[%d, %d]%s", calib->slam_camera[i].resolution[0], calib->slam_camera[i].resolution[1],
		              i + 1 < XREAL_AIR_SLAM_CAMERA_COUNT ? ", " : "");
	}
	(void)fprintf(f, "],\n");

	// The recorded IMU samples already have the factory bias / scale applied.
	(void)fprintf(f,
	              "    \"calib_accel_bias\": [0, 0, 0, 0, 0, 0, 0, 0, 0],\n"
	              "    \"calib_gyro_bias\": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n"
	              "    \"imu_update_rate\": %.3f,\n"
	              "    \"accel_noise_std\": [%g, %g, %g],\n"
	              "    \"gyro_noise_std\": [%g, %g, %g],\n"
	              "    \"accel_bias_std\": [%g, %g, %g],\n"
	              "    \"gyro_bias_std\": [%g, %g, %g],\n"
	              "    \"T_mocap_world\": {\"px\": 0, \"py\": 0, \"pz\": 0, \"qx\": 0, \"qy\": 0, \"qz\": 0, "
	              "\"qw\": 1},\n"
	              "    \"T_imu_marker\": {\"px\": 0, \"py\": 0, \"pz\": 0, \"qx\": 0, \"qy\": 0, \"qz\": 0, "
	              "\"qw\": 1},\n"
	              "    \"mocap_time_offset_ns\": 0,\n"
	              "    \"mocap_to_imu_offset_ns\": 0,\n"
	              "    \"cam_time_offset_ns\": 0,\n"
	              "    \"vignette\": []\n"
	              "  }\n}\n",
	              imu_rate, DEFAULT_ACCEL_NOISE_STD, DEFAULT_ACCEL_NOISE_STD, DEFAULT_ACCEL_NOISE_STD,
	              DEFAULT_GYRO_NOISE_STD, DEFAULT_GYRO_NOISE_STD, DEFAULT_GYRO_NOISE_STD, DEFAULT_ACCEL_BIAS_STD,
	              DEFAULT_ACCEL_BIAS_STD, DEFAULT_ACCEL_BIAS_STD, DEFAULT_GYRO_BIAS_STD, DEFAULT_GYRO_BIAS_STD,
	              DEFAULT_GYRO_BIAS_STD);

	close_file(&f);
}

static void
write_kalibr_calibration(struct xreal_air_record *r, const struct xreal_air_parsed_calibration *calib, double imu_rate)
{
	FILE *f = open_file(r, "xreal/camchain-imucam.yaml", NULL);
	if (f == NULL) {
		return;
	}

	(void)fprintf(f,
	              "# Kalibr camchain-imucam style, converted from the factory SLAM_camera calibration.\n"
	              "# NOTE: fisheye624 (radial k0-k5, tangential p0 p1, thin prism s0-s3) is not a Kalibr\n"
	              "# model, it needs a fit to e.g. pinhole-equidistant before Kalibr can use it.\n");

	struct pose_d T_cam_imu[XREAL_AIR_SLAM_CAMERA_COUNT];
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		const struct xreal_air_slam_camera_calibration *c = &calib->slam_camera[i];
		struct pose_d T_imu_cam;
		factory_T_imu_cam(c, &T_imu_cam);
		pose_inverse(&T_imu_cam, &T_cam_imu[i]);

		(void)fprintf(f, "cam%d:\n  T_cam_imu:\n", i);
		write_matrix_yaml(f, "  ", &T_cam_imu[i]);
		if (i > 0) {
			// T_cn_cnm1 = T_cn_imu * T_imu_cnm1
			struct pose_d T_imu_prev, T_cn_cnm1;
			pose_inverse(&T_cam_imu[i - 1], &T_imu_prev);
			pose_compose(&T_cam_imu[i], &T_imu_prev, &T_cn_cnm1);
			(void)fprintf(f, "  T_cn_cnm1:\n");
			write_matrix_yaml(f, "  ", &T_cn_cnm1);
		}
		(void)fprintf(f, "  cam_overlaps: [%d]\n", i == 0 ? 1 : 0);
		(void)fprintf(f, "  camera_model: %s\n", c->model[0] != '\0' ? c->model : "fisheye624");
		(void)fprintf(f, "  intrinsics: [%.12f, %.12f, %.12f, %.12f]\n", c->fc[0], c->fc[1], c->cc[0],
		              c->cc[1]);
		(void)fprintf(f, "  distortion_model: %s\n", c->model[0] != '\0' ? c->model : "fisheye624");
		(void)fprintf(f, "  distortion_coeffs: [");
		for (int k = 0; k < c->kc_count; k++) {
			(void)fprintf(f, "%s%.12g", k > 0 ? ", " : "", c->kc[k]);
		}
		(void)fprintf(f, "]\n  resolution: [%d, %d]\n", c->resolution[0], c->resolution[1]);
		(void)fprintf(f, "  rolling_shutter_time: %g\n", c->rolling_shutter_time);
		(void)fprintf(f, "  rostopic: /cam%d/image_raw\n  timeshift_cam_imu: 0.0\n", i);
	}
	close_file(&f);

	f = open_file(r, "xreal/imu.yaml", NULL);
	if (f == NULL) {
		return;
	}
	(void)fprintf(f,
	              "# Kalibr imu.yaml style. Noise values are generic placeholders, the factory imu_noises\n"
	              "# (meaning unknown) are: [%g, %g, %g, %g]\n"
	              "rostopic: /imu0\n"
	              "update_rate: %.3f\n"
	              "accelerometer_noise_density: %g\n"
	              "accelerometer_random_walk: %g\n"
	              "gyroscope_noise_density: %g\n"
	              "gyroscope_random_walk: %g\n",
	              calib->imu_noises[0], calib->imu_noises[1], calib->imu_noises[2], calib->imu_noises[3], imu_rate,
	              DEFAULT_ACCEL_NOISE_STD, DEFAULT_ACCEL_BIAS_STD, DEFAULT_GYRO_NOISE_STD, DEFAULT_GYRO_BIAS_STD);
	close_file(&f);
}

static void
write_calibration_files(struct xreal_air_record *r,
                        const char *document,
                        size_t document_len,
                        const struct xreal_air_parsed_calibration *calib)
{
	if (document != NULL) {
		FILE *f = open_file(r, "xreal/factory_calibration.json", NULL);
		if (f != NULL) {
			(void)fwrite(document, 1, document_len, f);
			close_file(&f);
		}
	}

	bool cams_valid = true;
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		cams_valid = cams_valid && calib->slam_camera[i].valid;
	}
	if (!cams_valid) {
		REC_WARN(r, "No SLAM_camera calibration in the factory blob, not writing converted calibration");
		return;
	}

	const double rate = r->imu_rate_hz > 0 ? r->imu_rate_hz : 1000.0;
	write_basalt_calibration(r, calib, rate);
	write_kalibr_calibration(r, calib, rate);
}

static const char readme[] =
    "XREAL Air 2 Ultra camera + IMU recording (Monado xreal_air driver)\n"
    "===================================================================\n"
    "\n"
    "mav0/            Standard EuRoC MAV layout, for Monado's EuRoC player and Basalt.\n"
    "  cam0/          LEFT camera  (factory SLAM_camera device_1), 480x640 8-bit PNG, upright.\n"
    "  cam1/          RIGHT camera (factory SLAM_camera device_2), 480x640 8-bit PNG, upright.\n"
    "                 Both images of a stereo pair share one timestamp (the left exposure).\n"
    "                 Pairs with zero exposure (auto exposure warm-up after start) are skipped.\n"
    "  imu0/data.csv  Gyro rad/s, accel m/s^2, factory calibrated, in the factory IMU frame\n"
    "                 (x right, y down, z forward) that imu_p_cam / imu_q_cam refer to.\n"
    "\n"
    "Timestamps in mav0 are CLOCK_MONOTONIC ns. The camera telemetry and IMU HID timestamps\n"
    "count the same device clock, which is mapped onto the host clock with a windowed\n"
    "minimum-delay skew tracker fed by the IMU (removes USB jitter). Camera frames are timed by\n"
    "their device time through that same mapping (= camera_frames.csv imu_clock_ns), so camera\n"
    "and IMU share one timeline. With XREAL_AIR_CAMERA_OWN_CLOCK=1, or until the IMU mapping is\n"
    "usable, a pair is timed through the camera's own tracker (mapped_ns) instead, which is\n"
    "~26 ms later (the camera's transport latency).\n"
    "\n"
    "xreal/factory_calibration.json  The verbatim factory calibration document.\n"
    "xreal/calibration_basalt.json   Basalt style, converted from SLAM_camera. camera_type is\n"
    "                                fisheye624 (needs a Basalt that supports it, or a KB4 fit).\n"
    "                                T_imu_cam assumes imu_q_cam is JPL, i.e. Hamilton = conjugate.\n"
    "xreal/camchain-imucam.yaml      Kalibr style, same conversion. xreal/imu.yaml likewise.\n"
    "xreal/camera_frames.csv         Every camera frame (paired or not):\n"
    "  cam             0 left, 1 right (telemetry byte 59, never arrival order)\n"
    "  v4l2_seq        V4L2 sequence number, gaps are frames dropped by the kernel\n"
    "  kernel_ns       V4L2 buffer timestamp, CLOCK_MONOTONIC\n"
    "  dequeue_ns      host CLOCK_MONOTONIC right after VIDIOC_DQBUF\n"
    "  device_ns       telemetry offset 0, device clock ns (power-on epoch)\n"
    "  counter         telemetry offset 18 (u16), stereo pair counter\n"
    "  exposure, gain  telemetry offsets 22 (u24) and 26 (u8)\n"
    "  id58, id59      telemetry bytes 58 (always 0 so far) and 59 (the camera id, 1 = right)\n"
    "  pair_ts         telemetry offset 62 (u64), second per-pair timestamp\n"
    "  phase           descrambler phase\n"
    "  mapped_ns       device_ns mapped via the camera clock tracker, or kernel_ns - 9 ms when\n"
    "                  that is not possible (logged once, counted at shutdown)\n"
    "  imu_clock_ns    device_ns mapped via the IMU clock tracker, only meaningful if the\n"
    "                  camera and IMU device clocks share an epoch (compare to mapped_ns)\n"
    "  euroc_ns        timestamp used in mav0, -1 if not recorded (unpaired / not synced)\n"
    "  telemetry_hex   leading telemetry bytes verbatim\n"
    "xreal/imu_raw.csv               Every IMU sample:\n"
    "  euroc_ns (-1 = not in mav0: clock not synced yet or before the factory calibration arrived),\n"
    "  device_ts (raw HID timestamp), host_ns (handled at), fusion_ns (what the 3DoF\n"
    "  fusion used), temperature, the raw integer gyro/accel/mag with their multiplier and\n"
    "  divisor, and the calibrated gyro/accel as written to mav0/imu0.\n"
    "xreal/raw/<v4l2_seq>.bin        The first few raw (scrambled) 640x482 frames.\n";

static bool
create_files(struct xreal_air_record *r)
{
	static const char *const dirs[] = {"mav0/cam0/data", "mav0/cam1/data", "mav0/imu0", "xreal/raw"};
	for (size_t i = 0; i < ARRAY_SIZE(dirs); i++) {
		char path[PATH_MAX];
		(void)snprintf(path, sizeof(path), "%s/%s", r->path, dirs[i]);
		if (!mkdir_p(path)) {
			REC_ERROR(r, "Could not create '%s': %s", path, strerror(errno));
			return false;
		}
	}

	r->imu_csv = open_file(r, "mav0/imu0/data.csv",
	                       "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
	                       "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\r\n");
	r->cam_csv[0] = open_file(r, "mav0/cam0/data.csv", "#timestamp [ns],filename\r\n");
	r->cam_csv[1] = open_file(r, "mav0/cam1/data.csv", "#timestamp [ns],filename\r\n");
	r->cam_frames_csv = open_file(r, "xreal/camera_frames.csv",
	                              "cam,v4l2_seq,kernel_ns,dequeue_ns,device_ns,counter,exposure,gain,id58,id59,"
	                              "pair_ts,phase,mapped_ns,imu_clock_ns,euroc_ns,telemetry_hex\n");
	r->imu_raw_csv = open_file(r, "xreal/imu_raw.csv",
	                           "euroc_ns,device_ts,host_ns,fusion_ns,temperature_c,"
	                           "gyro_raw_x,gyro_raw_y,gyro_raw_z,gyro_mul,gyro_div,"
	                           "accel_raw_x,accel_raw_y,accel_raw_z,accel_mul,accel_div,"
	                           "mag_raw_x,mag_raw_y,mag_raw_z,mag_mul,mag_div,"
	                           "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n");

	FILE *f = open_file(r, "xreal/README.txt", readme);
	close_file(&f);

	return r->imu_csv != NULL && r->cam_csv[0] != NULL && r->cam_csv[1] != NULL && r->cam_frames_csv != NULL &&
	       r->imu_raw_csv != NULL;
}


/*
 *
 * Writer thread.
 *
 */

static void
write_imu(struct xreal_air_record *r, const struct xreal_air_record_imu *s)
{
	const struct xreal_air_parsed_sample *raw = &s->raw;

	(void)fprintf(r->imu_raw_csv,
	              "%" PRId64 ",%" PRIu64 ",%" PRId64 ",%" PRId64
	              ",%.3f,"
	              "%d,%d,%d,%d,%d,"
	              "%d,%d,%d,%d,%d,"
	              "%d,%d,%d,%d,%d,"
	              "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
	              s->euroc_ns, s->device_ts, s->host_ns, s->fusion_ns, s->temperature, raw->gyro.x, raw->gyro.y,
	              raw->gyro.z, raw->gyro_multiplier, raw->gyro_divisor, raw->accel.x, raw->accel.y, raw->accel.z,
	              raw->accel_multiplier, raw->accel_divisor, raw->mag.x, raw->mag.y, raw->mag.z,
	              raw->mag_multiplier, raw->mag_divisor, s->gyro.x, s->gyro.y, s->gyro.z, s->accel.x, s->accel.y,
	              s->accel.z);

	// Never put an invalid timestamp into mav0 (the capture maps or falls back, this is a backstop).
	if (s->euroc_ns <= 0) {
		return;
	}

	(void)fprintf(r->imu_csv, "%" PRId64 ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\r\n", s->euroc_ns, s->gyro.x, s->gyro.y,
	              s->gyro.z, s->accel.x, s->accel.y, s->accel.z);

	if (r->imu_written == 0) {
		r->imu_first_ns = s->euroc_ns;
	}
	r->imu_last_ns = s->euroc_ns;
	r->imu_written++;
}

static void
write_cam_frame(struct xreal_air_record *r, const struct xreal_air_record_cam_frame *s)
{
	const struct xreal_air_cam_telemetry *t = &s->telemetry;

	char hex[XREAL_AIR_CAM_TELEMETRY_RAW_LEN * 2 + 1];
	for (int i = 0; i < XREAL_AIR_CAM_TELEMETRY_RAW_LEN; i++) {
		(void)snprintf(hex + i * 2, 3, "%02x", t->raw[i]);
	}

	(void)fprintf(r->cam_frames_csv,
	              "%d,%u,%" PRId64 ",%" PRId64 ",%" PRIu64 ",%u,%u,%u,%u,%u,%" PRIu64 ",%u,%" PRId64 ",%" PRId64
	              ",%" PRId64 ",%s\n",
	              (int)s->cam, s->v4l2_sequence, s->kernel_ns, s->dequeue_ns, t->device_ns, t->counter, t->exposure,
	              t->gain, t->id_byte58, t->id_byte59, t->pair_ts, s->phase, s->mapped_ns, s->imu_clock_ns,
	              s->euroc_ns, hex);
}

static void
write_stereo(struct xreal_air_record *r, int64_t euroc_ns, struct xrt_frame *frames[2])
{
	for (int i = 0; i < 2; i++) {
		struct xrt_frame *xf = frames[i];
		char path[PATH_MAX];
		(void)snprintf(path, sizeof(path), "%s/mav0/cam%d/data/%" PRId64 ".png", r->path, i, euroc_ns);

		if (stbi_write_png(path, (int)xf->width, (int)xf->height, 1, xf->data, (int)xf->stride) == 0) {
			REC_ERROR(r, "Could not write '%s'", path);
			continue;
		}
		(void)fprintf(r->cam_csv[i], "%" PRId64 ",%" PRId64 ".png\r\n", euroc_ns, euroc_ns);
	}
	r->pairs_written++;
}

static void
write_raw(struct xreal_air_record *r, const uint8_t *data, size_t size, uint32_t sequence)
{
	char rel[64];
	(void)snprintf(rel, sizeof(rel), "xreal/raw/%08u.bin", sequence);
	FILE *f = open_file(r, rel, NULL);
	if (f != NULL) {
		(void)fwrite(data, 1, size, f);
		close_file(&f);
	}
}

static void
free_item(struct item *it)
{
	if (it->type == ITEM_STEREO) {
		xrt_frame_reference(&it->stereo.frames[0], NULL);
		xrt_frame_reference(&it->stereo.frames[1], NULL);
	} else if (it->type == ITEM_RAW) {
		free(it->raw.data);
	}
	free(it);
}

static void
process_item(struct xreal_air_record *r, struct item *it)
{
	switch (it->type) {
	case ITEM_IMU: write_imu(r, &it->imu); break;
	case ITEM_CAM_FRAME: write_cam_frame(r, &it->cam_frame); break;
	case ITEM_STEREO: write_stereo(r, it->stereo.euroc_ns, it->stereo.frames); break;
	case ITEM_RAW: write_raw(r, it->raw.data, it->raw.size, it->raw.sequence); break;
	}
}

static void
flush_files(struct xreal_air_record *r)
{
	(void)fflush(r->imu_csv);
	(void)fflush(r->cam_csv[0]);
	(void)fflush(r->cam_csv[1]);
	(void)fflush(r->cam_frames_csv);
	(void)fflush(r->imu_raw_csv);
}

//! Called with the lock held, writes the calibration files if they changed (drops the lock to do so).
static void
maybe_write_calibration_locked(struct xreal_air_record *r, bool force)
{
	if (!r->imu_rate_final && r->imu_written >= IMU_RATE_SAMPLES && r->imu_last_ns > r->imu_first_ns) {
		r->imu_rate_hz = (double)(r->imu_written - 1) * 1e9 / (double)(r->imu_last_ns - r->imu_first_ns);
		r->imu_rate_final = true;
		r->shared.calib_dirty = true;
		REC_WARN(r, "Measured IMU rate %.2f Hz", r->imu_rate_hz);
	}

	if (!r->shared.calib_valid || !(r->shared.calib_dirty || force)) {
		return;
	}
	r->shared.calib_dirty = false;

	// The document and calibration are only replaced under the lock, copy the parsed part.
	struct xreal_air_parsed_calibration calib = r->shared.calib;
	char *doc = NULL;
	size_t doc_len = r->shared.calib_document_len;
	if (r->shared.calib_document != NULL) {
		doc = malloc(doc_len);
		if (doc != NULL) {
			memcpy(doc, r->shared.calib_document, doc_len);
		}
	}

	os_thread_helper_unlock(&r->oth);
	write_calibration_files(r, doc, doc_len, &calib);
	free(doc);
	os_thread_helper_lock(&r->oth);
}

static void *
writer_thread(void *ptr)
{
	struct xreal_air_record *r = (struct xreal_air_record *)ptr;
	os_thread_helper_name(&r->oth, "XREAL Air Record");

	os_thread_helper_lock(&r->oth);
	while (true) {
		maybe_write_calibration_locked(r, false);

		if (r->head == NULL) {
			if (!os_thread_helper_is_running_locked(&r->oth)) {
				break;
			}
			os_thread_helper_wait_locked(&r->oth);
			continue;
		}

		// Take the whole queue and write it without holding the lock.
		struct item *list = r->head;
		r->head = r->tail = NULL;
		r->queued_pairs = 0;
		os_thread_helper_unlock(&r->oth);

		while (list != NULL) {
			struct item *next = list->next;
			process_item(r, list);
			free_item(list);
			list = next;
		}
		flush_files(r);

		os_thread_helper_lock(&r->oth);
	}

	maybe_write_calibration_locked(r, true);
	os_thread_helper_unlock(&r->oth);

	return NULL;
}

static void
enqueue(struct xreal_air_record *r, struct item *it)
{
	os_thread_helper_lock(&r->oth);

	if (it->type == ITEM_STEREO) {
		if (r->queued_pairs >= MAX_QUEUED_PAIRS) {
			r->shared.dropped_pairs++;
			os_thread_helper_unlock(&r->oth);
			free_item(it);
			return;
		}
		r->queued_pairs++;
	}

	it->next = NULL;
	if (r->tail != NULL) {
		r->tail->next = it;
	} else {
		r->head = it;
	}
	r->tail = it;

	os_thread_helper_signal_locked(&r->oth);
	os_thread_helper_unlock(&r->oth);
}


/*
 *
 * Exported functions.
 *
 */

struct xreal_air_record *
xreal_air_record_create(const char *dir, enum u_logging_level log_level)
{
	struct xreal_air_record *r = U_TYPED_CALLOC(struct xreal_air_record);
	r->log_level = log_level;
	(void)snprintf(r->path, sizeof(r->path), "%s", dir);

	if (!dir_is_missing_or_empty(r->path)) {
		REC_ERROR(r, "'%s' exists and is not empty, refusing to record into it", r->path);
		free(r);
		return NULL;
	}

	// Fast PNGs, the disk has to keep up with 60 images a second.
	stbi_write_png_compression_level = 2;
	// Skip the per row filter search, the "Sub" filter alone is as small and much faster.
	stbi_write_force_png_filter = 1;

	if (os_thread_helper_init(&r->oth) != 0) {
		free(r);
		return NULL;
	}

	if (!create_files(r) || os_thread_helper_start(&r->oth, writer_thread, r) != 0) {
		REC_ERROR(r, "Could not set up the dataset in '%s'", r->path);
		os_thread_helper_destroy(&r->oth);
		for (int i = 0; i < 2; i++) {
			close_file(&r->cam_csv[i]);
		}
		close_file(&r->imu_csv);
		close_file(&r->cam_frames_csv);
		close_file(&r->imu_raw_csv);
		free(r);
		return NULL;
	}

	REC_INFO(r, "Recording EuRoC dataset to '%s'", r->path);
	return r;
}

const char *
xreal_air_record_path(const struct xreal_air_record *r)
{
	return r->path;
}

void
xreal_air_record_set_calibration(struct xreal_air_record *r,
                                 const char *document,
                                 size_t document_len,
                                 const struct xreal_air_parsed_calibration *calib)
{
	char *doc = NULL;
	if (document != NULL) {
		doc = malloc(document_len);
		if (doc != NULL) {
			memcpy(doc, document, document_len);
		}
	}

	os_thread_helper_lock(&r->oth);
	free(r->shared.calib_document);
	r->shared.calib_document = doc;
	r->shared.calib_document_len = doc != NULL ? document_len : 0;
	r->shared.calib = *calib;
	r->shared.calib_valid = true;
	r->shared.calib_dirty = true;
	os_thread_helper_signal_locked(&r->oth);
	os_thread_helper_unlock(&r->oth);
}

void
xreal_air_record_push_imu(struct xreal_air_record *r, const struct xreal_air_record_imu *imu)
{
	struct item *it = U_TYPED_CALLOC(struct item);
	it->type = ITEM_IMU;
	it->imu = *imu;
	enqueue(r, it);
}

void
xreal_air_record_push_cam_frame(struct xreal_air_record *r, const struct xreal_air_record_cam_frame *frame)
{
	struct item *it = U_TYPED_CALLOC(struct item);
	it->type = ITEM_CAM_FRAME;
	it->cam_frame = *frame;
	enqueue(r, it);
}

void
xreal_air_record_push_stereo(struct xreal_air_record *r,
                             int64_t euroc_ns,
                             struct xrt_frame *left,
                             struct xrt_frame *right)
{
	// Backstop, the capture always hands out mapped (or fallback) times.
	if (euroc_ns <= 0) {
		REC_ERROR(r, "Refusing a stereo pair with invalid timestamp %" PRId64, euroc_ns);
		return;
	}

	struct item *it = U_TYPED_CALLOC(struct item);
	it->type = ITEM_STEREO;
	it->stereo.euroc_ns = euroc_ns;
	xrt_frame_reference(&it->stereo.frames[0], left);
	xrt_frame_reference(&it->stereo.frames[1], right);
	enqueue(r, it);
}

void
xreal_air_record_push_raw(struct xreal_air_record *r, const uint8_t *data, size_t size, uint32_t v4l2_sequence)
{
	os_thread_helper_lock(&r->oth);
	const bool want = r->shared.raw_frames < MAX_RAW_FRAMES;
	if (want) {
		r->shared.raw_frames++;
	}
	os_thread_helper_unlock(&r->oth);

	if (!want) {
		return;
	}

	struct item *it = U_TYPED_CALLOC(struct item);
	it->type = ITEM_RAW;
	it->raw.data = malloc(size);
	if (it->raw.data == NULL) {
		free(it);
		return;
	}
	memcpy(it->raw.data, data, size);
	it->raw.size = size;
	it->raw.sequence = v4l2_sequence;
	enqueue(r, it);
}

void
xreal_air_record_destroy(struct xreal_air_record **r_ptr)
{
	struct xreal_air_record *r = *r_ptr;
	if (r == NULL) {
		return;
	}

	// The writer drains the queue before it exits.
	os_thread_helper_destroy(&r->oth);

	REC_WARN(r, "Closed '%s': %" PRIu64 " stereo pairs, %" PRIu64 " IMU samples, %" PRIu64 " pairs dropped",
	         r->path, r->pairs_written, r->imu_written, r->shared.dropped_pairs);

	for (int i = 0; i < 2; i++) {
		close_file(&r->cam_csv[i]);
	}
	close_file(&r->imu_csv);
	close_file(&r->cam_frames_csv);
	close_file(&r->imu_raw_csv);
	free(r->shared.calib_document);
	free(r);
	*r_ptr = NULL;
}
