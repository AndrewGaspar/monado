// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra 6DoF: visual-inertial SLAM through Monado's t_slam / VIT (Basalt).
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_slam.h"
#include "xreal_air_slam_calib.h"
#include "xreal_air_pose_blend.h"

#include "xrt/xrt_config_build.h"
#include "math/m_api.h"
#include "math/m_space.h"
#include "util/u_debug.h"

DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_slam, "XREAL_AIR_SLAM", false)

bool
xreal_air_slam_requested(void)
{
	return debug_get_bool_option_xreal_air_slam();
}

#ifndef XRT_FEATURE_SLAM

struct xreal_air_slam *
xreal_air_slam_create(enum u_logging_level log_level)
{
	if (xreal_air_slam_requested()) {
		U_LOG_IFL_E(log_level,
		            "Xreal Air SLAM: XREAL_AIR_SLAM=1 but this Monado was built without "
		            "XRT_FEATURE_SLAM (needs OpenCV), staying 3DoF");
	}
	return NULL;
}

void
xreal_air_slam_set_calibration(struct xreal_air_slam *s, const struct xreal_air_parsed_calibration *calib)
{
	(void)s;
	(void)calib;
}

void
xreal_air_slam_push_imu(struct xreal_air_slam *s,
                        int64_t timestamp_ns,
                        const struct xrt_vec3 *gyro_rad_s,
                        const struct xrt_vec3 *accel_m_s2)
{
	(void)s;
	(void)timestamp_ns;
	(void)gyro_rad_s;
	(void)accel_m_s2;
}

void
xreal_air_slam_push_stereo(struct xreal_air_slam *s,
                           int64_t left_start_ns,
                           uint32_t exposure_ns,
                           struct xrt_frame *left,
                           struct xrt_frame *right)
{
	(void)s;
	(void)left_start_ns;
	(void)exposure_ns;
	(void)left;
	(void)right;
}

void
xreal_air_slam_get_head_pose(struct xreal_air_slam *s,
                             int64_t at_timestamp_ns,
                             const struct xrt_space_relation *fusion,
                             struct xrt_space_relation *out)
{
	(void)s;
	(void)at_timestamp_ns;
	*out = *fusion;
}

void
xreal_air_slam_get_status(struct xreal_air_slam *s, bool *out_ready, bool *out_tracking, bool *out_failed)
{
	(void)s;
	*out_ready = false;
	*out_tracking = false;
	*out_failed = true;
}

void
xreal_air_slam_destroy(struct xreal_air_slam **s_ptr)
{
	*s_ptr = NULL;
}

#else // XRT_FEATURE_SLAM

#include "os/os_threading.h"
#include "os/os_time.h"
#include "tracking/t_tracking.h"
#include "util/u_misc.h"
#include "util/u_var.h"

#include <inttypes.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

DEBUG_GET_ONCE_OPTION(xreal_air_slam_config, "XREAL_AIR_SLAM_CONFIG", NULL)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_cam_offset_us, "XREAL_AIR_SLAM_CAM_OFFSET_US", 0)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_stale_ms, "XREAL_AIR_SLAM_STALE_MS", 300)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_warmup_ms, "XREAL_AIR_SLAM_WARMUP_MS", 1500)
DEBUG_GET_ONCE_FLOAT_OPTION(xreal_air_slam_kb4_step_px, "XREAL_AIR_SLAM_KB4_STEP_PX", 8.0f)

#define SLAM_INFO(s, ...) U_LOG_IFL_I((s)->log_level, "Xreal Air SLAM: " __VA_ARGS__)
// Lifecycle lines at WARN so they show at the driver's default log level.
#define SLAM_NOTE(s, ...) U_LOG_IFL_W((s)->log_level, "Xreal Air SLAM: " __VA_ARGS__)
#define SLAM_ERROR(s, ...) U_LOG_IFL_E((s)->log_level, "Xreal Air SLAM: " __VA_ARGS__)

//! IMU samples queued for the tracker, 4 s at 1 kHz.
#define IMU_QUEUE_SIZE 4096
#define IMU_BATCH 256

/*
 * IMU noise, measured on the user's unit (white noise gyro ~4e-5 rad/s/sqrt(Hz), accel
 * ~6.1e-4 m/s^2/sqrt(Hz)) and inflated x5, which beat both the raw values and Basalt's EuRoC
 * placeholders on every recording.
 */
#define GYRO_NOISE_STD 2.0e-4
#define ACCEL_NOISE_STD 3.0e-3
#define GYRO_BIAS_STD 1.0e-4
#define ACCEL_BIAS_STD 1.0e-3
#define IMU_FREQUENCY 1000.0
#define CAMERA_FREQUENCY 30.0

//! A new SLAM pose this far from the previous one (without a stall between) = the tracker re-initialised.
#define JUMP_DISTANCE_M 0.25f
#define JUMP_ANGLE_DEG 25.0f //!< Between two 30 Hz poses: 750 deg/s, faster than heads turn.

//! Frames are only pushed once the tracker has had IMU samples for this long before them.
#define FIRST_FRAME_IMU_LEAD_NS ((int64_t)100 * U_TIME_1MS_IN_NS)

//! How long destroy waits for Basalt to shut down (it can hang in ~OpticalFlowBase).
#define SHUTDOWN_TIMEOUT_NS ((int64_t)3 * U_TIME_1S_IN_NS)

struct xreal_air_slam
{
	enum u_logging_level log_level;

	//! Creates the tracker, then forwards IMU samples. Its lock protects the queue fields.
	struct os_thread_helper oth;
	struct xrt_imu_sample *imu_queue;
	uint32_t imu_head;
	uint32_t imu_count;
	bool calib_pending;
	bool calib_received;  //!< Only the first calibration creates the tracker.
	bool forwarding;      //!< The tracker is up, IMU samples are queued.
	int64_t first_imu_ts; //!< Of the first forwarded sample, 0 before.
	struct xreal_air_parsed_calibration calib;

	//! Protects the tracker pointers and the pose state.
	struct os_mutex lock;
	bool ready;
	bool failed;
	struct xrt_frame_context xfctx;
	struct xrt_tracked_slam *xts;
	struct xrt_slam_sinks *sinks;
	struct t_slam_calibration slam_calib;
	char config_path[PATH_MAX];

	struct xreal_air_pose_blend blend;
	bool have_slam_pose;
	int64_t first_slam_ts;
	int64_t last_slam_ts;
	struct xrt_vec3 last_slam_pos;
	struct xrt_quat last_slam_rot;
	uint32_t world_id; //!< Bumped on every detected re-initialisation.
	bool tracking;     //!< For the log lines on changes.

	// Camera thread only.
	int64_t last_frame_ts;
	int64_t cam_offset_ns;

	// Settings.
	int64_t stale_ns;
	int64_t warmup_ns;

	struct
	{
		uint64_t imu_dropped; //!< Queue full.
		uint64_t imu_pushed;
		uint64_t pairs_pushed;
		uint64_t pairs_out_of_order;
		uint64_t discontinuities; //!< Tracker re-initialisations seen.
		float kb4_rms[2];
		float kb4_max[2];
	} stats;
};


/*
 *
 * Calibration.
 *
 */

static bool
fill_slam_calibration(struct xreal_air_slam *s, const struct xreal_air_parsed_calibration *calib)
{
	struct t_slam_calibration *c = &s->slam_calib;
	U_ZERO(c);

	const double step = debug_get_float_option_xreal_air_slam_kb4_step_px();
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		const struct xreal_air_slam_camera_calibration *cam = &calib->slam_camera[i];
		if (!cam->valid || strcmp(cam->model, "fisheye624") != 0) {
			SLAM_ERROR(s, "SLAM camera %d: no fisheye624 factory calibration (model '%s')", i, cam->model);
			return false;
		}

		struct xreal_air_kb4 kb4;
		const timepoint_ns t0 = os_monotonic_get_ns();
		if (!xreal_air_kb4_fit(cam, step, &kb4)) {
			SLAM_ERROR(s, "SLAM camera %d: the KB4 fit of the fisheye624 calibration failed", i);
			return false;
		}
		const double fit_ms = (double)(os_monotonic_get_ns() - t0) / U_TIME_1MS_IN_NS;
		s->stats.kb4_rms[i] = (float)kb4.rms_px;
		s->stats.kb4_max[i] = (float)kb4.max_px;

		double q[4], p[3];
		xreal_air_slam_T_imu_cam(cam, q, p);

		SLAM_NOTE(s,
		          "cam%d (%s): KB4 fx=%.3f fy=%.3f cx=%.3f cy=%.3f k=[%.6f %.6f %.6f %.6f], fit RMS %.2f px, "
		          "max %.2f px over %d points (%.0f ms); T_imu_cam q=[%.6f %.6f %.6f %.6f] p=[%.4f %.4f %.4f]",
		          i, i == 0 ? "left" : "right", kb4.fx, kb4.fy, kb4.cx, kb4.cy, kb4.k[0], kb4.k[1], kb4.k[2],
		          kb4.k[3], kb4.rms_px, kb4.max_px, kb4.points, fit_ms, q[0], q[1], q[2], q[3], p[0], p[1],
		          p[2]);

		struct t_slam_camera_calibration *sc = &c->cams[i];
		sc->base.image_size_pixels.w = (uint32_t)cam->resolution[0];
		sc->base.image_size_pixels.h = (uint32_t)cam->resolution[1];
		sc->base.intrinsics[0][0] = kb4.fx;
		sc->base.intrinsics[0][2] = kb4.cx;
		sc->base.intrinsics[1][1] = kb4.fy;
		sc->base.intrinsics[1][2] = kb4.cy;
		sc->base.intrinsics[2][2] = 1.0;
		sc->base.distortion_model = T_DISTORTION_FISHEYE_KB4;
		sc->base.kb4.k1 = kb4.k[0];
		sc->base.kb4.k2 = kb4.k[1];
		sc->base.kb4.k3 = kb4.k[2];
		sc->base.kb4.k4 = kb4.k[3];
		sc->frequency = CAMERA_FREQUENCY;

		const struct xrt_pose pose = {
		    .orientation = {(float)q[0], (float)q[1], (float)q[2], (float)q[3]},
		    .position = {(float)p[0], (float)p[1], (float)p[2]},
		};
		math_matrix_4x4_isometry_from_pose(&pose, &sc->T_imu_cam);
	}
	c->cam_count = XREAL_AIR_SLAM_CAMERA_COUNT;

	// The samples handed over are already factory calibrated: identity transform, zero offset.
	struct t_inertial_calibration *accel = &c->imu.base.accel;
	struct t_inertial_calibration *gyro = &c->imu.base.gyro;
	for (int i = 0; i < 3; i++) {
		accel->transform[i][i] = 1.0;
		gyro->transform[i][i] = 1.0;
		accel->noise_std[i] = ACCEL_NOISE_STD;
		accel->bias_std[i] = ACCEL_BIAS_STD;
		gyro->noise_std[i] = GYRO_NOISE_STD;
		gyro->bias_std[i] = GYRO_BIAS_STD;
	}
	c->imu.frequency = IMU_FREQUENCY;

	return true;
}

static void
resolve_config_path(struct xreal_air_slam *s, const char *vit_library)
{
	s->config_path[0] = '\0';

	const char *forced = debug_get_option_xreal_air_slam_config();
	if (forced != NULL && forced[0] != '\0') {
		(void)snprintf(s->config_path, sizeof(s->config_path), "%s", forced);
		return;
	}

	// <prefix>/lib/libbasalt.so -> <prefix>/share/basalt/xreal_air.toml
	const char *slash = vit_library != NULL ? strrchr(vit_library, '/') : NULL;
	if (slash == NULL) {
		return;
	}
	char candidate[PATH_MAX];
	(void)snprintf(candidate, sizeof(candidate), "%.*s/../share/basalt/xreal_air.toml", (int)(slash - vit_library),
	               vit_library);
	if (access(candidate, R_OK) == 0) {
		(void)snprintf(s->config_path, sizeof(s->config_path), "%s", candidate);
	}
}


/*
 *
 * Tracker thread.
 *
 */

static void
mark_failed(struct xreal_air_slam *s)
{
	os_mutex_lock(&s->lock);
	s->failed = true;
	os_mutex_unlock(&s->lock);
}

static void
create_tracker(struct xreal_air_slam *s, const struct xreal_air_parsed_calibration *calib)
{
	if (!fill_slam_calibration(s, calib)) {
		mark_failed(s);
		return;
	}

	struct t_slam_tracker_config config;
	t_slam_fill_default_config(&config);
	config.cam_count = XREAL_AIR_SLAM_CAMERA_COUNT;
	config.slam_calib = &s->slam_calib;
	config.send_calib_with_config_file = true;
	config.submit_from_start = true;

	resolve_config_path(s, config.vit_system_library_path);
	config.slam_config = s->config_path[0] != '\0' ? s->config_path : NULL;

	SLAM_NOTE(s, "Creating the tracker: VIT library '%s', pipeline config '%s'", config.vit_system_library_path,
	          config.slam_config != NULL ? config.slam_config : "(Basalt defaults)");

	struct xrt_tracked_slam *xts = NULL;
	struct xrt_slam_sinks *sinks = NULL;
	const timepoint_ns t0 = os_monotonic_get_ns();
	if (t_slam_create(&s->xfctx, &config, &xts, &sinks) != 0 || xts == NULL || sinks == NULL) {
		SLAM_ERROR(s,
		           "Could not create the SLAM tracker (VIT_SYSTEM_LIBRARY_PATH set to a libbasalt.so?), "
		           "staying 3DoF");
		xrt_frame_context_destroy_nodes(&s->xfctx);
		mark_failed(s);
		return;
	}
	if (t_slam_start(xts) != 0) {
		SLAM_ERROR(s, "Could not start the SLAM tracker, staying 3DoF");
		xrt_frame_context_destroy_nodes(&s->xfctx);
		mark_failed(s);
		return;
	}

	os_mutex_lock(&s->lock);
	s->xts = xts;
	s->sinks = sinks;
	s->ready = true;
	os_mutex_unlock(&s->lock);

	SLAM_NOTE(s, "Tracker running (created in %.0f ms), waiting for camera frames",
	          (double)(os_monotonic_get_ns() - t0) / U_TIME_1MS_IN_NS);
}

static void *
tracker_thread(void *ptr)
{
	struct xreal_air_slam *s = (struct xreal_air_slam *)ptr;
	os_thread_helper_name(&s->oth, "XREAL Air SLAM");

	struct xrt_imu_sample batch[IMU_BATCH];

	os_thread_helper_lock(&s->oth);
	while (os_thread_helper_is_running_locked(&s->oth)) {
		if (s->calib_pending) {
			s->calib_pending = false;
			struct xreal_air_parsed_calibration calib = s->calib;
			os_thread_helper_unlock(&s->oth);

			create_tracker(s, &calib);

			os_thread_helper_lock(&s->oth);
			s->forwarding = s->ready;
			continue;
		}

		if (s->imu_count == 0) {
			os_thread_helper_wait_locked(&s->oth);
			continue;
		}

		uint32_t n = 0;
		while (s->imu_count > 0 && n < IMU_BATCH) {
			batch[n++] = s->imu_queue[s->imu_head];
			s->imu_head = (s->imu_head + 1) % IMU_QUEUE_SIZE;
			s->imu_count--;
		}
		os_thread_helper_unlock(&s->oth);

		// May block while Basalt's IMU queue is full, which only stalls this thread.
		for (uint32_t i = 0; i < n; i++) {
			xrt_sink_push_imu(s->sinks->imu, &batch[i]);
		}

		os_thread_helper_lock(&s->oth);
		s->stats.imu_pushed += n;
	}
	os_thread_helper_unlock(&s->oth);

	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

struct xreal_air_slam *
xreal_air_slam_create(enum u_logging_level log_level)
{
	if (!xreal_air_slam_requested()) {
		return NULL;
	}

	struct xreal_air_slam *s = U_TYPED_CALLOC(struct xreal_air_slam);
	s->log_level = log_level;
	s->imu_queue = U_TYPED_ARRAY_CALLOC(struct xrt_imu_sample, IMU_QUEUE_SIZE);
	s->last_frame_ts = INT64_MIN;
	s->cam_offset_ns = (int64_t)debug_get_num_option_xreal_air_slam_cam_offset_us() * U_TIME_1US_IN_NS;
	s->stale_ns = (int64_t)debug_get_num_option_xreal_air_slam_stale_ms() * U_TIME_1MS_IN_NS;
	s->warmup_ns = (int64_t)debug_get_num_option_xreal_air_slam_warmup_ms() * U_TIME_1MS_IN_NS;
	xreal_air_pose_blend_init(&s->blend);

	if (os_mutex_init(&s->lock) != 0 || os_thread_helper_init(&s->oth) != 0 ||
	    os_thread_helper_start(&s->oth, tracker_thread, s) != 0) {
		SLAM_ERROR(s, "Could not start the SLAM thread");
		// Leaks the half-initialised primitives, this does not happen in practice.
		return NULL;
	}

	u_var_add_root(s, "Xreal Air SLAM", true);
	u_var_add_ro_u64(s, &s->stats.imu_pushed, "IMU samples pushed");
	u_var_add_ro_u64(s, &s->stats.imu_dropped, "IMU samples dropped");
	u_var_add_ro_u64(s, &s->stats.pairs_pushed, "Stereo pairs pushed");
	u_var_add_ro_u64(s, &s->stats.pairs_out_of_order, "Stereo pairs out of order");
	u_var_add_ro_u64(s, &s->stats.discontinuities, "Tracker re-initialisations");
	u_var_add_ro_u32(s, &s->blend.slam_acquired, "SLAM acquired");
	u_var_add_ro_u32(s, &s->blend.slam_lost, "SLAM lost");
	u_var_add_bool(s, &s->tracking, "Tracking (read only)");
	u_var_add_pose(s, &s->blend.last_out, "Last head pose");

	SLAM_NOTE(s,
	          "6DoF requested (XREAL_AIR_SLAM=1), the tracker starts once the factory calibration is in. "
	          "Mid-exposure frame stamps%s, stale after %" PRId64 " ms, warm-up %" PRId64 " ms",
	          s->cam_offset_ns != 0 ? " plus an extra offset" : "", s->stale_ns / U_TIME_1MS_IN_NS,
	          s->warmup_ns / U_TIME_1MS_IN_NS);

	return s;
}

void
xreal_air_slam_set_calibration(struct xreal_air_slam *s, const struct xreal_air_parsed_calibration *calib)
{
	os_thread_helper_lock(&s->oth);
	if (!s->calib_received) {
		s->calib_received = true;
		s->calib = *calib;
		s->calib_pending = true;
		os_thread_helper_signal_locked(&s->oth);
	}
	os_thread_helper_unlock(&s->oth);
}

void
xreal_air_slam_push_imu(struct xreal_air_slam *s,
                        int64_t timestamp_ns,
                        const struct xrt_vec3 *gyro_rad_s,
                        const struct xrt_vec3 *accel_m_s2)
{
	const struct xrt_imu_sample sample = {
	    .timestamp_ns = timestamp_ns,
	    .accel_m_s2 = {accel_m_s2->x, accel_m_s2->y, accel_m_s2->z},
	    .gyro_rad_secs = {gyro_rad_s->x, gyro_rad_s->y, gyro_rad_s->z},
	};

	os_thread_helper_lock(&s->oth);
	if (s->forwarding) {
		if (s->first_imu_ts == 0) {
			s->first_imu_ts = timestamp_ns;
		}
		if (s->imu_count < IMU_QUEUE_SIZE) {
			s->imu_queue[(s->imu_head + s->imu_count) % IMU_QUEUE_SIZE] = sample;
			s->imu_count++;
			os_thread_helper_signal_locked(&s->oth);
		} else {
			if (s->stats.imu_dropped++ == 0) {
				SLAM_ERROR(
				    s, "The tracker is not keeping up, dropping IMU samples (only counted from now)");
			}
		}
	}
	os_thread_helper_unlock(&s->oth);
}

void
xreal_air_slam_push_stereo(struct xreal_air_slam *s,
                           int64_t left_start_ns,
                           uint32_t exposure_ns,
                           struct xrt_frame *left,
                           struct xrt_frame *right)
{
	const int64_t timestamp_ns = xreal_air_slam_frame_timestamp(left_start_ns, exposure_ns, s->cam_offset_ns);

	os_mutex_lock(&s->lock);
	const bool ready = s->ready;
	struct xrt_slam_sinks *sinks = s->sinks;
	os_mutex_unlock(&s->lock);

	if (!ready) {
		return;
	}

	// Basalt wants IMU data from before the first frame (it initialises from the accelerometer).
	os_thread_helper_lock(&s->oth);
	const int64_t first_imu_ts = s->first_imu_ts;
	os_thread_helper_unlock(&s->oth);
	if (first_imu_ts == 0 || timestamp_ns < first_imu_ts + FIRST_FRAME_IMU_LEAD_NS) {
		return;
	}

	if (timestamp_ns <= s->last_frame_ts) {
		s->stats.pairs_out_of_order++;
		return;
	}
	s->last_frame_ts = timestamp_ns;

	// The frames are only read by the capture's own sinks otherwise, which are done with them.
	left->timestamp = (uint64_t)timestamp_ns;
	right->timestamp = (uint64_t)timestamp_ns;

	// Always both, in this order: Basalt expects cam0 then cam1 with the same timestamp.
	xrt_sink_push_frame(sinks->cams[0], left);
	xrt_sink_push_frame(sinks->cams[1], right);

	if (s->stats.pairs_pushed++ == 0) {
		SLAM_NOTE(s, "First stereo pair pushed to the tracker");
	}
}

//! Called with the lock held. The fresh SLAM head relation at @p at_ns, false if not tracking.
static bool
get_slam_head_locked(struct xreal_air_slam *s, int64_t at_ns, struct xrt_space_relation *out_head)
{
	if (!s->ready) {
		return false;
	}

	timepoint_ns latest_ts = 0;
	struct xrt_space_relation latest = XRT_SPACE_RELATION_ZERO;
	if (!t_slam_get_latest_pose(s->xts, &latest_ts, &latest)) {
		return false;
	}

	if (latest_ts != s->last_slam_ts) {
		if (!s->have_slam_pose) {
			s->have_slam_pose = true;
			s->first_slam_ts = latest_ts;
			SLAM_NOTE(s, "First SLAM pose, warming up");
		} else {
			struct xrt_vec3 d = {latest.pose.position.x - s->last_slam_pos.x,
			                     latest.pose.position.y - s->last_slam_pos.y,
			                     latest.pose.position.z - s->last_slam_pos.z};
			const float dist = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
			const float dot = fabsf(math_quat_dot(&latest.pose.orientation, &s->last_slam_rot));
			const float angle_deg = 2.0f * acosf(dot > 1.0f ? 1.0f : dot) * 180.0f / (float)M_PI;
			if (latest_ts - s->last_slam_ts > s->stale_ns) {
				// Back from a stall: same world, but let it settle again before following it.
				s->first_slam_ts = latest_ts;
				SLAM_NOTE(s, "SLAM poses again after %.1f s, warming up",
				          (double)(latest_ts - s->last_slam_ts) / U_TIME_1S_IN_NS);
			} else if (dist > JUMP_DISTANCE_M || angle_deg > JUMP_ANGLE_DEG) {
				s->stats.discontinuities++;
				s->world_id++;
				s->first_slam_ts = latest_ts; // Warm up again.
				SLAM_NOTE(s, "SLAM pose jumped %.2f m / %.0f deg, treating it as a re-initialisation",
				          dist, angle_deg);
			}
		}
		s->last_slam_ts = latest_ts;
		s->last_slam_pos = latest.pose.position;
		s->last_slam_rot = latest.pose.orientation;
	}

	const int64_t now = (int64_t)os_monotonic_get_ns();
	if (now - latest_ts > s->stale_ns || latest_ts - s->first_slam_ts < s->warmup_ns) {
		return false;
	}

	struct xrt_space_relation world_imu = XRT_SPACE_RELATION_ZERO;
	xrt_tracked_slam_get_tracked_pose(s->xts, at_ns, &world_imu);
	if ((world_imu.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) == 0 ||
	    (world_imu.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return false;
	}

	xreal_air_basalt_to_head(&world_imu, out_head);
	return true;
}

void
xreal_air_slam_get_head_pose(struct xreal_air_slam *s,
                             int64_t at_timestamp_ns,
                             const struct xrt_space_relation *fusion,
                             struct xrt_space_relation *out)
{
	os_mutex_lock(&s->lock);

	struct xrt_space_relation head;
	const bool tracking = get_slam_head_locked(s, at_timestamp_ns, &head);
	xreal_air_pose_blend_update(&s->blend, at_timestamp_ns, tracking ? &head : NULL, s->world_id, fusion, out);

	if (tracking != s->tracking) {
		s->tracking = tracking;
		if (tracking) {
			SLAM_NOTE(s, "6DoF tracking (SLAM acquired, #%u)", s->blend.slam_acquired);
		} else if (s->blend.slam_lost > 0) {
			SLAM_NOTE(s, "Lost SLAM tracking (#%u), 3DoF with the position held", s->blend.slam_lost);
		}
	}

	os_mutex_unlock(&s->lock);
}

void
xreal_air_slam_get_status(struct xreal_air_slam *s, bool *out_ready, bool *out_tracking, bool *out_failed)
{
	os_mutex_lock(&s->lock);
	*out_ready = s->ready;
	*out_tracking = s->tracking;
	*out_failed = s->failed;
	os_mutex_unlock(&s->lock);
}

static void *
shutdown_thread(void *ptr)
{
	struct xreal_air_slam *s = (struct xreal_air_slam *)ptr;
	xrt_frame_context_destroy_nodes(&s->xfctx);

	os_mutex_lock(&s->lock);
	s->ready = false;
	os_mutex_unlock(&s->lock);
	return NULL;
}

void
xreal_air_slam_destroy(struct xreal_air_slam **s_ptr)
{
	struct xreal_air_slam *s = *s_ptr;
	if (s == NULL) {
		return;
	}
	*s_ptr = NULL;

	u_var_remove_root(s);

	// No more IMU forwarding (the thread may finish creating the tracker first).
	os_thread_helper_destroy(&s->oth);

	SLAM_NOTE(s,
	          "%" PRIu64 " IMU samples and %" PRIu64 " stereo pairs pushed, %" PRIu64
	          " IMU samples dropped, %" PRIu64 " pairs out of order, %" PRIu64
	          " re-initialisations, SLAM acquired %u / lost %u times",
	          s->stats.imu_pushed, s->stats.pairs_pushed, s->stats.imu_dropped, s->stats.pairs_out_of_order,
	          s->stats.discontinuities, s->blend.slam_acquired, s->blend.slam_lost);

	os_mutex_lock(&s->lock);
	const bool ready = s->ready;
	os_mutex_unlock(&s->lock);

	if (ready) {
		// Basalt can hang on shutdown: give it a bounded time, then leave it (and this struct) be.
		pthread_t thread;
		if (pthread_create(&thread, NULL, shutdown_thread, s) != 0) {
			shutdown_thread(s);
		} else {
			const timepoint_ns deadline = os_monotonic_get_ns() + SHUTDOWN_TIMEOUT_NS;
			bool done = false;
			while (!done && os_monotonic_get_ns() < deadline) {
				os_nanosleep(10 * U_TIME_1MS_IN_NS);
				os_mutex_lock(&s->lock);
				done = !s->ready;
				os_mutex_unlock(&s->lock);
			}
			if (!done) {
				SLAM_ERROR(s, "The SLAM tracker did not shut down within %d s, leaving it behind",
				           (int)(SHUTDOWN_TIMEOUT_NS / U_TIME_1S_IN_NS));
				pthread_detach(thread);
				return;
			}
			pthread_join(thread, NULL);
		}
	}

	os_mutex_destroy(&s->lock);
	free(s->imu_queue);
	free(s);
}

#endif // XRT_FEATURE_SLAM
