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
xreal_air_slam_set_display_on(struct xreal_air_slam *s, bool on)
{
	(void)s;
	(void)on;
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

#include "math/m_vec3.h"
#include "xreal_air_slam_guard.h"

#include <dlfcn.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

DEBUG_GET_ONCE_OPTION(xreal_air_slam_config, "XREAL_AIR_SLAM_CONFIG", NULL)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_cam_offset_us, "XREAL_AIR_SLAM_CAM_OFFSET_US", 0)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_stale_ms, "XREAL_AIR_SLAM_STALE_MS", 300)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_fresh_ms, "XREAL_AIR_SLAM_FRESH_MS", 200)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_fresh_hold_ms, "XREAL_AIR_SLAM_FRESH_HOLD_MS", 500)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_warmup_ms, "XREAL_AIR_SLAM_WARMUP_MS", 1500)
DEBUG_GET_ONCE_FLOAT_OPTION(xreal_air_slam_kb4_step_px, "XREAL_AIR_SLAM_KB4_STEP_PX", 8.0f)
DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_slam_wear, "XREAL_AIR_SLAM_WEAR", true)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_wear_debounce_ms, "XREAL_AIR_SLAM_WEAR_DEBOUNCE_MS", 400)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_doff_lookback_ms, "XREAL_AIR_SLAM_DOFF_LOOKBACK_MS", 1000)
DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_slam_divergence, "XREAL_AIR_SLAM_DIVERGENCE", true)
DEBUG_GET_ONCE_FLOAT_OPTION(xreal_air_slam_max_speed, "XREAL_AIR_SLAM_MAX_SPEED", 3.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(xreal_air_slam_still_speed, "XREAL_AIR_SLAM_STILL_SPEED", 0.10f)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_slam_restart_min_ms, "XREAL_AIR_SLAM_RESTART_MIN_MS", 3000)
DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_slam_recreate, "XREAL_AIR_SLAM_RECREATE", true)

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

/*
 * IMU at rest (glasses lying on a desk): 10 ms block means of |gyro| all below 0.03 rad/s and of
 * |accel| within 0.15 m/s^2 over 0.3 s. Worn by someone sitting still this holds ~10% of the time on
 * the recordings (it only matters together with SLAM claiming the head moves, see the guard).
 */
#define STILL_GYRO_MAX 0.03f
#define STILL_ACCEL_RANGE 0.15f

//! Frames are only pushed once the tracker has had IMU samples for this long before them.
#define FIRST_FRAME_IMU_LEAD_NS ((int64_t)100 * U_TIME_1MS_IN_NS)

//! How long a tracker teardown may take (Basalt can hang in ~OpticalFlowBase), then it is left be.
#define SHUTDOWN_TIMEOUT_NS ((int64_t)3 * U_TIME_1S_IN_NS)

//! Repeated log lines (re-initialisations, suspicions, lost/acquired) at most this often.
#define LOG_INTERVAL_NS ((int64_t)2 * U_TIME_1S_IN_NS)

//! Reset in place: poses from before the second frame pushed after the reset may be the old world's.
#define RESET_GATE_FRAMES 2

//! Reset in place while the glasses are off (the tracker keeps running): this often, so it never runs
//! away for long (Basalt crashed once after nine minutes of that).
#define IDLE_RESET_INTERVAL_NS ((int64_t)10 * U_TIME_1S_IN_NS)

//! Output positions kept for looking back (every 25 ms, 128 of them: 3.2 s).
#define POS_HISTORY_INTERVAL_NS ((int64_t)25 * U_TIME_1MS_IN_NS)

//! A log line that may repeat at most every @ref LOG_INTERVAL_NS, counting what it swallowed.
struct rate_limit
{
	int64_t last_ns;
	uint32_t suppressed;
};

static bool
rate_limit_ok(struct rate_limit *r, int64_t now_ns, uint32_t *out_suppressed)
{
	if (r->last_ns != 0 && now_ns - r->last_ns < LOG_INTERVAL_NS) {
		r->suppressed++;
		return false;
	}
	*out_suppressed = r->suppressed;
	r->suppressed = 0;
	r->last_ns = now_ns;
	return true;
}

//! A tracker being torn down on a helper thread, see @ref teardown_bounded.
struct teardown_job
{
	struct xrt_frame_context *xfctx;
	int done; //!< Atomic.
};

struct xreal_air_slam
{
	enum u_logging_level log_level;

	/*!
	 * Creates, restarts and destroys the tracker, and forwards IMU samples to it. Its lock protects
	 * the fields up to @ref lock. Lock order: @ref lock, then this; @ref push_lock, then this.
	 */
	struct os_thread_helper oth;
	struct xrt_imu_sample *imu_queue;
	uint32_t imu_head;
	uint32_t imu_count;
	bool calib_pending;
	bool calib_received;  //!< Only the first calibration counts.
	bool have_calib;      //!< @ref slam_calib is converted, trackers can be created.
	bool restart_pending; //!< Destroy the tracker, then create a new one if @ref want_tracker.
	bool want_tracker;    //!< False while the glasses are off.
	const char *restart_reason;
	bool forwarding;      //!< The tracker is up, IMU samples are queued.
	int64_t first_imu_ts; //!< Of the first sample forwarded to the current tracker, 0 before.
	struct xreal_air_still still;
	bool imu_still;
	struct xreal_air_parsed_calibration calib;

	//! Protects the tracker pointers and the pose state.
	struct os_mutex lock;
	bool ready;
	bool failed;
	struct xrt_frame_context *xfctx; //!< The current tracker's, one per tracker.
	struct xrt_tracked_slam *xts;
	struct t_slam_calibration slam_calib;
	char config_path[PATH_MAX];
	void *vit_library; //!< Keeps the VIT library loaded across tracker restarts.
	/*!
	 * Restart by destroying and creating the tracker (and stop it while the glasses are off). Needs
	 * a VIT library that can create a second tracker in one process (Basalt exporting
	 * basalt_vit_restartable); otherwise the tracker is reset in place (VIT tracker_reset) and keeps
	 * running while the glasses are off.
	 */
	bool recreate;

	struct xreal_air_pose_blend blend;
	struct xreal_air_slam_guard guard;
	struct xreal_air_wear wear;
	struct xreal_air_pos_history history;
	int64_t last_seen_ts;    //!< Of the newest SLAM pose the guard saw.
	int64_t last_restart_ns; //!< When the last restart was asked for.
	bool tracking;           //!< For the log lines on changes.
	bool hold_pending;       //!< The glasses came off: hold the position from before at the next query.
	//! Restarted after a divergence: only follow the new tracker once the IMU has seen the head move
	//! (lying on a desk, a fresh tracker drifts off again).
	bool await_motion;
	struct rate_limit log_reinit, log_suspect, log_tracking;

	//! Held by the camera thread while it pushes a pair, the tracker thread takes it to stop that.
	struct os_mutex push_lock;
	bool push_enabled;
	struct xrt_slam_sinks *push_sinks;
	//! Reset in place: poses older than this may be from the old world (atomic; 0 = no gate).
	int64_t gate_ts;
	//! Frames still to push before @ref gate_ts is known (atomic).
	int gate_frames_left;

	// Camera thread only.
	int64_t last_frame_ts;
	int64_t cam_offset_ns;

	// Settings.
	bool use_wear;
	bool allow_recreate;
	int64_t wear_debounce_ns;
	int64_t doff_lookback_ns;
	int64_t restart_min_ns;

	struct
	{
		uint64_t imu_dropped; //!< Queue full.
		uint64_t imu_pushed;
		uint64_t pairs_pushed;
		uint64_t pairs_out_of_order;
		uint64_t discontinuities; //!< Tracker re-initialisations seen.
		uint64_t divergences;     //!< Trackers declared diverged.
		uint64_t restarts;        //!< Trackers torn down and created again.
		uint64_t doffs;           //!< Times the glasses were taken off.
		float kb4_rms[2];
		float kb4_max[2];
		float last_teardown_ms;
		float last_create_ms;
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
 * Tracker lifecycle, on the tracker thread.
 *
 */

static void
mark_failed(struct xreal_air_slam *s)
{
	os_mutex_lock(&s->lock);
	s->failed = true;
	os_mutex_unlock(&s->lock);
}

static void *
teardown_thread(void *ptr)
{
	struct teardown_job *job = (struct teardown_job *)ptr;
	xrt_frame_context_destroy_nodes(job->xfctx);
	__atomic_store_n(&job->done, 1, __ATOMIC_RELEASE);
	return NULL;
}

/*!
 * Destroy a tracker's frame context (stops and destroys the tracker) on a helper thread, waiting at
 * most @ref SHUTDOWN_TIMEOUT_NS for it. Basalt can hang on shutdown: then it is left behind (the
 * context and its threads leak) and false returned.
 */
static bool
teardown_bounded(struct xreal_air_slam *s, struct xrt_frame_context *xfctx, float *out_ms)
{
	struct teardown_job *job = U_TYPED_CALLOC(struct teardown_job);
	job->xfctx = xfctx;

	const timepoint_ns t0 = os_monotonic_get_ns();
	pthread_t thread;
	if (pthread_create(&thread, NULL, teardown_thread, job) != 0) {
		teardown_thread(job);
	} else {
		const timepoint_ns deadline = t0 + SHUTDOWN_TIMEOUT_NS;
		while (__atomic_load_n(&job->done, __ATOMIC_ACQUIRE) == 0 && os_monotonic_get_ns() < deadline) {
			os_nanosleep(2 * U_TIME_1MS_IN_NS);
		}
		if (__atomic_load_n(&job->done, __ATOMIC_ACQUIRE) == 0) {
			SLAM_ERROR(s, "The SLAM tracker did not shut down within %d s, leaving it behind",
			           (int)(SHUTDOWN_TIMEOUT_NS / U_TIME_1S_IN_NS));
			pthread_detach(thread);
			*out_ms = (float)((double)(os_monotonic_get_ns() - t0) / U_TIME_1MS_IN_NS);
			return false;
		}
		pthread_join(thread, NULL);
	}
	*out_ms = (float)((double)(os_monotonic_get_ns() - t0) / U_TIME_1MS_IN_NS);

	free(job);
	free(xfctx);
	return true;
}

//! Creates and starts a tracker from @ref xreal_air_slam::slam_calib and publishes it.
static bool
create_tracker(struct xreal_air_slam *s, struct xrt_slam_sinks **out_sinks)
{
	struct t_slam_tracker_config config;
	t_slam_fill_default_config(&config);
	config.cam_count = XREAL_AIR_SLAM_CAMERA_COUNT;
	config.slam_calib = &s->slam_calib;
	config.send_calib_with_config_file = true;
	config.submit_from_start = true;

	resolve_config_path(s, config.vit_system_library_path);
	config.slam_config = s->config_path[0] != '\0' ? s->config_path : NULL;

	if (s->vit_library == NULL) {
		SLAM_NOTE(s, "Creating the tracker: VIT library '%s', pipeline config '%s'",
		          config.vit_system_library_path,
		          config.slam_config != NULL ? config.slam_config : "(Basalt defaults)");
	}

	struct xrt_frame_context *xfctx = U_TYPED_CALLOC(struct xrt_frame_context);
	struct xrt_tracked_slam *xts = NULL;
	struct xrt_slam_sinks *sinks = NULL;
	const timepoint_ns t0 = os_monotonic_get_ns();
	if (t_slam_create(xfctx, &config, &xts, &sinks) != 0 || xts == NULL || sinks == NULL) {
		SLAM_ERROR(s,
		           "Could not create the SLAM tracker (VIT_SYSTEM_LIBRARY_PATH set to a libbasalt.so?), "
		           "staying 3DoF");
		xrt_frame_context_destroy_nodes(xfctx);
		free(xfctx);
		return false;
	}
	if (t_slam_start(xts) != 0) {
		SLAM_ERROR(s, "Could not start the SLAM tracker, staying 3DoF");
		xrt_frame_context_destroy_nodes(xfctx);
		free(xfctx);
		return false;
	}
	s->stats.last_create_ms = (float)((double)(os_monotonic_get_ns() - t0) / U_TIME_1MS_IN_NS);

	// t_slam unloads the VIT library with each tracker; keep it loaded so restarts do not reload it.
	bool first = false;
	bool recreate = false;
	if (s->vit_library == NULL && config.vit_system_library_path != NULL) {
		first = true;
		s->vit_library = dlopen(config.vit_system_library_path, RTLD_NOW | RTLD_NOLOAD);
		recreate = s->allow_recreate && s->vit_library != NULL &&
		           dlsym(s->vit_library, "basalt_vit_restartable") != NULL;
		SLAM_NOTE(s, "Tracker restarts: %s",
		          recreate ? "a fresh tracker each time, stopped while the glasses are off"
		                   : "reset in place (the VIT library cannot create a second tracker), kept "
		                     "running while the glasses are off");
	}

	os_mutex_lock(&s->lock);
	if (first) {
		s->recreate = recreate;
	}
	s->xfctx = xfctx;
	s->xts = xts;
	s->ready = true;
	s->last_seen_ts = 0;
	xreal_air_slam_guard_restart(&s->guard);
	os_mutex_unlock(&s->lock);

	os_mutex_lock(&s->push_lock);
	s->push_sinks = sinks;
	s->push_enabled = true;
	__atomic_store_n(&s->gate_frames_left, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&s->gate_ts, 0, __ATOMIC_RELEASE);
	os_mutex_unlock(&s->push_lock);

	*out_sinks = sinks;
	return true;
}

/*!
 * Tear the current tracker down (if any) and create a fresh one if @p want: a fresh map, a fresh
 * world. Off the pose path: the pose calls see no tracker meanwhile (3DoF, position held).
 */
static void
restart_tracker(struct xreal_air_slam *s, bool want, const char *reason, struct xrt_slam_sinks **inout_sinks)
{
	// Stop the camera thread pushing, waiting for a pair in flight.
	os_mutex_lock(&s->push_lock);
	s->push_enabled = false;
	s->push_sinks = NULL;
	os_mutex_unlock(&s->push_lock);
	*inout_sinks = NULL;

	os_mutex_lock(&s->lock);
	struct xrt_frame_context *xfctx = s->xfctx;
	const bool first = !s->ready && s->xfctx == NULL && s->stats.restarts == 0 && s->vit_library == NULL;
	s->xfctx = NULL;
	s->xts = NULL;
	s->ready = false;
	os_mutex_unlock(&s->lock);

	float teardown_ms = 0;
	if (xfctx != NULL) {
		teardown_bounded(s, xfctx, &teardown_ms);
		s->stats.last_teardown_ms = teardown_ms;
	}

	if (!want) {
		if (xfctx != NULL) {
			SLAM_NOTE(s, "Tracker stopped (%s) in %.0f ms", reason, teardown_ms);
		}
		return;
	}

	if (!create_tracker(s, inout_sinks)) {
		mark_failed(s);
		return;
	}

	if (first) {
		SLAM_NOTE(s, "Tracker running (created in %.0f ms), waiting for camera frames",
		          s->stats.last_create_ms);
	} else {
		s->stats.restarts++;
		SLAM_NOTE(s, "Fresh tracker (%s): old one stopped in %.0f ms, new one created in %.0f ms", reason,
		          teardown_ms, s->stats.last_create_ms);
	}
}

static void *
tracker_thread(void *ptr)
{
	struct xreal_air_slam *s = (struct xreal_air_slam *)ptr;
	os_thread_helper_name(&s->oth, "XREAL Air SLAM");

	struct xrt_imu_sample batch[IMU_BATCH];
	struct xrt_slam_sinks *sinks = NULL; // This thread's view of the current tracker.

	os_thread_helper_lock(&s->oth);
	while (os_thread_helper_is_running_locked(&s->oth)) {
		if (s->calib_pending) {
			s->calib_pending = false;
			struct xreal_air_parsed_calibration calib = s->calib;
			os_thread_helper_unlock(&s->oth);

			const bool ok = fill_slam_calibration(s, &calib);
			if (!ok) {
				mark_failed(s);
			}

			os_thread_helper_lock(&s->oth);
			s->have_calib = ok;
			if (ok) {
				s->restart_pending = true;
				if (s->restart_reason == NULL) {
					s->restart_reason = "start";
				}
			}
			continue;
		}

		if (s->have_calib && s->restart_pending) {
			s->restart_pending = false;
			const bool want = s->want_tracker;
			const char *reason = s->restart_reason != NULL ? s->restart_reason : "restart";
			s->restart_reason = NULL;
			s->forwarding = false;
			s->imu_count = 0;
			s->first_imu_ts = 0;
			os_thread_helper_unlock(&s->oth);

			restart_tracker(s, want, reason, &sinks);

			os_thread_helper_lock(&s->oth);
			s->forwarding = sinks != NULL;
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
		for (uint32_t i = 0; i < n && sinks != NULL; i++) {
			xrt_sink_push_imu(sinks->imu, &batch[i]);
		}

		os_thread_helper_lock(&s->oth);
		s->stats.imu_pushed += n;
	}
	os_thread_helper_unlock(&s->oth);

	return NULL;
}

/*!
 * Called with @ref xreal_air_slam::lock held: reset the running tracker in place. Its poses from before
 * the reset took effect are ignored (see @ref gate_ts), the guard starts a new world.
 */
static void
reset_in_place_locked(struct xreal_air_slam *s, int64_t now_ns, const char *reason, bool quiet)
{
	if (s->xts == NULL) {
		return;
	}
	__atomic_store_n(&s->gate_ts, INT64_MAX, __ATOMIC_RELEASE);
	__atomic_store_n(&s->gate_frames_left, RESET_GATE_FRAMES, __ATOMIC_RELEASE);
	t_slam_reset(s->xts);
	xreal_air_slam_guard_restart(&s->guard);
	s->last_seen_ts = 0;
	s->last_restart_ns = now_ns;
	s->stats.restarts++;
	if (quiet) {
		SLAM_INFO(s, "Tracker reset (%s)", reason);
	} else {
		SLAM_NOTE(s, "Tracker reset (%s)", reason);
	}
}

//! Called with @ref xreal_air_slam::lock held.
static void
request_restart_locked(struct xreal_air_slam *s, int64_t now_ns, const char *reason, bool want_tracker)
{
	s->last_restart_ns = now_ns;

	if (!s->recreate && s->xts != NULL) {
		// Off: keep it running (its output is not used), it is reset when the glasses are back on.
		if (want_tracker) {
			reset_in_place_locked(s, now_ns, reason, false);
		}
		return;
	}

	os_thread_helper_lock(&s->oth);
	s->want_tracker = want_tracker;
	s->restart_pending = true;
	s->restart_reason = reason;
	os_thread_helper_signal_locked(&s->oth);
	os_thread_helper_unlock(&s->oth);
}


/*
 *
 * Exported functions.
 *
 */

static void
poll_wear_locked(struct xreal_air_slam *s, int64_t now);

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
	s->want_tracker = true;
	s->use_wear = debug_get_bool_option_xreal_air_slam_wear();
	s->allow_recreate = debug_get_bool_option_xreal_air_slam_recreate();
	s->wear_debounce_ns = (int64_t)debug_get_num_option_xreal_air_slam_wear_debounce_ms() * U_TIME_1MS_IN_NS;
	s->doff_lookback_ns = (int64_t)debug_get_num_option_xreal_air_slam_doff_lookback_ms() * U_TIME_1MS_IN_NS;
	s->restart_min_ns = (int64_t)debug_get_num_option_xreal_air_slam_restart_min_ms() * U_TIME_1MS_IN_NS;

	struct xreal_air_slam_guard_config cfg;
	xreal_air_slam_guard_default_config(&cfg);
	cfg.stale_ns = (int64_t)debug_get_num_option_xreal_air_slam_stale_ms() * U_TIME_1MS_IN_NS;
	cfg.fresh_ns = (int64_t)debug_get_num_option_xreal_air_slam_fresh_ms() * U_TIME_1MS_IN_NS;
	cfg.fresh_hold_ns = (int64_t)debug_get_num_option_xreal_air_slam_fresh_hold_ms() * U_TIME_1MS_IN_NS;
	cfg.warmup_ns = (int64_t)debug_get_num_option_xreal_air_slam_warmup_ms() * U_TIME_1MS_IN_NS;
	cfg.max_speed_m_s = debug_get_float_option_xreal_air_slam_max_speed();
	cfg.still_speed_m_s = debug_get_float_option_xreal_air_slam_still_speed();
	const bool divergence = debug_get_bool_option_xreal_air_slam_divergence();
	if (!divergence) {
		// The old behaviour: every discontinuity is a re-initialisation, any speed goes.
		cfg.max_speed_m_s = INFINITY;
		cfg.still_speed_m_s = INFINITY;
		cfg.jump_repeat_ns = 0;
	}
	xreal_air_slam_guard_init(&s->guard, &cfg);
	xreal_air_wear_init(&s->wear);
	xreal_air_still_init(&s->still, STILL_GYRO_MAX, STILL_ACCEL_RANGE);
	xreal_air_pos_history_init(&s->history, POS_HISTORY_INTERVAL_NS);
	xreal_air_pose_blend_init(&s->blend);

	if (os_mutex_init(&s->lock) != 0 || os_mutex_init(&s->push_lock) != 0 || os_thread_helper_init(&s->oth) != 0 ||
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
	u_var_add_ro_u64(s, &s->stats.divergences, "Tracker divergences");
	u_var_add_ro_u64(s, &s->stats.restarts, "Tracker restarts");
	u_var_add_ro_u64(s, &s->stats.doffs, "Glasses taken off");
	u_var_add_ro_u32(s, &s->blend.slam_acquired, "SLAM acquired");
	u_var_add_ro_u32(s, &s->blend.slam_lost, "SLAM lost");
	u_var_add_ro_f32(s, &s->guard.last_speed, "SLAM speed (m/s)");
	u_var_add_bool(s, &s->imu_still, "IMU at rest (read only)");
	u_var_add_bool(s, &s->tracking, "Tracking (read only)");
	u_var_add_pose(s, &s->blend.last_out, "Last head pose");

	SLAM_NOTE(
	    s,
	    "6DoF requested (XREAL_AIR_SLAM=1), the tracker starts once the factory calibration is in. "
	    "Mid-exposure frame stamps%s, lost after %" PRId64 " ms without a pose (back under %" PRId64
	    " ms for %" PRId64 " ms), warm-up %" PRId64 " ms, %s, divergence detection %s",
	    s->cam_offset_ns != 0 ? " plus an extra offset" : "", cfg.stale_ns / U_TIME_1MS_IN_NS,
	    cfg.fresh_ns / U_TIME_1MS_IN_NS, cfg.fresh_hold_ns / U_TIME_1MS_IN_NS, cfg.warmup_ns / U_TIME_1MS_IN_NS,
	    s->use_wear ? "paused while the glasses are off" : "ignoring the wear sensor", divergence ? "on" : "off");

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
xreal_air_slam_set_display_on(struct xreal_air_slam *s, bool on)
{
	os_mutex_lock(&s->lock);
	xreal_air_wear_raw(&s->wear, on, (int64_t)os_monotonic_get_ns());
	os_mutex_unlock(&s->lock);
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
	s->imu_still = xreal_air_still_push(&s->still, timestamp_ns, gyro_rad_s, accel_m_s2);
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

	// The wear state also changes while nothing asks for poses.
	os_mutex_lock(&s->lock);
	poll_wear_locked(s, (int64_t)os_monotonic_get_ns());
	os_mutex_unlock(&s->lock);

	// Held through the push, so that a tracker restart waits for it.
	os_mutex_lock(&s->push_lock);
	if (!s->push_enabled || s->push_sinks == NULL) {
		os_mutex_unlock(&s->push_lock);
		return;
	}
	struct xrt_slam_sinks *sinks = s->push_sinks;

	// Basalt wants IMU data from before the first frame (it initialises from the accelerometer).
	os_thread_helper_lock(&s->oth);
	const int64_t first_imu_ts = s->first_imu_ts;
	os_thread_helper_unlock(&s->oth);
	if (first_imu_ts == 0 || timestamp_ns < first_imu_ts + FIRST_FRAME_IMU_LEAD_NS) {
		os_mutex_unlock(&s->push_lock);
		return;
	}

	if (timestamp_ns <= s->last_frame_ts) {
		s->stats.pairs_out_of_order++;
		os_mutex_unlock(&s->push_lock);
		return;
	}
	s->last_frame_ts = timestamp_ns;

	// The frames are only read by the capture's own sinks otherwise, which are done with them.
	left->timestamp = (uint64_t)timestamp_ns;
	right->timestamp = (uint64_t)timestamp_ns;

	// Always both, in this order: Basalt expects cam0 then cam1 with the same timestamp.
	xrt_sink_push_frame(sinks->cams[0], left);
	xrt_sink_push_frame(sinks->cams[1], right);

	if (__atomic_load_n(&s->gate_frames_left, __ATOMIC_ACQUIRE) > 0 &&
	    __atomic_sub_fetch(&s->gate_frames_left, 1, __ATOMIC_ACQ_REL) == 0) {
		__atomic_store_n(&s->gate_ts, timestamp_ns, __ATOMIC_RELEASE);
	}

	if (s->stats.pairs_pushed++ == 0) {
		SLAM_NOTE(s, "First stereo pair pushed to the tracker");
	}
	os_mutex_unlock(&s->push_lock);
}

//! Called with the lock held: log and act on what the guard made of a new SLAM pose.
static void
handle_verdict_locked(struct xreal_air_slam *s, int64_t now, enum xreal_air_slam_pose_verdict v, int64_t gap_ns)
{
	const struct xreal_air_slam_guard *g = &s->guard;
	uint32_t suppressed = 0;

	switch (v) {
	case XREAL_AIR_SLAM_POSE_SAME:
	case XREAL_AIR_SLAM_POSE_OK: break;
	case XREAL_AIR_SLAM_POSE_FIRST: SLAM_NOTE(s, "First SLAM pose, warming up"); break;
	case XREAL_AIR_SLAM_POSE_RESUMED:
		SLAM_NOTE(s, "SLAM poses again after %.1f s, warming up", (double)gap_ns / U_TIME_1S_IN_NS);
		break;
	case XREAL_AIR_SLAM_POSE_REINIT:
		s->stats.discontinuities++;
		if (rate_limit_ok(&s->log_reinit, now, &suppressed)) {
			SLAM_NOTE(
			    s,
			    "SLAM pose jumped %.2f m / %.0f deg: the tracker re-initialised, re-anchoring once it "
			    "settles (%u more not logged)",
			    g->last_dist, g->last_angle_deg, suppressed);
		}
		break;
	case XREAL_AIR_SLAM_POSE_SUSPECT:
		if (rate_limit_ok(&s->log_suspect, now, &suppressed)) {
			SLAM_NOTE(s, "SLAM claims %.2f m/s (%s), not following it (%u more not logged)", g->last_speed,
			          g->reason != NULL ? g->reason : "?", suppressed);
		}
		break;
	case XREAL_AIR_SLAM_POSE_DIVERGED:
		if (s->last_restart_ns != 0 && now - s->last_restart_ns < s->restart_min_ns) {
			break; // Not again so soon, the guard stays latched until the restart.
		}
		s->stats.divergences++;
		SLAM_NOTE(s, "SLAM diverged (%s, %.2f m/s, last step %.2f m), starting a fresh tracker",
		          g->reason != NULL ? g->reason : "?", g->last_speed, g->last_dist);
		request_restart_locked(s, now, "diverged", true);
		s->await_motion = true;
		break;
	}
}

//! Called with the lock held. The fresh SLAM head relation at @p at_ns, false if not to be followed.
static bool
get_slam_head_locked(struct xreal_air_slam *s, int64_t now, int64_t at_ns, struct xrt_space_relation *out_head)
{
	if (!s->ready || s->xts == NULL) {
		return false;
	}

	timepoint_ns latest_ts = 0;
	struct xrt_space_relation latest = XRT_SPACE_RELATION_ZERO;
	if (!t_slam_get_latest_pose(s->xts, &latest_ts, &latest)) {
		return false;
	}
	if (latest_ts < __atomic_load_n(&s->gate_ts, __ATOMIC_ACQUIRE)) {
		return false; // Reset in place, this pose may still be from the old world.
	}

	if (latest_ts != s->last_seen_ts) {
		s->last_seen_ts = latest_ts;

		os_thread_helper_lock(&s->oth);
		const bool imu_still = s->imu_still;
		os_thread_helper_unlock(&s->oth);

		const bool have_vel = (latest.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0;
		const int64_t gap_ns = s->guard.have_pose ? latest_ts - s->guard.last_ts : 0;
		const enum xreal_air_slam_pose_verdict v = xreal_air_slam_guard_push(
		    &s->guard, latest_ts, &latest.pose, have_vel ? &latest.linear_velocity : NULL, imu_still);
		handle_verdict_locked(s, now, v, gap_ns);
	}

	if (!xreal_air_slam_guard_usable(&s->guard, now, latest_ts)) {
		return false;
	}
	if (s->await_motion) {
		os_thread_helper_lock(&s->oth);
		const bool imu_still = s->imu_still;
		os_thread_helper_unlock(&s->oth);
		if (imu_still) {
			return false;
		}
		s->await_motion = false;
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

/*!
 * Called with the lock held, from the pose queries and the camera thread (so that it also happens
 * while nothing asks for poses): act on the glasses being taken off / put on.
 */
static void
poll_wear_locked(struct xreal_air_slam *s, int64_t now)
{
	if (!s->use_wear) {
		return;
	}

	if (xreal_air_wear_update(&s->wear, now, s->wear_debounce_ns)) {
		if (s->wear.worn) {
			s->hold_pending = false;
			s->await_motion = false;
			SLAM_NOTE(s, "Glasses put on: %s, 6DoF again once it has settled",
			          s->recreate || s->xts == NULL ? "starting a fresh tracker" : "resetting the tracker");
			request_restart_locked(s, now, "glasses put on", true);
		} else {
			s->stats.doffs++;
			s->hold_pending = true;
			SLAM_NOTE(s, "Glasses taken off: 3DoF with the position held, %s",
			          s->recreate || s->xts == NULL ? "stopping the tracker"
			                                        : "ignoring the tracker until they are back on");
			request_restart_locked(s, now, "glasses taken off", false);
		}
		return;
	}

	// Kept running while off: do not let it run away for long.
	if (!s->wear.worn && !s->recreate && s->xts != NULL && now - s->last_restart_ns >= IDLE_RESET_INTERVAL_NS) {
		reset_in_place_locked(s, now, "glasses off", true);
	}
}

/*!
 * Called with the lock held. Taking the glasses off moved them before the sensor noticed, and the
 * output followed SLAM through that: hold the position from before, where the head was (and
 * probably will be again).
 */
static void
apply_doff_hold_locked(struct xreal_air_slam *s, int64_t now, int64_t at_ns, const struct xrt_space_relation *fusion)
{
	s->hold_pending = false;
	const int64_t before = s->wear.worn_change_ns - s->doff_lookback_ns;
	struct xrt_vec3 pos = s->blend.last_out.position;
	xreal_air_pos_history_at(&s->history, before, &pos);
	const struct xrt_vec3 d = m_vec3_sub(pos, s->blend.last_out.position);
	xreal_air_pose_blend_hold(&s->blend, at_ns, fusion, &pos);
	SLAM_NOTE(s, "Holding the position from %.1f s before (%.1f cm back)", (double)(now - before) / U_TIME_1S_IN_NS,
	          m_vec3_len(d) * 100.0f);
}

void
xreal_air_slam_get_head_pose(struct xreal_air_slam *s,
                             int64_t at_timestamp_ns,
                             const struct xrt_space_relation *fusion,
                             struct xrt_space_relation *out)
{
	os_mutex_lock(&s->lock);
	const int64_t now = (int64_t)os_monotonic_get_ns();

	poll_wear_locked(s, now);
	if (s->hold_pending) {
		apply_doff_hold_locked(s, now, at_timestamp_ns, fusion);
	}

	struct xrt_space_relation head;
	const bool worn = !s->use_wear || s->wear.worn;
	const bool tracking = worn && get_slam_head_locked(s, now, at_timestamp_ns, &head);
	xreal_air_pose_blend_update(&s->blend, at_timestamp_ns, tracking ? &head : NULL, s->guard.world_id, fusion,
	                            out);
	xreal_air_pos_history_push(&s->history, now, &out->pose.position);

	if (tracking != s->tracking) {
		s->tracking = tracking;
		uint32_t suppressed = 0;
		if (rate_limit_ok(&s->log_tracking, now, &suppressed)) {
			if (tracking) {
				SLAM_NOTE(s, "6DoF tracking (SLAM acquired, #%u; %u changes not logged)",
				          s->blend.slam_acquired, suppressed);
			} else if (s->blend.slam_lost > 0) {
				SLAM_NOTE(
				    s,
				    "Lost SLAM tracking (#%u, newest pose %.0f ms old), 3DoF with the position held "
				    "(%u changes not logged)",
				    s->blend.slam_lost, (double)s->guard.last_lag_ns / U_TIME_1MS_IN_NS, suppressed);
			}
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

void
xreal_air_slam_destroy(struct xreal_air_slam **s_ptr)
{
	struct xreal_air_slam *s = *s_ptr;
	if (s == NULL) {
		return;
	}
	*s_ptr = NULL;

	u_var_remove_root(s);

	// No more IMU forwarding (the thread may finish creating or restarting the tracker first).
	os_thread_helper_destroy(&s->oth);

	SLAM_NOTE(s,
	          "%" PRIu64 " IMU samples and %" PRIu64 " stereo pairs pushed, %" PRIu64
	          " IMU samples dropped, %" PRIu64 " pairs out of order, %" PRIu64 " re-initialisations, %" PRIu64
	          " divergences, %" PRIu64 " restarts, taken off %" PRIu64 " times, SLAM acquired %u / lost %u times",
	          s->stats.imu_pushed, s->stats.pairs_pushed, s->stats.imu_dropped, s->stats.pairs_out_of_order,
	          s->stats.discontinuities, s->stats.divergences, s->stats.restarts, s->stats.doffs,
	          s->blend.slam_acquired, s->blend.slam_lost);

	os_mutex_lock(&s->push_lock);
	s->push_enabled = false;
	s->push_sinks = NULL;
	os_mutex_unlock(&s->push_lock);

	os_mutex_lock(&s->lock);
	struct xrt_frame_context *xfctx = s->xfctx;
	s->xfctx = NULL;
	s->xts = NULL;
	s->ready = false;
	os_mutex_unlock(&s->lock);

	bool clean = true;
	if (xfctx != NULL) {
		float ms = 0;
		clean = teardown_bounded(s, xfctx, &ms);
		SLAM_INFO(s, "Tracker shut down in %.0f ms", ms);
	}
	if (clean && s->vit_library != NULL) {
		dlclose(s->vit_library); // A tracker left behind still runs its code, keep it loaded then.
	}

	os_mutex_destroy(&s->push_lock);
	os_mutex_destroy(&s->lock);
	free(s->imu_queue);
	free(s);
}

#endif // XRT_FEATURE_SLAM
