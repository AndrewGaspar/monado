// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra opt-in SLAM camera capture and dataset recording.
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_capture.h"
#include "xreal_air_camera.h"
#include "xreal_air_camera_decode.h"
#include "xreal_air_record.h"

#include "math/m_clock_tracking.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_misc.h"
#include "util/u_sink.h"
#include "util/u_var.h"
#include "xrt/xrt_frame.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_cameras, "XREAL_AIR_CAMERAS", false)
DEBUG_GET_ONCE_OPTION(xreal_air_euroc_record, "XREAL_AIR_EUROC_RECORD", NULL)
DEBUG_GET_ONCE_OPTION(xreal_air_camera_device, "XREAL_AIR_CAMERA_DEVICE", NULL)
/*
 * The camera telemetry and the IMU HID timestamps count the same device clock: on a 3 minute
 * recording both drift identically against the host (-51 ppm) and their epochs agree to within
 * the camera's transport latency (~26 ms). So by default camera frames are timed by mapping their
 * device time through the IMU's clock tracker, which puts them on exactly the IMU's timeline.
 * XREAL_AIR_CAMERA_OWN_CLOCK=1 maps them through their own tracker instead (each stream then
 * carries its own transport latency, ~26 ms apart).
 */
DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_camera_own_clock, "XREAL_AIR_CAMERA_OWN_CLOCK", false)

#define CAP_INFO(c, ...) U_LOG_IFL_I((c)->log_level, "Xreal Air capture: " __VA_ARGS__)
// One-shot lifecycle / diagnostics lines, at WARN so they show at the driver's default log level.
#define CAP_NOTE(c, ...) U_LOG_IFL_W((c)->log_level, "Xreal Air capture: " __VA_ARGS__)
#define CAP_WARN(c, ...) U_LOG_IFL_W((c)->log_level, "Xreal Air capture: " __VA_ARGS__)
#define CAP_ERROR(c, ...) U_LOG_IFL_E((c)->log_level, "Xreal Air capture: " __VA_ARGS__)

//! The camera node may show up after the HID interfaces, keep looking for this long.
#define CAMERA_OPEN_ATTEMPTS 60
#define CAMERA_OPEN_INTERVAL_NS (1000 * U_TIME_1MS_IN_NS)

//! Clock tracker windows: ~1 s of camera frames, ~0.5 s of IMU samples.
#define CAMERA_CLOCK_WINDOW 64
#define IMU_CLOCK_WINDOW 512

/*!
 * A device time mapped onto the host clock is only trusted if it lands at most this long before
 * (and not after) its host arrival time. Otherwise the arrival time minus a typical transport
 * latency is used instead, see @ref map_device_time.
 */
#define MAX_MAPPED_LATENCY_NS (500 * U_TIME_1MS_IN_NS)
#define MAX_MAPPED_LEAD_NS (20 * U_TIME_1MS_IN_NS) // The tracker smooths, it can briefly lead.
//! Measured on the Air 2 Ultra: frames arrive ~8.8 ms after their device time, IMU samples < 1 ms.
#define CAMERA_FALLBACK_LATENCY_NS (9 * U_TIME_1MS_IN_NS)
#define IMU_FALLBACK_LATENCY_NS (1 * U_TIME_1MS_IN_NS)

//! When to log the clock diagnostics.
#define IMU_RATE_CHECK_SAMPLES 2000
#define CAMERA_CLOCK_CHECK_FRAMES 600

struct xreal_air_capture
{
	enum u_logging_level log_level;

	//! Opens the camera (retrying until it shows up), then idles until destroyed.
	struct os_thread_helper oth;
	struct xreal_air_camera *camera;
	char camera_path[64];

	//! Protects everything below that is not marked otherwise.
	struct os_mutex lock;

	struct m_clock_windowed_skew_tracker *imu_clock;
	struct m_clock_windowed_skew_tracker *cam_clock;
	int64_t last_imu_euroc_ns;

	struct xreal_air_record *record;
	char record_base[1024];

	char *calib_document;
	size_t calib_document_len;
	struct xreal_air_parsed_calibration calib;
	bool calib_valid;

	struct
	{
		uint64_t count;
		uint64_t first_device_ts;
		int64_t first_host_ns;
		bool checked;
	} imu_rate;

	// Camera thread only.
	struct
	{
		struct xrt_frame *frame;
		struct xreal_air_record_cam_frame info;
	} pending[2];
	int64_t last_pair_euroc_ns;

	struct
	{
		struct u_sink_debug sinks[2];
		struct u_var_button record_btn;
		uint64_t frames;
		uint64_t pairs;
		uint64_t unpaired;
		uint64_t warmup; //!< Pairs with zero exposure, kept out of the dataset.
		uint64_t imu_samples;
		int64_t cam_minus_imu_clock_ns; //!< Camera clock minus IMU clock mapping of the same instant.
		uint64_t cam_fallbacks;         //!< Frames whose device time could not be mapped.
		uint64_t imu_fallbacks;         //!< IMU samples whose device time could not be mapped.
		uint64_t pairs_own_clock;       //!< Pairs timed through the camera's own clock mapping.
	} gui;
};


/*
 *
 * Clock mapping.
 *
 */

/*!
 * Map a device timestamp onto CLOCK_MONOTONIC through @p tracker, checking the result against the
 * host arrival time. On failure (tracker not ready, or a nonsensical result) fall back to the
 * arrival time minus @p fallback_latency_ns, warn once per stream and count it.
 *
 * @return The mapped time, always valid.
 */
static int64_t
map_device_time(struct xreal_air_capture *cap,
                struct m_clock_windowed_skew_tracker *tracker,
                const char *stream,
                int64_t device_ns,
                int64_t arrival_ns,
                int64_t fallback_latency_ns,
                uint64_t *fallbacks)
{
	timepoint_ns mapped = 0;
	if (m_clock_windowed_skew_tracker_to_local(tracker, device_ns, &mapped) &&
	    mapped <= arrival_ns + MAX_MAPPED_LEAD_NS && arrival_ns - mapped <= MAX_MAPPED_LATENCY_NS) {
		return mapped;
	}

	if (*fallbacks == 0) {
		CAP_NOTE(cap,
		         "Could not map the %s device time %" PRId64 " (arrival %" PRId64 ", tracker %s %" PRId64
		         "), falling back to arrival - %.1f ms. Further fallbacks are only counted.",
		         stream, device_ns, arrival_ns, mapped != 0 ? "gave" : "not ready,", (int64_t)mapped,
		         (double)fallback_latency_ns / U_TIME_1MS_IN_NS);
	}
	(*fallbacks)++;

	return arrival_ns - fallback_latency_ns;
}


/*!
 * The EuRoC timestamp of a pair from its left frame: the device time through the IMU clock
 * (see XREAL_AIR_CAMERA_OWN_CLOCK), as long as that is sane against the arrival time, else the
 * camera's own mapping.
 */
static int64_t
pair_timestamp(struct xreal_air_capture *cap, const struct xreal_air_record_cam_frame *left)
{
	const int64_t arrival = left->kernel_ns != 0 ? left->kernel_ns : left->dequeue_ns;

	if (!debug_get_bool_option_xreal_air_camera_own_clock() && left->imu_clock_ns > 0 &&
	    left->imu_clock_ns <= arrival + MAX_MAPPED_LEAD_NS &&
	    arrival - left->imu_clock_ns <= MAX_MAPPED_LATENCY_NS) {
		return left->imu_clock_ns;
	}

	if (cap->gui.pairs_own_clock++ == 0) {
		CAP_NOTE(cap, "Timing a stereo pair with the camera's own clock mapping (IMU clock %s)",
		         debug_get_bool_option_xreal_air_camera_own_clock() ? "disabled by XREAL_AIR_CAMERA_OWN_CLOCK"
		                                                            : "not usable yet");
	}
	return left->mapped_ns;
}


/*
 *
 * Recording control.
 *
 */

static void
make_record_path(const char *base, bool exact, char *out, size_t out_size)
{
	if (exact) {
		(void)snprintf(out, out_size, "%s", base);
		return;
	}

	char datetime[32];
	time_t now = time(NULL);
	struct tm tm;
	(void)localtime_r(&now, &tm);
	(void)strftime(datetime, sizeof(datetime), "%Y%m%d_%H%M%S", &tm);
	(void)snprintf(out, out_size, "%s_%s", base, datetime);
}

//! Start a recording, must be called without the lock held.
static void
start_record(struct xreal_air_capture *cap, bool exact_path)
{
	char path[PATH_MAX];
	make_record_path(cap->record_base, exact_path, path, sizeof(path));

	struct xreal_air_record *record = xreal_air_record_create(path, cap->log_level);
	if (record == NULL && exact_path) {
		// The exact path is taken, fall back to a fresh one next to it.
		make_record_path(cap->record_base, false, path, sizeof(path));
		record = xreal_air_record_create(path, cap->log_level);
	}
	if (record == NULL) {
		return;
	}

	os_mutex_lock(&cap->lock);
	if (cap->calib_valid) {
		xreal_air_record_set_calibration(record, cap->calib_document, cap->calib_document_len, &cap->calib);
	}
	struct xreal_air_record *old = cap->record;
	cap->record = record;
	os_mutex_unlock(&cap->lock);

	xreal_air_record_destroy(&old);
	CAP_WARN(cap, "Recording EuRoC dataset to '%s'", path);
}

static void
stop_record(struct xreal_air_capture *cap)
{
	os_mutex_lock(&cap->lock);
	struct xreal_air_record *old = cap->record;
	cap->record = NULL;
	os_mutex_unlock(&cap->lock);

	if (old != NULL) {
		CAP_WARN(cap, "Stopped recording '%s'", xreal_air_record_path(old));
	}
	xreal_air_record_destroy(&old);
}

static void
record_btn_cb(void *ptr)
{
	struct xreal_air_capture *cap = (struct xreal_air_capture *)ptr;

	os_mutex_lock(&cap->lock);
	const bool recording = cap->record != NULL;
	os_mutex_unlock(&cap->lock);

	if (recording) {
		stop_record(cap);
		(void)snprintf(cap->gui.record_btn.label, sizeof(cap->gui.record_btn.label), "Record EuRoC dataset");
	} else {
		start_record(cap, false);
		(void)snprintf(cap->gui.record_btn.label, sizeof(cap->gui.record_btn.label), "Stop recording");
	}
}


/*
 *
 * Camera frames, on the camera thread.
 *
 */

static void
drop_pending(struct xreal_air_capture *cap, int i)
{
	if (cap->pending[i].frame != NULL) {
		cap->gui.unpaired++;
	}
	xrt_frame_reference(&cap->pending[i].frame, NULL);
}

static void
on_camera_frame(void *user, const struct xreal_air_camera_raw_frame *raw)
{
	struct xreal_air_capture *cap = (struct xreal_air_capture *)user;

	struct xreal_air_record_cam_frame info;
	U_ZERO(&info);
	if (!xreal_air_cam_parse_telemetry(raw->data, raw->size, &info.telemetry) ||
	    raw->size < XREAL_AIR_CAM_IMAGE_SIZE) {
		return;
	}

	info.cam = xreal_air_cam_identify(&info.telemetry);
	info.v4l2_sequence = raw->v4l2_sequence;
	info.kernel_ns = raw->kernel_ns;
	info.dequeue_ns = raw->dequeue_ns;
	info.mapped_ns = -1;
	info.imu_clock_ns = -1;
	info.euroc_ns = -1;

	const timepoint_ns device_ns = (timepoint_ns)info.telemetry.device_ns;
	const timepoint_ns host_ns = raw->kernel_ns != 0 ? raw->kernel_ns : raw->dequeue_ns;

	struct xrt_frame *xf = NULL;
	u_frame_create_one_off(XRT_FORMAT_L8, XREAL_AIR_CAM_WIDTH, XREAL_AIR_CAM_HEIGHT, &xf);
	if (xf == NULL) {
		return;
	}
	info.phase = xreal_air_cam_decode(raw->data, info.cam, xf->data, xf->stride);

	os_mutex_lock(&cap->lock);

	m_clock_windowed_skew_tracker_push(cap->cam_clock, host_ns, device_ns);
	info.mapped_ns = map_device_time(cap, cap->cam_clock, "camera", device_ns, host_ns, CAMERA_FALLBACK_LATENCY_NS,
	                                 &cap->gui.cam_fallbacks);
	// Diagnostic only (meaningful if the camera and IMU device clocks are one), no fallback.
	timepoint_ns imu_mapped = 0;
	if (m_clock_windowed_skew_tracker_to_local(cap->imu_clock, device_ns, &imu_mapped)) {
		info.imu_clock_ns = imu_mapped;
	}
	if (info.mapped_ns >= 0 && info.imu_clock_ns >= 0) {
		cap->gui.cam_minus_imu_clock_ns = info.mapped_ns - info.imu_clock_ns;
	}

	if (cap->record != NULL) {
		xreal_air_record_push_raw(cap->record, raw->data, raw->size, raw->v4l2_sequence);
	}

	os_mutex_unlock(&cap->lock);

	xf->timestamp = (uint64_t)info.mapped_ns;
	xf->source_timestamp = info.telemetry.device_ns;
	xf->source_sequence = info.telemetry.counter;

	cap->gui.frames++;
	if (cap->gui.frames == CAMERA_CLOCK_CHECK_FRAMES) {
		CAP_NOTE(cap,
		         "Camera clock minus IMU clock (same instant, both mapped to monotonic): %.3f ms. "
		         "Small and stable = the camera and IMU device clocks share an epoch.",
		         (double)cap->gui.cam_minus_imu_clock_ns / U_TIME_1MS_IN_NS);
	}

	if (info.cam == XREAL_AIR_CAM_LEFT || info.cam == XREAL_AIR_CAM_RIGHT) {
		u_sink_debug_push_frame(&cap->gui.sinks[info.cam], xf);

		// Pair by counter, the order within a pair is not fixed.
		const int self = (int)info.cam;
		const int other = 1 - self;
		if (cap->pending[other].frame != NULL &&
		    cap->pending[other].info.telemetry.counter == info.telemetry.counter) {
			struct xrt_frame *frames[2] = {NULL, NULL};
			struct xreal_air_record_cam_frame *infos[2] = {NULL, NULL};
			frames[self] = xf;
			frames[other] = cap->pending[other].frame;
			infos[self] = &info;
			infos[other] = &cap->pending[other].info;

			// One timestamp for the pair, the left exposure (they are within microseconds).
			const int64_t pair_ns = pair_timestamp(cap, infos[XREAL_AIR_CAM_LEFT]);

			// Auto exposure starts from zero after STREAMON, the first pairs are black.
			const bool warmup = infos[0]->telemetry.exposure == 0 || infos[1]->telemetry.exposure == 0;
			if (warmup) {
				cap->gui.warmup++;
			}

			os_mutex_lock(&cap->lock);
			if (cap->record != NULL && !warmup && pair_ns > cap->last_pair_euroc_ns) {
				infos[0]->euroc_ns = pair_ns;
				infos[1]->euroc_ns = pair_ns;
				xreal_air_record_push_stereo(cap->record, pair_ns, frames[0], frames[1]);
				cap->last_pair_euroc_ns = pair_ns;
			}
			// The pending frame's telemetry row was held back for its euroc_ns.
			if (cap->record != NULL) {
				xreal_air_record_push_cam_frame(cap->record, infos[other]);
				xreal_air_record_push_cam_frame(cap->record, infos[self]);
			}
			os_mutex_unlock(&cap->lock);

			xrt_frame_reference(&cap->pending[other].frame, NULL);
			cap->gui.pairs++;
		} else {
			// Whatever was pending for either camera will not be paired any more.
			for (int i = 0; i < 2; i++) {
				if (cap->pending[i].frame == NULL) {
					continue;
				}
				os_mutex_lock(&cap->lock);
				if (cap->record != NULL) {
					xreal_air_record_push_cam_frame(cap->record, &cap->pending[i].info);
				}
				os_mutex_unlock(&cap->lock);
				drop_pending(cap, i);
			}
			xrt_frame_reference(&cap->pending[self].frame, xf);
			cap->pending[self].info = info;
		}
	}

	xrt_frame_reference(&xf, NULL);
}


/*
 *
 * Camera open thread.
 *
 */

static void *
open_thread(void *ptr)
{
	struct xreal_air_capture *cap = (struct xreal_air_capture *)ptr;
	os_thread_helper_name(&cap->oth, "XREAL Air Camera Open");

	const char *forced = debug_get_option_xreal_air_camera_device();

	for (int attempt = 0; attempt < CAMERA_OPEN_ATTEMPTS && os_thread_helper_is_running(&cap->oth); attempt++) {
		char path[64] = {0};
		if (forced != NULL && forced[0] != '\0') {
			(void)snprintf(path, sizeof(path), "%s", forced);
		} else if (!xreal_air_camera_find_device(path, sizeof(path))) {
			path[0] = '\0';
		}

		// Only try nodes that exist, a forced path may not have appeared yet.
		if (path[0] != '\0' && access(path, F_OK) == 0) {
			cap->camera = xreal_air_camera_create(path, on_camera_frame, cap, cap->log_level);
			if (cap->camera != NULL) {
				(void)snprintf(cap->camera_path, sizeof(cap->camera_path), "%s", path);
				break;
			}
		}

		if (attempt == 0) {
			CAP_WARN(cap, "Camera not available yet, retrying for %d s", CAMERA_OPEN_ATTEMPTS);
		}

		// Sleep in small steps so destroy is not held up.
		for (int i = 0; i < 10 && os_thread_helper_is_running(&cap->oth); i++) {
			os_nanosleep(CAMERA_OPEN_INTERVAL_NS / 10);
		}
	}

	if (cap->camera == NULL && os_thread_helper_is_running(&cap->oth)) {
		CAP_ERROR(cap, "Could not open the Air 2 Ultra camera, recording IMU only");
	}

	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

struct xreal_air_capture *
xreal_air_capture_create(enum u_logging_level log_level)
{
	const char *record_path = debug_get_option_xreal_air_euroc_record();
	const bool record = record_path != NULL && record_path[0] != '\0';
	if (!record && !debug_get_bool_option_xreal_air_cameras()) {
		return NULL;
	}

	struct xreal_air_capture *cap = U_TYPED_CALLOC(struct xreal_air_capture);
	cap->log_level = log_level;
	cap->last_imu_euroc_ns = -1;
	cap->last_pair_euroc_ns = -1;
	(void)snprintf(cap->record_base, sizeof(cap->record_base), "%s", record ? record_path : "xreal_euroc");

	if (os_mutex_init(&cap->lock) != 0) {
		free(cap);
		return NULL;
	}
	if (os_thread_helper_init(&cap->oth) != 0) {
		os_mutex_destroy(&cap->lock);
		free(cap);
		return NULL;
	}

	cap->imu_clock = m_clock_windowed_skew_tracker_alloc(IMU_CLOCK_WINDOW);
	cap->cam_clock = m_clock_windowed_skew_tracker_alloc(CAMERA_CLOCK_WINDOW);

	u_sink_debug_init(&cap->gui.sinks[0]);
	u_sink_debug_init(&cap->gui.sinks[1]);

	if (record) {
		start_record(cap, true);
	}

	(void)snprintf(cap->gui.record_btn.label, sizeof(cap->gui.record_btn.label), "%s",
	               cap->record != NULL ? "Stop recording" : "Record EuRoC dataset");
	cap->gui.record_btn.cb = record_btn_cb;
	cap->gui.record_btn.ptr = cap;

	u_var_add_root(cap, "Xreal Air cameras", true);
	u_var_add_button(cap, &cap->gui.record_btn, cap->gui.record_btn.label);
	u_var_add_ro_u64(cap, &cap->gui.frames, "Camera frames");
	u_var_add_ro_u64(cap, &cap->gui.pairs, "Stereo pairs");
	u_var_add_ro_u64(cap, &cap->gui.unpaired, "Unpaired frames");
	u_var_add_ro_u64(cap, &cap->gui.warmup, "Zero exposure pairs (skipped)");
	u_var_add_ro_u64(cap, &cap->gui.imu_samples, "IMU samples");
	u_var_add_ro_i64_ns(cap, &cap->gui.cam_minus_imu_clock_ns, "Camera minus IMU clock");
	u_var_add_ro_u64(cap, &cap->gui.cam_fallbacks, "Camera timestamp fallbacks");
	u_var_add_ro_u64(cap, &cap->gui.imu_fallbacks, "IMU timestamp fallbacks");
	u_var_add_sink_debug(cap, &cap->gui.sinks[XREAL_AIR_CAM_LEFT], "Left (cam0)");
	u_var_add_sink_debug(cap, &cap->gui.sinks[XREAL_AIR_CAM_RIGHT], "Right (cam1)");

	if (os_thread_helper_start(&cap->oth, open_thread, cap) != 0) {
		CAP_ERROR(cap, "Could not start the camera open thread");
	}

	return cap;
}

void
xreal_air_capture_set_calibration(struct xreal_air_capture *cap,
                                  const char *buffer,
                                  size_t size,
                                  const struct xreal_air_parsed_calibration *calib)
{
	size_t doc_len = 0;
	char *doc = xreal_air_calibration_document(buffer, size, &doc_len);

	os_mutex_lock(&cap->lock);
	free(cap->calib_document);
	cap->calib_document = doc;
	cap->calib_document_len = doc != NULL ? doc_len : 0;
	cap->calib = *calib;
	cap->calib_valid = true;
	if (cap->record != NULL) {
		xreal_air_record_set_calibration(cap->record, cap->calib_document, cap->calib_document_len,
		                                 &cap->calib);
	}
	os_mutex_unlock(&cap->lock);

	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_COUNT; i++) {
		const struct xreal_air_slam_camera_calibration *c = &calib->slam_camera[i];
		if (!c->valid) {
			CAP_WARN(cap, "No factory SLAM_camera device_%d calibration", i + 1);
			continue;
		}
		CAP_INFO(cap, "SLAM camera %d: %s %dx%d f=(%.2f, %.2f) c=(%.2f, %.2f) %d coeffs", i, c->model,
		         c->resolution[0], c->resolution[1], c->fc[0], c->fc[1], c->cc[0], c->cc[1], c->kc_count);
	}
}

void
xreal_air_capture_push_imu(struct xreal_air_capture *cap,
                           uint64_t device_ts,
                           int64_t fusion_ns,
                           const struct xreal_air_parsed_sample *raw,
                           float temperature,
                           const struct xrt_vec3 *gyro,
                           const struct xrt_vec3 *accel)
{
	const int64_t host_ns = (int64_t)os_monotonic_get_ns();

	struct xreal_air_record_imu imu = {
	    .device_ts = device_ts,
	    .host_ns = host_ns,
	    .fusion_ns = fusion_ns,
	    .euroc_ns = -1,
	    .raw = *raw,
	    .temperature = temperature,
	    // The driver's output frame is (x right, y up, z back), the factory IMU frame the camera
	    // extrinsics refer to is (x right, y down, z forward): undo post_biased_coordinate_system.
	    .gyro = {gyro->x, -gyro->y, -gyro->z},
	    .accel = {accel->x, -accel->y, -accel->z},
	};

	os_mutex_lock(&cap->lock);

	m_clock_windowed_skew_tracker_push(cap->imu_clock, host_ns, (timepoint_ns)device_ts);
	// Samples before the factory calibration arrived are not calibrated, keep them out of mav0.
	if (cap->calib_valid) {
		int64_t mapped = map_device_time(cap, cap->imu_clock, "IMU", (int64_t)device_ts, host_ns,
		                                 IMU_FALLBACK_LATENCY_NS, &cap->gui.imu_fallbacks);
		// EuRoC wants strictly increasing timestamps.
		if (mapped <= cap->last_imu_euroc_ns) {
			mapped = cap->last_imu_euroc_ns + 1;
		}
		imu.euroc_ns = mapped;
		cap->last_imu_euroc_ns = mapped;
	}

	if (cap->record != NULL) {
		xreal_air_record_push_imu(cap->record, &imu);
	}

	// Sanity check the assumption that the HID timestamp counts nanoseconds.
	if (cap->imu_rate.count == 0) {
		cap->imu_rate.first_device_ts = device_ts;
		cap->imu_rate.first_host_ns = host_ns;
	} else if (cap->imu_rate.count == IMU_RATE_CHECK_SAMPLES && !cap->imu_rate.checked) {
		const double device_span = (double)(device_ts - cap->imu_rate.first_device_ts);
		const double host_span = (double)(host_ns - cap->imu_rate.first_host_ns);
		const double ratio = host_span > 0 ? device_span / host_span : 0.0;
		cap->imu_rate.checked = true;
		if (ratio < 0.99 || ratio > 1.01) {
			CAP_ERROR(cap, "IMU device timestamps run at %.6f x host time, they are NOT nanoseconds",
			          ratio);
		} else {
			CAP_NOTE(cap, "IMU device clock / host clock = %.6f, %.1f Hz", ratio,
			         (double)IMU_RATE_CHECK_SAMPLES * 1e9 / host_span);
		}
	}
	cap->imu_rate.count++;
	cap->gui.imu_samples++;

	os_mutex_unlock(&cap->lock);
}

void
xreal_air_capture_destroy(struct xreal_air_capture **cap_ptr)
{
	struct xreal_air_capture *cap = *cap_ptr;
	if (cap == NULL) {
		return;
	}

	u_var_remove_root(cap);

	// Stop the open thread first, then the camera thread, so no more frames arrive.
	os_thread_helper_destroy(&cap->oth);
	xreal_air_camera_destroy(&cap->camera);

	CAP_NOTE(cap,
	         "%" PRIu64 " camera frames, %" PRIu64 " pairs, %" PRIu64 " unpaired, %" PRIu64
	         " zero exposure pairs skipped, %" PRIu64 " IMU samples, unmappable timestamps: %" PRIu64
	         " camera / %" PRIu64 " IMU",
	         cap->gui.frames, cap->gui.pairs, cap->gui.unpaired, cap->gui.warmup, cap->gui.imu_samples,
	         cap->gui.cam_fallbacks, cap->gui.imu_fallbacks);
	CAP_NOTE(cap, "%" PRIu64 " pairs timed via the camera's own clock instead of the IMU clock",
	         cap->gui.pairs_own_clock);

	for (int i = 0; i < 2; i++) {
		xrt_frame_reference(&cap->pending[i].frame, NULL);
	}

	stop_record(cap);

	u_sink_debug_destroy(&cap->gui.sinks[0]);
	u_sink_debug_destroy(&cap->gui.sinks[1]);

	m_clock_windowed_skew_tracker_destroy(cap->imu_clock);
	m_clock_windowed_skew_tracker_destroy(cap->cam_clock);

	os_mutex_destroy(&cap->lock);
	free(cap->calib_document);
	free(cap);
	*cap_ptr = NULL;
}
