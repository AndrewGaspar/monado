// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra 6DoF flight recorder: on-demand diagnostic captures with look-back.
 *
 * Always on while the camera capture runs (XREAL_AIR_SLAM=1 or XREAL_AIR_CAMERAS=1): the last
 * XREAL_AIR_FLIGHT_LOOKBACK_S seconds of the low-bandwidth signals are kept in RAM rings, and a
 * capture started at runtime writes them out first and then keeps recording live until stopped (or
 * a limit hits). No restart and no debug GUI needed: the runtime is driven through control files
 * (see below, and scripts/xreal-capture in HypXRland for the user-facing tool).
 *
 * Rings (always on, ~25 MB at 60 s): every IMU sample (the imu_raw.csv row), every camera frame's
 * telemetry row (camera_frames.csv: kernel / dequeue times, so frame gaps and dequeue stalls), every
 * new SLAM pose as the driver saw it (with the pose guard's verdict and world), t_slam's per-stage
 * timing of each pose (timing.csv, needs no SLAM_WRITE_CSVS), every head pose handed out (output
 * pose, 3DoF fusion orientation, SLAM or not) and the SLAM state changes (tracking lost / acquired,
 * re-initialisations, divergences, wear, tracker restarts; never rate limited, unlike the log).
 * Opt-in: the decoded stereo pairs of the last N seconds (references to the frames the tracker gets,
 * no copy; 614 KB per pair, ~17 MB per second), XREAL_AIR_FLIGHT_FRAMES_S or the arm command.
 *
 * A capture is a directory in the driver's own EuRoC layout minus the images (mav0/imu0, xreal/ with
 * camera_frames.csv, imu_raw.csv and the calibration files), the stereo pairs in one append-only
 * xreal/frames.bin (raw 8-bit or zstd, no PNG on the device) and the flight-only files in flight/.
 * scripts/xreal-capture-to-euroc.py expands frames.bin into mav0/cam0, mav0/cam1 PNGs, after which
 * monado-cli xreal-slam-replay takes the directory as is. Everything is flushed as it goes (the
 * runtime is stopped with SIGKILL).
 *
 * Control: files dropped into the control directory, XREAL_AIR_FLIGHT_CTL, default
 * $RUNTIME_DIRECTORY/capture (systemd's RuntimeDirectory=, the packaged unit) or else
 * $XDG_RUNTIME_DIR/monado-xreal/capture. Write the file elsewhere in that directory and rename it
 * into place; the runtime reads, deletes and answers it in the "status" file (key=value, also
 * written while recording, once a second). Command files hold key=value lines:
 *  - start  out=/abs/dir (required, must not exist or be empty), lookback_s (default: all the ring
 *           holds), max_s (default 600), frames=0|1 (default 1), codec=raw|zstd (default zstd if
 *           built with it), level (zstd, default 1), id (echoed as cmd_id in the status)
 *  - stop   reason
 *  - mark   label, mono_ns (the caller's CLOCK_MONOTONIC, else the time it is read): an event in
 *           flight/events.csv, "the user saw something NOW".
 *  - arm    frames_s: the frame look-back in seconds, 0 = off. Takes effect at once.
 *
 * Limits: a capture refuses to start with less than XREAL_AIR_FLIGHT_MIN_FREE_MB (2048) free at
 * out, stops itself below half of that, past XREAL_AIR_FLIGHT_MAX_MB (8192) written, and after
 * max_s. The producers never wait for the writer or the disk: the writer is a nice 10 thread that
 * reads the rings behind the producers; what it does not get to in time is overwritten and counted
 * (capture.txt).
 *
 * Environment:
 *  - XREAL_AIR_FLIGHT=0                 No rings, no control directory.
 *  - XREAL_AIR_FLIGHT_LOOKBACK_S=60     Length of the always-on rings.
 *  - XREAL_AIR_FLIGHT_FRAMES_S=0        Frame look-back from the start (see arm).
 *  - XREAL_AIR_FLIGHT_CTL=/dir          The control directory.
 *  - XREAL_AIR_FLIGHT_MIN_FREE_MB=2048, XREAL_AIR_FLIGHT_MAX_MB=8192   See limits.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_logging.h"

#include "xreal_air_record.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_frame;
struct xreal_air_flight;

//! Settings, from the environment by @ref xreal_air_flight_default_config.
struct xreal_air_flight_config
{
	enum u_logging_level log_level;
	char ctl_dir[1024];
	uint32_t lookback_s;
	uint32_t frames_lookback_s;
	uint32_t min_free_mb;
	uint64_t max_mb;
	uint32_t default_max_s;
	int writer_nice;
};

//! A new pose from the tracker, as the driver first saw it.
struct xreal_air_flight_slam_pose
{
	int64_t ts_ns;        //!< The pose's timestamp (the stereo pair's mid-exposure).
	int64_t seen_ns;      //!< CLOCK_MONOTONIC when the driver first saw it.
	struct xrt_pose pose; //!< Basalt's world (Z up), IMU body.
	struct xrt_vec3 linear_velocity;
	uint32_t world_id; //!< The pose guard's world, see xreal_air_slam_guard.h.
	int32_t verdict;   //!< The guard's verdict, enum xreal_air_slam_pose_verdict.
};

//! A head pose handed out.
struct xreal_air_flight_head
{
	int64_t query_ns;       //!< CLOCK_MONOTONIC when it was asked for.
	int64_t at_ns;          //!< The time it was asked for.
	struct xrt_pose pose;   //!< The output (OpenXR, Y up, head).
	struct xrt_quat fusion; //!< The 3DoF fusion's orientation.
	uint32_t world_id;
	bool slam; //!< The output followed SLAM.
};

/*!
 * The settings from the environment (see the file comment).
 *
 * @return false if XREAL_AIR_FLIGHT=0.
 */
bool
xreal_air_flight_default_config(enum u_logging_level log_level, struct xreal_air_flight_config *out);

/*!
 * Allocate the rings and start the writer thread (which watches the control directory).
 *
 * @return NULL on failure.
 */
struct xreal_air_flight *
xreal_air_flight_create(const struct xreal_air_flight_config *config);

//! The factory calibration, for the calibration files of each capture.
void
xreal_air_flight_set_calibration(struct xreal_air_flight *f,
                                 const char *document,
                                 size_t document_len,
                                 const struct xreal_air_parsed_calibration *calib);

//! An IMU sample (sensor thread). Never waits for the writer, as all the pushes.
void
xreal_air_flight_push_imu(struct xreal_air_flight *f, const struct xreal_air_record_imu *imu);

//! A camera frame's telemetry row (camera thread).
void
xreal_air_flight_push_cam_frame(struct xreal_air_flight *f, const struct xreal_air_record_cam_frame *frame);

/*!
 * A stereo pair of upright L8 frames with its EuRoC timestamp (camera thread). Kept by reference
 * while the frame look-back is armed or a capture records frames, dropped right away otherwise.
 */
void
xreal_air_flight_push_stereo(struct xreal_air_flight *f,
                             int64_t euroc_ns,
                             struct xrt_frame *left,
                             struct xrt_frame *right);

void
xreal_air_flight_push_slam_pose(struct xreal_air_flight *f, const struct xreal_air_flight_slam_pose *pose);

//! The timing columns' names for flight/slam_timing.csv (t_slam_get_timing_columns).
void
xreal_air_flight_set_timing_columns(struct xreal_air_flight *f, const char *columns_csv);

//! One pose's pipeline timestamps (t_slam_timing_callback).
void
xreal_air_flight_push_slam_timing(struct xreal_air_flight *f, const int64_t *timestamps, uint32_t count);

void
xreal_air_flight_push_head(struct xreal_air_flight *f, const struct xreal_air_flight_head *head);

//! An event for flight/events.csv, @p type a short word ("tracking", "slam", "wear", ...).
void
xreal_air_flight_event(struct xreal_air_flight *f, const char *type, const char *fmt, ...) XRT_PRINTF_FORMAT(3, 4);

//! The control directory (for logs and tools).
const char *
xreal_air_flight_ctl_dir(const struct xreal_air_flight *f);

/*!
 * Stop the writer (finishing a running capture) and free everything. Call after nothing pushes.
 */
void
xreal_air_flight_destroy(struct xreal_air_flight **f_ptr);

#ifdef __cplusplus
}
#endif
