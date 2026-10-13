// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra camera + IMU dataset recorder (EuRoC layout + sidecars).
 *
 * Writes a standard EuRoC MAV dataset (mav0/cam0, mav0/cam1, mav0/imu0, consumable by
 * Monado's EuRoC player and Basalt) plus, under xreal/, everything EuRoC cannot carry:
 * the factory calibration blob verbatim, Basalt and Kalibr style calibration files
 * converted from it, and per-frame / per-IMU-sample CSVs with the raw device and host
 * timestamps needed to solve the camera <-> IMU clock relation offline.
 *
 * Deliberately independent of OpenCV (t_euroc_recorder needs it): PNGs are written with
 * stb_image_write on a writer thread. Every file is flushed as it goes, the runtime is
 * usually stopped with SIGKILL so the dataset must be usable without a clean shutdown.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_logging.h"

#include "xreal_air_camera_decode.h"
#include "xreal_air_hmd.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_frame;
struct xreal_air_record;

/*!
 * One IMU sample as seen by the driver.
 */
struct xreal_air_record_imu
{
	uint64_t device_ts; //!< Raw device timestamp from the HID packet (ns, power-on epoch).
	int64_t host_ns;    //!< os_monotonic_get_ns() when the sample was handled.
	int64_t fusion_ns;  //!< The timestamp the 3DoF fusion was given.
	int64_t euroc_ns;   //!< Device time mapped onto CLOCK_MONOTONIC, or -1 if not mapped yet.

	struct xreal_air_parsed_sample raw; //!< Raw integer sample with its conversion factors.
	float temperature;                  //!< Celsius.

	//! Factory calibrated, SI units, in the factory "IMU" frame (x right, y down, z forward),
	//! the frame the SLAM_camera imu_p_cam / imu_q_cam are expressed in.
	struct xrt_vec3 gyro;
	struct xrt_vec3 accel;
};

/*!
 * Telemetry of one camera frame, every frame (paired or not).
 */
struct xreal_air_record_cam_frame
{
	struct xreal_air_cam_telemetry telemetry;
	enum xreal_air_cam_id cam;
	uint32_t phase;         //!< Descramble phase.
	uint32_t v4l2_sequence; //!< V4L2 sequence, gaps are frames dropped before us.
	int64_t kernel_ns;      //!< V4L2 buffer timestamp (CLOCK_MONOTONIC).
	int64_t dequeue_ns;     //!< Host time right after VIDIOC_DQBUF.
	int64_t mapped_ns;      //!< Device time mapped onto CLOCK_MONOTONIC via the camera clock, or -1.
	int64_t imu_clock_ns;   //!< Device time mapped via the IMU clock (valid iff same epoch), or -1.
	int64_t euroc_ns;       //!< The pair's EuRoC timestamp, or -1 if not part of a recorded pair.
};

/*!
 * Create a dataset in @p dir (created, must not exist or be empty) and start the writer thread.
 */
struct xreal_air_record *
xreal_air_record_create(const char *dir, enum u_logging_level log_level);

//! The dataset directory.
const char *
xreal_air_record_path(const struct xreal_air_record *r);

/*!
 * Set the factory calibration, writes xreal/factory_calibration.json (the verbatim document)
 * and the converted calibration files. May be called again (e.g. once the IMU rate is known).
 */
void
xreal_air_record_set_calibration(struct xreal_air_record *r,
                                 const char *document,
                                 size_t document_len,
                                 const struct xreal_air_parsed_calibration *calib);

//! Queue an IMU sample, never blocks on I/O.
void
xreal_air_record_push_imu(struct xreal_air_record *r, const struct xreal_air_record_imu *imu);

//! Queue a camera frame's telemetry row, never blocks on I/O.
void
xreal_air_record_push_cam_frame(struct xreal_air_record *r, const struct xreal_air_record_cam_frame *frame);

/*!
 * Queue a stereo pair of upright L8 images for mav0/cam0 (left) and mav0/cam1 (right), both
 * written with the same EuRoC timestamp @p euroc_ns. Takes its own references.
 */
void
xreal_air_record_push_stereo(struct xreal_air_record *r,
                             int64_t euroc_ns,
                             struct xrt_frame *left,
                             struct xrt_frame *right);

/*!
 * Keep a raw (scrambled) frame for offline decoder debugging, the first few frames only.
 */
void
xreal_air_record_push_raw(struct xreal_air_record *r, const uint8_t *data, size_t size, uint32_t v4l2_sequence);

/*!
 * Drain the queue, write the final calibration files and close everything.
 */
void
xreal_air_record_destroy(struct xreal_air_record **r_ptr);

#ifdef __cplusplus
}
#endif
