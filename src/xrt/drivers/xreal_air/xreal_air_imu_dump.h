// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Non-blocking CSV dump of the Xreal Air IMU stream, for offline fusion tuning.
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * One fused IMU sample, all vectors in the fusion's (calibrated, body) frame.
 *
 * @ingroup drv_xreal_air
 */
struct xreal_air_imu_dump_record
{
	uint64_t device_ns; //!< Raw device timestamp of the sample.
	int64_t fusion_ns;  //!< Timestamp handed to m_imu_3dof_update.
	int64_t host_ns;    //!< os_monotonic_get_ns() when the packet was handled.
	float temperature_c;
	struct xrt_vec3 gyro;  //!< rad/s, factory calibration applied.
	struct xrt_vec3 accel; //!< m/s^2, factory calibration applied.
	struct xrt_vec3 mag;
	struct xrt_vec3 bias; //!< Fusion gyro bias after this sample (rad/s).
	struct xrt_quat rot;  //!< Fused IMU orientation after this sample.
	bool still;           //!< Bias estimator considered this sample still.
};

struct xreal_air_imu_dump;

/*!
 * Open @p path (or a timestamped file inside it if it is a directory) and start
 * the writer thread. @p header is written as a leading comment line.
 *
 * @return NULL on failure.
 */
struct xreal_air_imu_dump *
xreal_air_imu_dump_create(const char *path, const char *header);

/*!
 * Queue one record, never blocks: drops (and counts) the record when the
 * writer has fallen too far behind. Single producer only.
 */
void
xreal_air_imu_dump_push(struct xreal_air_imu_dump *d, const struct xreal_air_imu_dump_record *r);

//! Path actually written to.
const char *
xreal_air_imu_dump_path(const struct xreal_air_imu_dump *d);

/*!
 * Drain what is queued, close the file and free everything.
 */
void
xreal_air_imu_dump_destroy(struct xreal_air_imu_dump **d_ptr);


#ifdef __cplusplus
}
#endif
