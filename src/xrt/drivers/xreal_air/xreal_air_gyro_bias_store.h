// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Persist the learned m_imu_3dof gyro bias of Xreal Air glasses across sessions.
 * @ingroup drv_xreal_air
 */

#pragma once

#include "util/u_logging.h"

#include "math/m_imu_3dof.h"

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


struct xreal_air_gyro_bias_store;

/*!
 * Load the saved bias of @p device_id (if any) from the state directory and start the writer
 * thread. Garbage or a mismatching file is warned about and ignored (and replaced on the next
 * save).
 *
 * @param params Tunables the saved bias is validated against.
 * @return NULL if no state directory could be found.
 */
struct xreal_air_gyro_bias_store *
xreal_air_gyro_bias_store_create(const char *device_id,
                                 const struct m_imu_3dof_gyro_bias_params *params,
                                 enum u_logging_level log_level);

/*!
 * The record loaded at create, if any.
 */
bool
xreal_air_gyro_bias_store_get_loaded(const struct xreal_air_gyro_bias_store *s,
                                     struct m_imu_3dof_gyro_bias_record *out);

/*!
 * Seed @p fusion with the loaded record, its uncertainty inflated for the time and temperature
 * change since it was saved. Does nothing if nothing was loaded. Logs what it did.
 */
void
xreal_air_gyro_bias_store_seed(struct xreal_air_gyro_bias_store *s, struct m_imu_3dof *fusion, float temp_c);

/*!
 * Call after every fusion update, from the sensor thread only. Cheap: every 30 s at most (when
 * blocks were learned, otherwise as a slower heartbeat) hands a snapshot of the estimator to the
 * writer thread, which logs it and saves it when it meaningfully changed. Never touches the disk.
 */
void
xreal_air_gyro_bias_store_tick(struct xreal_air_gyro_bias_store *s,
                               const struct m_imu_3dof *fusion,
                               float temp_c,
                               int64_t now_ns);

/*!
 * Stop the writer thread, then save @p fusion one last time if it learned anything since the last
 * save (pass NULL to skip that), and free everything. The sensor thread must be stopped already.
 */
void
xreal_air_gyro_bias_store_destroy(struct xreal_air_gyro_bias_store **s_ptr,
                                  const struct m_imu_3dof *fusion,
                                  float temp_c);

//! File the bias is loaded from and saved to.
const char *
xreal_air_gyro_bias_store_path(const struct xreal_air_gyro_bias_store *s);


#ifdef __cplusplus
}
#endif
