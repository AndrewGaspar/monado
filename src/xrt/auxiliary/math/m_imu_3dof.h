// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A IMU fusion specially made for 3dof devices.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#pragma once

#include "xrt/xrt_defines.h"


#ifdef __cplusplus
extern "C" {
#endif


#define M_IMU_3DOF_USE_GRAVITY_DUR_300MS (1 << 0)
#define M_IMU_3DOF_USE_GRAVITY_DUR_20MS (1 << 1)
/*!
 * Opt-in: continuously re-estimate the gyro bias while the device is still.
 *
 * Gravity correction only observes pitch and roll, so any residual gyro bias
 * integrates into an unbounded yaw drift. With this flag the fusion watches for
 * sustained stillness (low mean angular rate, no rate spikes, accelerometer at
 * ~1g with no jitter) and slowly low-pass tracks the residual rate into
 * @ref m_imu_3dof::gyro_bias. The approach follows the stationary-gated
 * high-pass offset tracker of x-io's Fusion library (FusionBias, MIT), with a
 * mean-rate gate, a deferred commit and slew/deviation clamps added so slow
 * deliberate head motion is not absorbed. See @ref m_imu_3dof_gyro_bias_params.
 */
#define M_IMU_3DOF_AUTO_GYRO_BIAS (1 << 2)


struct m_ff_vec3_f32;

//! Maximum number of still blocks held back before being learned from.
#define M_IMU_3DOF_BIAS_EST_MAX_HELD 8

/*!
 * Tunables for @ref M_IMU_3DOF_AUTO_GYRO_BIAS. Angular quantities are in degrees
 * per second (readable in the debug GUI), everything else in SI units.
 */
struct m_imu_3dof_gyro_bias_params
{
	//! Runtime switch, only has an effect when the flag was given at init.
	bool enabled;

	//! Gate: magnitude of the mean bias-corrected angular rate over one block (deg/s).
	float gate_block_deg_s;
	//! Gate: magnitude of any single bias-corrected angular rate sample (deg/s).
	float gate_sample_deg_s;
	//! Gate: how far the mean acceleration magnitude may be from 1g (m/s^2).
	float gate_accel_m_s2;
	//! Gate: how far any single acceleration sample may be from the mean (m/s^2).
	float gate_accel_jitter_m_s2;
	//! Time constant of the running acceleration mean (s).
	float accel_window_s;
	//! All gates must have passed for this long before a block may be learned (s).
	float still_period_s;
	//! Stillness is judged and learned in blocks of this length (s).
	float block_s;
	//! A block is only learned once this many newer blocks also passed, so the
	//! start of a slow motion is discarded rather than learned [1, M_IMU_3DOF_BIAS_EST_MAX_HELD].
	int32_t held_blocks;
	//! Bias tracking time constant (s).
	float tau_s;
	//! Bias tracking time constant during the first @ref fast_period_s of learning (s).
	float fast_tau_s;
	//! Seconds of learned stillness that use @ref fast_tau_s.
	float fast_period_s;
	//! Maximum bias change per second of learned stillness (deg/s per s).
	float max_slew_deg_s2;
	//! Maximum distance of the estimate from the bias at init (deg/s).
	float max_dev_deg_s;
};

enum m_imu_3dof_state
{
	M_IMU_3DOF_STATE_START = 0,
	M_IMU_3DOF_STATE_RUNNING = 1,
};

struct m_imu_3dof
{
	struct xrt_quat rot; //!< Orientation

	struct
	{
		uint64_t timestamp_ns;
		struct xrt_vec3 gyro;  //!< Angular velocity
		struct xrt_vec3 accel; //!< Acceleration
		double delta_ms;
		float accel_length;
		float gyro_length;
		float gyro_biased_length;
	} last;

	enum m_imu_3dof_state state;

	int flags;

	// Filter fifos for accelerometer and gyroscope.
	struct m_ff_vec3_f32 *word_accel_ff;
	struct m_ff_vec3_f32 *gyro_ff;

	// gravity correction
	struct
	{
		uint64_t level_timestamp_ns;
		struct xrt_vec3 error_axis;
		float error_angle;
		bool is_accel;
		bool is_rotating;
	} grav;

	// gyro bias correction
	struct
	{
		struct xrt_vec3 value;
		bool manually_fire;
	} gyro_bias;

	//! Automatic gyro bias estimation, see @ref M_IMU_3DOF_AUTO_GYRO_BIAS.
	struct
	{
		struct m_imu_3dof_gyro_bias_params params;

		//! @ref gyro_bias value the estimate may not stray far from.
		struct xrt_vec3 origin;

		//! Running mean of the acceleration, feeds the accel gates.
		struct xrt_vec3 accel_mean;
		bool accel_mean_valid;

		//! Last time any gate failed.
		uint64_t still_start_ns;

		//! Block currently being accumulated (bias-corrected rate * dt).
		struct xrt_vec3 block_sum;
		double block_dt;
		uint64_t block_start_ns;

		//! Completed still blocks awaiting learning, oldest first, absolute rates.
		struct
		{
			struct xrt_vec3 rate;
			double dt;
			uint64_t start_ns;
		} held[M_IMU_3DOF_BIAS_EST_MAX_HELD];
		int32_t held_count;

		//! Total learned stillness (s), selects the fast time constant.
		double learned_s;

		// Debug output.
		struct xrt_vec3 last_block_rate; //!< Bias-corrected mean of the last block.
		bool is_still;
		float still_s;
		uint32_t commits;
	} bias_est;
};

/*!
 * Fill in the default @ref M_IMU_3DOF_AUTO_GYRO_BIAS tunables.
 */
void
m_imu_3dof_gyro_bias_params_default(struct m_imu_3dof_gyro_bias_params *p);

/*!
 * Override tunables from a string like "tau_s=30,gate_block_deg_s=0.2", keys are
 * the field names of @ref m_imu_3dof_gyro_bias_params, separated by commas or
 * whitespace. Meant for env vars and offline tools.
 *
 * @return false on an unknown key or malformed value, earlier keys stay applied.
 */
bool
m_imu_3dof_gyro_bias_params_parse(struct m_imu_3dof_gyro_bias_params *p, const char *str);

/*!
 * Print all tunables as a string @ref m_imu_3dof_gyro_bias_params_parse accepts.
 */
void
m_imu_3dof_gyro_bias_params_print(const struct m_imu_3dof_gyro_bias_params *p, char *buf, size_t size);

void
m_imu_3dof_init(struct m_imu_3dof *f, int flags);

void
m_imu_3dof_reset(struct m_imu_3dof *f);

void
m_imu_3dof_close(struct m_imu_3dof *f);

void
m_imu_3dof_add_vars(struct m_imu_3dof *f, void *root, const char *prefix);

void
m_imu_3dof_update(struct m_imu_3dof *f,
                  uint64_t timestamp_ns,
                  const struct xrt_vec3 *accel,
                  const struct xrt_vec3 *gyro);


#ifdef __cplusplus
}
#endif
