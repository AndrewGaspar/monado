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
 * integrates into an unbounded yaw drift. With this flag the fusion collects
 * the bias-corrected rate in short blocks while nothing moves fast (no rate
 * spikes, accelerometer at ~1g without jitter) and feeds blocks that agree with
 * each other into a small Kalman filter on @ref m_imu_3dof::gyro_bias.
 *
 * The components are treated by observability. Rotation about gravity (yaw) is
 * exactly what a slow deliberate head pan looks like, so the gravity-aligned
 * residual has to stay small and consistent over every recent window of
 * blocks; the horizontal components are cross-checked by gravity correction and
 * may be anything up to the deviation clamp. When the blocks barely scatter at
 * all (the device lies on a table, no head attached) the yaw gate opens up to
 * the deviation clamp as well. The idea of stationary-gated offset tracking is
 * from x-io's Fusion library (FusionBias, MIT); this is a reimplementation with
 * different gating. See @ref m_imu_3dof_gyro_bias_params.
 */
#define M_IMU_3DOF_AUTO_GYRO_BIAS (1 << 2)


struct m_ff_vec3_f32;

//! Maximum number of still blocks held back before being learned from.
#define M_IMU_3DOF_BIAS_EST_MAX_HELD 16

/*!
 * Tunables for @ref M_IMU_3DOF_AUTO_GYRO_BIAS. Angular quantities are in degrees
 * per second (readable in the debug GUI), everything else in SI units.
 */
struct m_imu_3dof_gyro_bias_params
{
	//! Runtime switch, only has an effect when the flag was given at init.
	bool enabled;

	//! Sample gate: magnitude of any single bias-corrected angular rate sample (deg/s).
	float gate_sample_deg_s;
	//! Sample gate: how far the mean acceleration magnitude may be from 1g (m/s^2).
	float gate_accel_m_s2;
	//! Sample gate: how far any single acceleration sample may be from the mean (m/s^2).
	float gate_accel_jitter_m_s2;
	//! Time constant of the running acceleration mean (s).
	float accel_window_s;
	//! After a sample gate failed, blocks starting within this time are dropped (s).
	float settle_s;

	//! Rates are collected in blocks of this length (s).
	float block_s;
	//! Block gate: magnitude of a block's mean bias-corrected rate (deg/s).
	float gate_block_deg_s;
	//! Window gate: the gravity-aligned (yaw) residual averaged over the newest N
	//! blocks must stay below gate_yaw_deg_s / sqrt(N) ...
	float gate_yaw_deg_s;
	//! ... but never below this (deg/s); slower directed yaw rotation is learned.
	float gate_yaw_min_deg_s;
	//! Blocks held back, and checked as windows, before the oldest is learned
	//! [1, M_IMU_3DOF_BIAS_EST_MAX_HELD].
	int32_t held_blocks;
	//! Per-axis scatter of the held block means below which the device is
	//! considered off-head; the yaw gate then opens up to @ref max_dev_deg_s (deg/s).
	float quiet_spread_deg_s;

	//! Floor of the per-block measurement noise (deg/s).
	float noise_floor_deg_s;
	//! Prior uncertainty of the bias at init (deg/s).
	float init_sd_deg_s;
	//! Random walk of the true bias, e.g. with temperature (deg/s per sqrt(s)).
	float walk_deg_s_sqrt_s;
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

		//! Running mean of the acceleration, feeds the accel gates and gives "up".
		struct xrt_vec3 accel_mean;
		bool accel_mean_valid;

		//! Blocks starting before this are dropped.
		uint64_t settle_until_ns;

		//! Block currently being accumulated (bias-corrected rate * dt).
		struct xrt_vec3 block_sum;
		double block_dt;
		uint64_t block_start_ns;

		//! Completed blocks awaiting learning, oldest first.
		struct
		{
			struct xrt_vec3 rate; //!< Absolute mean rate (rad/s).
			struct xrt_vec3 up;   //!< Gravity direction (unit) during the block.
			double dt;
		} held[M_IMU_3DOF_BIAS_EST_MAX_HELD];
		int32_t held_count;

		//! Kalman variance of each bias component (rad^2/s^2).
		double var;
		bool var_valid;
		uint64_t last_learn_ns;

		//! Total learned block time (s).
		double learned_s;

		// Debug output.
		struct xrt_vec3 last_block_rate; //!< Bias-corrected mean of the last block.
		float spread_deg_s;              //!< Per-axis scatter of the held blocks.
		float window_yaw_deg_s;          //!< Yaw residual of the full held window.
		float sd_deg_s;                  //!< Current estimate uncertainty.
		bool is_still;
		bool is_quiet;
		uint32_t commits;
		uint32_t rejects_sample;
		uint32_t rejects_block;
		uint32_t rejects_yaw;
	} bias_est;
};

/*!
 * Fill in the default @ref M_IMU_3DOF_AUTO_GYRO_BIAS tunables.
 */
void
m_imu_3dof_gyro_bias_params_default(struct m_imu_3dof_gyro_bias_params *p);

/*!
 * Override tunables from a string like "gate_yaw_min_deg_s=0.1,block_s=3", keys are
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
