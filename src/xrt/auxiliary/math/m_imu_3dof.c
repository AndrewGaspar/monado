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

#include "util/u_misc.h"
#include "util/u_var.h"

#include "math/m_api.h"
#include "math/m_filter_fifo.h"
#include "math/m_imu_3dof.h"
#include "math/m_vec3.h"
#include "math/m_mathinclude.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define DUR_1S_IN_NS (1000 * 1000 * 1000)
#define DUR_300MS_IN_NS (300 * 1000 * 1000)
#define DUR_20MS_IN_NS (20 * 1000 * 1000)

/*!
 * A sample gap this long means the stream stalled, the stillness history is not
 * trustworthy across it.
 */
#define BIAS_EST_MAX_DT_S (0.1)


void
m_imu_3dof_gyro_bias_params_default(struct m_imu_3dof_gyro_bias_params *p)
{
	*p = (struct m_imu_3dof_gyro_bias_params){
	    .enabled = true,
	    .gate_block_deg_s = 0.25f,
	    .gate_sample_deg_s = 3.0f,
	    .gate_accel_m_s2 = 0.5f,
	    .gate_accel_jitter_m_s2 = 0.5f,
	    .accel_window_s = 1.0f,
	    .still_period_s = 3.0f,
	    .block_s = 2.0f,
	    .held_blocks = 2,
	    .tau_s = 20.0f,
	    .fast_tau_s = 5.0f,
	    .fast_period_s = 10.0f,
	    .max_slew_deg_s2 = 0.1f,
	    .max_dev_deg_s = 0.5f,
	};
}

#define BIAS_PARAMS_FLOAT_FIELDS(X)                                                                                    \
	X(gate_block_deg_s)                                                                                            \
	X(gate_sample_deg_s)                                                                                           \
	X(gate_accel_m_s2)                                                                                             \
	X(gate_accel_jitter_m_s2)                                                                                      \
	X(accel_window_s)                                                                                              \
	X(still_period_s)                                                                                              \
	X(block_s)                                                                                                     \
	X(tau_s)                                                                                                       \
	X(fast_tau_s)                                                                                                  \
	X(fast_period_s)                                                                                               \
	X(max_slew_deg_s2)                                                                                             \
	X(max_dev_deg_s)

bool
m_imu_3dof_gyro_bias_params_parse(struct m_imu_3dof_gyro_bias_params *p, const char *str)
{
	const char *c = str;

	while (*c != '\0') {
		// Skip separators.
		if (*c == ',' || *c == ' ' || *c == '\t') {
			c++;
			continue;
		}

		const char *eq = strchr(c, '=');
		if (eq == NULL) {
			return false;
		}
		size_t key_len = (size_t)(eq - c);
		char *end = NULL;
		double value = strtod(eq + 1, &end);
		if (end == eq + 1) {
			return false;
		}

#define KEY_IS(NAME) (key_len == strlen(NAME) && strncmp(c, NAME, key_len) == 0)
#define TRY_FLOAT(NAME)                                                                                                \
	if (KEY_IS(#NAME)) {                                                                                           \
		p->NAME = (float)value;                                                                                \
	} else

		BIAS_PARAMS_FLOAT_FIELDS(TRY_FLOAT)
		if (KEY_IS("enabled")) {
			p->enabled = value != 0.0;
		} else if (KEY_IS("held_blocks")) {
			p->held_blocks = (int32_t)value;
		} else {
			return false;
		}

#undef TRY_FLOAT
#undef KEY_IS

		c = end;
	}

	return true;
}

void
m_imu_3dof_gyro_bias_params_print(const struct m_imu_3dof_gyro_bias_params *p, char *buf, size_t size)
{
	int n = snprintf(buf, size, "enabled=%d,held_blocks=%d", p->enabled ? 1 : 0, (int)p->held_blocks);

#define PRINT_FLOAT(NAME)                                                                                              \
	if (n >= 0 && (size_t)n < size) {                                                                              \
		n += snprintf(buf + n, size - (size_t)n, "," #NAME "=%g", (double)p->NAME);                            \
	}

	BIAS_PARAMS_FLOAT_FIELDS(PRINT_FLOAT)

#undef PRINT_FLOAT
}

static void
bias_est_restart(struct m_imu_3dof *f, uint64_t timestamp_ns)
{
	f->bias_est.still_start_ns = timestamp_ns;
	f->bias_est.block_sum = (struct xrt_vec3)XRT_VEC3_ZERO;
	f->bias_est.block_dt = 0.0;
	f->bias_est.block_start_ns = timestamp_ns;
	f->bias_est.held_count = 0;
	f->bias_est.is_still = false;
	f->bias_est.still_s = 0.0f;
}

void
m_imu_3dof_init(struct m_imu_3dof *f, int flags)
{
	U_ZERO(f);
	f->rot.w = 1.0f;

	m_ff_vec3_f32_alloc(&f->word_accel_ff, 1000);
	m_ff_vec3_f32_alloc(&f->gyro_ff, 1000);

	f->flags = flags;

	m_imu_3dof_gyro_bias_params_default(&f->bias_est.params);
}

void
m_imu_3dof_reset(struct m_imu_3dof *f)
{
	struct m_ff_vec3_f32 *aff = f->word_accel_ff;
	struct m_ff_vec3_f32 *gff = f->gyro_ff;
	int flags = f->flags;
	struct m_imu_3dof_gyro_bias_params params = f->bias_est.params;

	U_ZERO(f);
	f->rot.w = 1.0f;
	f->word_accel_ff = aff;
	f->gyro_ff = gff;
	f->flags = flags;
	f->bias_est.params = params;
}

void
m_imu_3dof_close(struct m_imu_3dof *f)
{
	m_ff_vec3_f32_free(&f->word_accel_ff);
	m_ff_vec3_f32_free(&f->gyro_ff);
}

void
m_imu_3dof_add_vars(struct m_imu_3dof *f, void *root, const char *prefix)
{
	char tmp[512];
	snprintf(tmp, sizeof(tmp), "%sff.world_accel", prefix);
	u_var_add_ro_ff_vec3_f32(root, f->word_accel_ff, tmp);
	snprintf(tmp, sizeof(tmp), "%sff.gyro", prefix);
	u_var_add_ro_ff_vec3_f32(root, f->gyro_ff, tmp);

	snprintf(tmp, sizeof(tmp), "%slast.timestamp_ns", prefix);
	u_var_add_ro_u64(root, &f->last.timestamp_ns, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.gyro", prefix);
	u_var_add_ro_vec3_f32(root, &f->last.gyro, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.gyro_length", prefix);
	u_var_add_ro_f32(root, &f->last.gyro_length, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.gyro_biased_length", prefix);
	u_var_add_ro_f32(root, &f->last.gyro_biased_length, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.accel", prefix);
	u_var_add_ro_vec3_f32(root, &f->last.accel, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.accel_length", prefix);
	u_var_add_ro_f32(root, &f->last.accel_length, tmp);
	snprintf(tmp, sizeof(tmp), "%slast.delta_ms", prefix);
	u_var_add_ro_f64(root, &f->last.delta_ms, tmp);

	snprintf(tmp, sizeof(tmp), "%sgrav.level_timestamp_ns", prefix);
	u_var_add_ro_u64(root, &f->grav.level_timestamp_ns, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.error_axis", prefix);
	u_var_add_ro_vec3_f32(root, &f->grav.error_axis, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.error_angle", prefix);
	u_var_add_ro_f32(root, &f->grav.error_angle, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.is_accel", prefix);
	u_var_add_bool(root, &f->grav.is_accel, tmp);
	snprintf(tmp, sizeof(tmp), "%sgrav.is_rotating", prefix);
	u_var_add_bool(root, &f->grav.is_rotating, tmp);

	snprintf(tmp, sizeof(tmp), "%sgyro_bias.value", prefix);
	u_var_add_ro_vec3_f32(root, &f->gyro_bias.value, tmp);
	snprintf(tmp, sizeof(tmp), "%sgyro_bias.manually_fire", prefix);
	u_var_add_bool(root, &f->gyro_bias.manually_fire, tmp);

	if ((f->flags & M_IMU_3DOF_AUTO_GYRO_BIAS) == 0) {
		return;
	}

	struct m_imu_3dof_gyro_bias_params *p = &f->bias_est.params;

#define ADD_F32(NAME)                                                                                                  \
	snprintf(tmp, sizeof(tmp), "%sbias_est." #NAME, prefix);                                                       \
	u_var_add_f32(root, &p->NAME, tmp);

	snprintf(tmp, sizeof(tmp), "%sbias_est.enabled", prefix);
	u_var_add_bool(root, &p->enabled, tmp);
	ADD_F32(gate_block_deg_s);
	ADD_F32(gate_sample_deg_s);
	ADD_F32(gate_accel_m_s2);
	ADD_F32(gate_accel_jitter_m_s2);
	ADD_F32(accel_window_s);
	ADD_F32(still_period_s);
	ADD_F32(block_s);
	snprintf(tmp, sizeof(tmp), "%sbias_est.held_blocks", prefix);
	u_var_add_i32(root, &p->held_blocks, tmp);
	ADD_F32(tau_s);
	ADD_F32(fast_tau_s);
	ADD_F32(fast_period_s);
	ADD_F32(max_slew_deg_s2);
	ADD_F32(max_dev_deg_s);

#undef ADD_F32

	snprintf(tmp, sizeof(tmp), "%sbias_est.origin", prefix);
	u_var_add_ro_vec3_f32(root, &f->bias_est.origin, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.last_block_rate", prefix);
	u_var_add_ro_vec3_f32(root, &f->bias_est.last_block_rate, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.accel_mean", prefix);
	u_var_add_ro_vec3_f32(root, &f->bias_est.accel_mean, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.is_still", prefix);
	u_var_add_bool(root, &f->bias_est.is_still, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.still_s", prefix);
	u_var_add_ro_f32(root, &f->bias_est.still_s, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.learned_s", prefix);
	u_var_add_ro_f64(root, &f->bias_est.learned_s, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.commits", prefix);
	u_var_add_ro_u32(root, &f->bias_est.commits, tmp);
	snprintf(tmp, sizeof(tmp), "%sbias_est.held_count", prefix);
	u_var_add_ro_i32(root, &f->bias_est.held_count, tmp);
}

static void
gravity_correction(struct m_imu_3dof *f,
                   uint64_t timestamp_ns,
                   const struct xrt_vec3 *accel,
                   const struct xrt_vec3 *gyro,
                   double dt,
                   float gyro_length)
{
	uint64_t dur_ns = 0;
	if (f->flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) {
		dur_ns = DUR_20MS_IN_NS;
	} else if (f->flags & M_IMU_3DOF_USE_GRAVITY_DUR_300MS) {
		dur_ns = DUR_300MS_IN_NS;
	} else {
		return;
	}

	const float gravity_tolerance = .9f;
	const float gyro_tolerance = .1f;
	const float min_tilt_error = 0.05f;
	const float max_tilt_error = 0.01f;

	/*
	 * If the device is within tolerance levels, count this
	 * as the device is level and add to the counter otherwise
	 * reset the counter and start over.
	 */

	float accel_length = m_vec3_len(*accel);
	bool is_accel = fabsf(accel_length - 9.82f) >= gravity_tolerance;
	bool is_rotating = gyro_length >= gyro_tolerance;
	if (is_accel || is_rotating) {
		f->grav.level_timestamp_ns = timestamp_ns;
	}
	f->grav.is_accel = is_accel;
	f->grav.is_rotating = is_rotating;

	/*
	 * Device has been level for long enough, grab mean from the
	 * accelerometer filter queue (last n values) and use for correction.
	 */
	uint64_t level_ns = f->grav.level_timestamp_ns + dur_ns;
	if (level_ns < timestamp_ns) {
		// Reset the timepoint
		f->grav.level_timestamp_ns = timestamp_ns;

		struct xrt_vec3 accel_mean;
		m_ff_vec3_f32_filter(f->word_accel_ff,      // Filter
		                     timestamp_ns - dur_ns, // Start time
		                     timestamp_ns,          // End time
		                     &accel_mean);          // Results
		if ((m_vec3_len(accel_mean) - 9.82f) < gravity_tolerance) {
			/*
			 * Calculate a cross product between what the device
			 * thinks is up and what gravity indicates is down.
			 * The values are optimized of what we would get out
			 * from the cross product.
			 */
			struct xrt_vec3 tilt = {
			    accel_mean.z,
			    0,
			    -accel_mean.x,
			};

			tilt = m_vec3_normalize(tilt);
			accel_mean = m_vec3_normalize(accel_mean);

			struct xrt_vec3 up = {0, 1.0f, 0};
			float tilt_angle = m_vec3_angle(up, accel_mean);

			if (tilt_angle > max_tilt_error) {
				f->grav.error_angle = tilt_angle;
				f->grav.error_axis = tilt;
			}
		}
	}

	if (f->grav.error_angle > min_tilt_error) {
		// Correct 180° over 5 seconds, when moving.
		float max_radians = (float)M_PI * (float)dt / 5;
		// Correct 180° over 60 seconds, when stationary.
		float min_radians = (float)M_PI * (float)dt / 60;

		/*
		 * We're treating 0.5 * gyro_length as a unitless scale factor.
		 * Tested in a headset, 0.5 felt nice.
		 */
		float correction_radians = 0.5f * gyro_length * max_radians;
		// Clamp to the range [min_radians, max_radians]
		correction_radians = fmaxf(min_radians, correction_radians);
		correction_radians = fminf(max_radians, correction_radians);
		// Do not exceed the remaining error to correct for
		correction_radians = -fminf(correction_radians, f->grav.error_angle);

		// Update how much is left.
		f->grav.error_angle += correction_radians;

		// Perform the correction.
		struct xrt_quat corr_quat;
		struct xrt_quat old_orient;
		math_quat_from_angle_vector(correction_radians, &f->grav.error_axis, &corr_quat);
		old_orient = f->rot;
		math_quat_rotate(&corr_quat, &old_orient, &f->rot);
	}
}

static void
gyro_biasing(struct m_imu_3dof *f, uint64_t timestamp_ns)
{
	if (!f->gyro_bias.manually_fire) {
		return;
	}

	f->gyro_bias.manually_fire = false;

	uint64_t dur_ns = DUR_300MS_IN_NS;

	struct xrt_vec3 gyro_mean = XRT_VEC3_ZERO;
	m_ff_vec3_f32_filter(f->gyro_ff,            // Filter
	                     timestamp_ns - dur_ns, // Start time
	                     timestamp_ns,          // End time
	                     &gyro_mean);           // Results

	f->gyro_bias.value = gyro_mean;
}

static void
bias_est_learn(struct m_imu_3dof *f, struct xrt_vec3 rate, double dt)
{
	const struct m_imu_3dof_gyro_bias_params *p = &f->bias_est.params;

	// Residual against the current estimate; the held rate is absolute.
	struct xrt_vec3 residual = m_vec3_sub(rate, f->gyro_bias.value);

	float tau = f->bias_est.learned_s < (double)p->fast_period_s ? p->fast_tau_s : p->tau_s;
	tau = fmaxf(tau, 0.001f);
	float k = (float)(1.0 - exp(-dt / (double)tau));
	struct xrt_vec3 step = m_vec3_mul_scalar(residual, k);

	// Never snap: limit how fast the estimate may move.
	float max_step = (float)DEG_TO_RAD(p->max_slew_deg_s2) * (float)dt;
	float step_len = m_vec3_len(step);
	if (step_len > max_step) {
		step = m_vec3_mul_scalar(step, max_step / step_len);
	}

	// Never stray far from where we started (factory calibration).
	struct xrt_vec3 dev = m_vec3_sub(m_vec3_add(f->gyro_bias.value, step), f->bias_est.origin);
	float max_dev = (float)DEG_TO_RAD(p->max_dev_deg_s);
	float dev_len = m_vec3_len(dev);
	if (dev_len > max_dev) {
		dev = m_vec3_mul_scalar(dev, max_dev / dev_len);
	}

	f->gyro_bias.value = m_vec3_add(f->bias_est.origin, dev);
	f->bias_est.learned_s += dt;
	f->bias_est.commits++;
}

static void
bias_est_finish_block(struct m_imu_3dof *f, uint64_t timestamp_ns)
{
	const struct m_imu_3dof_gyro_bias_params *p = &f->bias_est.params;

	struct xrt_vec3 corrected = m_vec3_mul_scalar(f->bias_est.block_sum, (float)(1.0 / f->bias_est.block_dt));
	f->bias_est.last_block_rate = corrected;

	/*
	 * A directed rotation shows up as a block mean at its full rate, while
	 * oscillating head sway mostly cancels within a block. A failing block also
	 * throws away the held ones: they may hold the slow start of this motion.
	 */
	if (m_vec3_len(corrected) >= (float)DEG_TO_RAD(p->gate_block_deg_s)) {
		bias_est_restart(f, timestamp_ns);
		return;
	}

	// Every sample in the block was corrected by the current value, store it absolute.
	struct xrt_vec3 rate = m_vec3_add(corrected, f->gyro_bias.value);

	int32_t depth = p->held_blocks;
	depth = depth < 1 ? 1 : depth;
	depth = depth > M_IMU_3DOF_BIAS_EST_MAX_HELD ? M_IMU_3DOF_BIAS_EST_MAX_HELD : depth;

	// Enough newer blocks passed: learn from the oldest one.
	while (f->bias_est.held_count >= depth) {
		uint64_t still_period_ns = (uint64_t)((double)p->still_period_s * DUR_1S_IN_NS);
		if (f->bias_est.held[0].start_ns >= f->bias_est.still_start_ns + still_period_ns) {
			bias_est_learn(f, f->bias_est.held[0].rate, f->bias_est.held[0].dt);
		}

		f->bias_est.held_count--;
		for (int32_t i = 0; i < f->bias_est.held_count; i++) {
			f->bias_est.held[i] = f->bias_est.held[i + 1];
		}
	}

	int32_t i = f->bias_est.held_count++;
	f->bias_est.held[i].rate = rate;
	f->bias_est.held[i].dt = f->bias_est.block_dt;
	f->bias_est.held[i].start_ns = f->bias_est.block_start_ns;

	f->bias_est.block_sum = (struct xrt_vec3)XRT_VEC3_ZERO;
	f->bias_est.block_dt = 0.0;
	f->bias_est.block_start_ns = timestamp_ns;
}

/*!
 * Stillness-gated gyro bias tracking, see @ref M_IMU_3DOF_AUTO_GYRO_BIAS.
 *
 * The gates look at the bias-corrected rate, so at rest they see the residual
 * bias plus noise. Per sample: no rate spike, no acceleration jitter and ~1g.
 * Per block: a low mean rate. Learning is deferred by @ref
 * m_imu_3dof_gyro_bias_params::held_blocks blocks so the slow start of a motion
 * (before any gate trips) is thrown away instead of learned.
 */
static void
bias_est_update(struct m_imu_3dof *f,
                uint64_t timestamp_ns,
                const struct xrt_vec3 *accel,
                const struct xrt_vec3 *gyro_biased,
                double dt)
{
	const struct m_imu_3dof_gyro_bias_params *p = &f->bias_est.params;

	if ((f->flags & M_IMU_3DOF_AUTO_GYRO_BIAS) == 0 || !p->enabled) {
		f->bias_est.accel_mean_valid = false;
		f->bias_est.is_still = false;
		return;
	}

	if (!f->bias_est.accel_mean_valid || dt > BIAS_EST_MAX_DT_S) {
		f->bias_est.accel_mean = *accel;
		f->bias_est.accel_mean_valid = true;
		bias_est_restart(f, timestamp_ns);
		return;
	}

	float alpha = (float)(dt / ((double)p->accel_window_s + dt));
	f->bias_est.accel_mean = m_vec3_lerp(f->bias_est.accel_mean, *accel, alpha);

	float gyro_len = m_vec3_len(*gyro_biased);
	float accel_mean_err = fabsf(m_vec3_len(f->bias_est.accel_mean) - (float)MATH_GRAVITY_M_S2);
	float accel_jitter = m_vec3_len(m_vec3_sub(*accel, f->bias_est.accel_mean));

	bool still = gyro_len < (float)DEG_TO_RAD(p->gate_sample_deg_s) && //
	             accel_mean_err < p->gate_accel_m_s2 &&                //
	             accel_jitter < p->gate_accel_jitter_m_s2;

	if (!still) {
		bias_est_restart(f, timestamp_ns);
		return;
	}

	f->bias_est.is_still = true;
	f->bias_est.still_s = (float)((double)(timestamp_ns - f->bias_est.still_start_ns) / DUR_1S_IN_NS);

	f->bias_est.block_sum = m_vec3_add(f->bias_est.block_sum, m_vec3_mul_scalar(*gyro_biased, (float)dt));
	f->bias_est.block_dt += dt;

	if (f->bias_est.block_dt >= (double)p->block_s) {
		bias_est_finish_block(f, timestamp_ns);
	}
}

void
m_imu_3dof_update(struct m_imu_3dof *f,
                  uint64_t timestamp_ns,
                  const struct xrt_vec3 *accel,
                  const struct xrt_vec3 *gyro)
{
	//! Skip the first sample.
	if (f->state == M_IMU_3DOF_STATE_START) {
		f->state = M_IMU_3DOF_STATE_RUNNING;
		f->last.timestamp_ns = timestamp_ns;
		return;
	}

	// This code assumes all timestamps makes some forward progress.
	assert(timestamp_ns >= f->last.timestamp_ns);

	f->last.gyro = *gyro;
	f->last.accel = *accel;

	struct xrt_vec3 world_accel = {0};
	math_quat_rotate_vec3(&f->rot, accel, &world_accel);

	uint64_t diff = timestamp_ns - f->last.timestamp_ns;
	double dt = (double)diff / DUR_1S_IN_NS;

	f->last.delta_ms = dt * 1000.0f;
	f->last.timestamp_ns = timestamp_ns;

	m_ff_vec3_f32_push(f->word_accel_ff, &world_accel, timestamp_ns);
	m_ff_vec3_f32_push(f->gyro_ff, gyro, timestamp_ns);

	struct xrt_vec3 gyro_biased = m_vec3_sub(*gyro, f->gyro_bias.value);
	float gyro_biased_length = m_vec3_len(gyro_biased);
	float gyro_length = m_vec3_len(*gyro);
	float accel_length = m_vec3_len(*accel);

	f->last.accel_length = accel_length;
	f->last.gyro_length = gyro_length;
	f->last.gyro_biased_length = gyro_biased_length;


	if (gyro_biased_length > 0.0001f) {
#if 0
		math_quat_integrate_velocity(&f->rot, gyro, dt, &f->rot);
#else
		struct xrt_vec3 rot_axis = {
		    gyro_biased.x / gyro_biased_length,
		    gyro_biased.y / gyro_biased_length,
		    gyro_biased.z / gyro_biased_length,
		};

		float rot_angle = gyro_biased_length * (float)dt;

		struct xrt_quat delta_orient;
		math_quat_from_angle_vector(rot_angle, &rot_axis, &delta_orient);

		math_quat_rotate(&f->rot, &delta_orient, &f->rot);
#endif
	}

	// Gravity correction.
	gravity_correction(f, timestamp_ns, accel, &gyro_biased, dt, gyro_biased_length);

	// Gyro bias calculations.
	gyro_biasing(f, timestamp_ns);
	bias_est_update(f, timestamp_ns, accel, &gyro_biased, dt);

	/*
	 * Mitigate drift due to floating point
	 * inprecision with quat multiplication.
	 */
	math_quat_normalize(&f->rot);
}
