// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Synthetic-stream tests for the m_imu_3dof automatic gyro bias estimator.
 */

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_mathinclude.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstdio>
#include <random>


static constexpr double kRateHz = 1000.0;
static constexpr uint64_t kStepNs = 1000000; // 1 kHz, like the XREAL Air 2 Ultra.

static double
deg(double rad)
{
	return rad * 180.0 / M_PI;
}

static double
rad(double deg)
{
	return deg * M_PI / 180.0;
}

/*!
 * A level device rotating only about the world up axis (+Y), fed to the fusion
 * at 1 kHz with a configurable gyro bias and white sensor noise.
 */
struct Sim
{
	struct m_imu_3dof fusion = {};
	std::mt19937 rng{1234};
	std::normal_distribution<float> gyro_noise;
	std::normal_distribution<float> accel_noise;

	uint64_t now_ns = 1000000000;
	double true_yaw = 0; //!< Integrated true yaw (rad).

	xrt_vec3 bias = {}; //!< Sensor bias (rad/s), what the estimator has to find.

	Sim(bool auto_bias, float gyro_noise_deg_s = 0.05f, float accel_noise_m_s2 = 0.02f)
	    : gyro_noise(0.0f, (float)rad(gyro_noise_deg_s)), accel_noise(0.0f, accel_noise_m_s2)
	{
		int flags = M_IMU_3DOF_USE_GRAVITY_DUR_20MS;
		if (auto_bias) {
			flags |= M_IMU_3DOF_AUTO_GYRO_BIAS;
		}
		m_imu_3dof_init(&fusion, flags);
	}

	~Sim()
	{
		m_imu_3dof_close(&fusion);
	}

	//! One sample with a true yaw rate (rad/s) and an extra body-frame accel.
	void
	step(double yaw_rate, xrt_vec3 extra_accel = {})
	{
		now_ns += kStepNs;
		true_yaw += yaw_rate / kRateHz;

		xrt_vec3 gyro = {
		    bias.x + gyro_noise(rng),
		    (float)yaw_rate + bias.y + gyro_noise(rng),
		    bias.z + gyro_noise(rng),
		};
		xrt_vec3 accel = {
		    extra_accel.x + accel_noise(rng),
		    (float)MATH_GRAVITY_M_S2 + extra_accel.y + accel_noise(rng),
		    extra_accel.z + accel_noise(rng),
		};
		m_imu_3dof_update(&fusion, now_ns, &accel, &gyro);
	}

	void
	rest(double seconds)
	{
		for (int i = 0; i < (int)(seconds * kRateHz); i++) {
			step(0.0);
		}
	}

	void
	rotate(double rate_deg_s, double seconds)
	{
		for (int i = 0; i < (int)(seconds * kRateHz); i++) {
			step(rad(rate_deg_s));
		}
	}

	//! Fused yaw about +Y (rad), from where the forward vector points.
	double
	fused_yaw() const
	{
		xrt_vec3 fwd = {0, 0, -1};
		xrt_vec3 out;
		math_quat_rotate_vec3(&fusion.rot, &fwd, &out);
		return atan2(-out.x, -out.z);
	}

	//! Fused minus true yaw, wrapped (deg).
	double
	yaw_error_deg() const
	{
		double e = fused_yaw() - true_yaw;
		e = std::remainder(e, 2.0 * M_PI);
		return deg(e);
	}

	//! Remaining bias error on the yaw axis (deg/s).
	double
	yaw_bias_error_deg_s() const
	{
		return deg(bias.y - fusion.gyro_bias.value.y);
	}

	double
	bias_error_len_deg_s() const
	{
		xrt_vec3 d = {bias.x - fusion.gyro_bias.value.x, bias.y - fusion.gyro_bias.value.y,
		              bias.z - fusion.gyro_bias.value.z};
		return deg(std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
	}
};

TEST_CASE("imu_3dof_gyro_bias: flag off keeps the old behaviour")
{
	Sim sim(false);
	sim.bias = {0, (float)rad(0.05), 0};
	sim.rest(120);

	CHECK(sim.fusion.gyro_bias.value.x == 0.0f);
	CHECK(sim.fusion.gyro_bias.value.y == 0.0f);
	CHECK(sim.fusion.gyro_bias.value.z == 0.0f);
	CHECK(sim.fusion.bias_est.commits == 0);
	// ~0.05 deg/s * 120 s = 6 deg of yaw drift.
	CHECK(std::abs(sim.yaw_error_deg()) > 5.0);
}

TEST_CASE("imu_3dof_gyro_bias: constant bias at rest converges and bounds drift")
{
	// Observed on the XREAL 1S: ~2 deg/min = 0.033 deg/s yaw drift at rest.
	const xrt_vec3 bias = {(float)rad(0.02), (float)rad(0.05), (float)rad(-0.03)};

	Sim base(false);
	Sim sim(true);
	base.bias = bias;
	sim.bias = bias;

	base.rest(15 * 60);
	sim.rest(30);
	double err_30s = sim.bias_error_len_deg_s();
	double yaw_30s = sim.yaw_error_deg();
	sim.rest(15 * 60 - 30);

	std::printf("rest 15 min: baseline yaw drift %.2f deg, estimator yaw drift %.3f deg\n", base.yaw_error_deg(),
	            sim.yaw_error_deg());
	std::printf("  bias error after 30 s %.4f deg/s, after 15 min %.5f deg/s, commits %u\n", err_30s,
	            sim.bias_error_len_deg_s(), sim.fusion.bias_est.commits);

	CHECK(std::abs(base.yaw_error_deg()) > 40.0);
	// Startup fast-converge: most of the 0.062 deg/s bias is gone within 30 s.
	CHECK(err_30s < 0.01);
	CHECK(sim.bias_error_len_deg_s() < 0.003);
	// Whole-run yaw drift is bounded to what accumulated while converging.
	CHECK(std::abs(sim.yaw_error_deg()) < 1.0);
	// And after convergence there is essentially no further drift.
	CHECK(std::abs(sim.yaw_error_deg() - yaw_30s) < 0.5);
}

TEST_CASE("imu_3dof_gyro_bias: thermal step in bias re-converges")
{
	Sim sim(true);
	sim.bias = {0, (float)rad(0.03), 0};
	sim.rest(5 * 60);
	REQUIRE(std::abs(sim.yaw_bias_error_deg_s()) < 0.003);

	// Warm-up walk: bias steps by 0.06 deg/s (twice the 10 min walk seen on the 1S).
	sim.bias.y = (float)rad(0.09);
	double yaw0 = sim.yaw_error_deg();
	sim.rest(60);
	double err_60 = sim.yaw_bias_error_deg_s();
	sim.rest(60);
	double err_120 = sim.yaw_bias_error_deg_s();
	double yaw_gain = sim.yaw_error_deg() - yaw0;

	std::printf("bias step 0.03->0.09 deg/s: error after 60 s %.4f, after 120 s %.5f deg/s, yaw lost %.2f deg\n",
	            err_60, err_120, yaw_gain);

	// Time constant 20 s (+3.5 s gate latency): ~5% left after 60 s.
	CHECK(std::abs(err_60) < 0.006);
	CHECK(std::abs(err_120) < 0.002);
	// Total yaw lost to the step ~= step * tau = 0.06 * ~23 s.
	CHECK(std::abs(yaw_gain) < 2.0);
}

TEST_CASE("imu_3dof_gyro_bias: motion bursts never adapt")
{
	Sim sim(true);
	sim.bias = {0, (float)rad(0.05), 0};

	// 10 minutes of 1 s head turns (+-30 deg/s) separated by 2 s still - always
	// shorter than the still period, so the estimator must never learn.
	for (int i = 0; i < 200; i++) {
		double dir = (i % 2) ? 1.0 : -1.0;
		for (int s = 0; s < 1000; s++) {
			double phase = M_PI * s / 1000.0;
			sim.step(dir * rad(30.0) * std::sin(phase));
		}
		sim.rest(2.0);
	}

	std::printf("motion bursts: commits %u, bias.y %.5f deg/s\n", sim.fusion.bias_est.commits,
	            deg(sim.fusion.gyro_bias.value.y));
	CHECK(sim.fusion.bias_est.commits == 0);
	CHECK(sim.fusion.gyro_bias.value.y == 0.0f);
}

TEST_CASE("imu_3dof_gyro_bias: linear acceleration blocks adaptation")
{
	Sim sim(true);
	sim.bias = {0, (float)rad(0.05), 0};

	// Riding in something bumpy: no rotation, but 1 m/s^2 jolts every 0.5 s.
	for (int i = 0; i < 120 * 2; i++) {
		for (int s = 0; s < 500; s++) {
			xrt_vec3 jolt = {s < 50 ? 1.0f : 0.0f, 0, 0};
			sim.step(0.0, jolt);
		}
	}
	CHECK(sim.fusion.bias_est.commits == 0);
}

TEST_CASE("imu_3dof_gyro_bias: zero-mean head sway still learns")
{
	Sim sim(true);
	sim.bias = {0, (float)rad(0.05), 0};

	// Worn and fixating: +-0.2 deg sway at 0.3 Hz (peak 0.38 deg/s) - oscillatory,
	// so it mostly cancels within a block; what is left averages out over blocks.
	const double amp = rad(0.2);
	const double w = 2.0 * M_PI * 0.3;
	for (int i = 0; i < 300 * 1000; i++) {
		double t = i / kRateHz;
		sim.step(amp * w * std::cos(w * t));
	}

	std::printf("head sway: bias error %.4f deg/s, commits %u\n", sim.yaw_bias_error_deg_s(),
	            sim.fusion.bias_est.commits);
	CHECK(sim.fusion.bias_est.commits > 0);
	CHECK(std::abs(sim.yaw_bias_error_deg_s()) < 0.015);
}

/*
 * The documented trade-off: a directed rotation is only distinguishable from
 * bias by its rate. Above the mean gate nothing leaks; below it, a pan that
 * outlasts the still period is (partly) learned as bias and leaks back out as
 * drift after it ends, bounded by the deviation clamp.
 */
static void
run_pan(double rate_deg_s, double seconds, double *out_yaw_err_end, double *out_yaw_err_after, double *out_bias)
{
	Sim sim(true);
	sim.bias = {0, (float)rad(0.03), 0};
	sim.rest(120); // Converged.
	double yaw0 = sim.yaw_error_deg();

	sim.rotate(rate_deg_s, seconds);
	*out_yaw_err_end = sim.yaw_error_deg() - yaw0;
	*out_bias = sim.yaw_bias_error_deg_s();

	sim.rest(120); // Let it re-learn.
	*out_yaw_err_after = sim.yaw_error_deg() - yaw0;
}

TEST_CASE("imu_3dof_gyro_bias: slow pans")
{
	struct
	{
		double rate;
		double seconds;
	} pans[] = {
	    {30.0, 6.0}, {5.0, 20.0}, {2.0, 30.0}, {1.0, 30.0}, {0.5, 30.0}, {0.3, 60.0}, {0.2, 60.0}, {0.1, 120.0},
	};

	std::printf("pan rate  dur  | yaw err at end | bias err at end | yaw err 120s later\n");
	for (auto &p : pans) {
		double end = 0, after = 0, bias = 0;
		run_pan(p.rate, p.seconds, &end, &after, &bias);
		std::printf("%5.2f deg/s %4.0fs | %8.3f deg   | %9.4f deg/s | %8.3f deg\n", p.rate, p.seconds, end,
		            bias, after);

		if (p.rate > 0.25) {
			// Above the block gate: no leak at all.
			CHECK(std::abs(end) < 0.1);
			CHECK(std::abs(after) < 0.2);
		} else {
			// Below it: the learned bias is bounded by the rate and the deviation clamp.
			CHECK(std::abs(bias) <= p.rate + 0.01);
			CHECK(std::abs(bias) <= 0.5 + 0.01);
		}
	}
}


TEST_CASE("imu_3dof_gyro_bias: deviation clamp and reset")
{
	Sim sim(true);
	// Open the gate wide so a large bias is visible to the estimator at all.
	sim.fusion.bias_est.params.gate_block_deg_s = 5.0f;
	sim.bias = {0, (float)rad(1.0), 0};
	sim.rest(300);

	std::printf("clamp: true bias 1.0 deg/s, estimate %.4f deg/s\n", deg(sim.fusion.gyro_bias.value.y));
	CHECK(deg(sim.fusion.gyro_bias.value.y) == Catch::Approx(0.5).margin(1e-4));

	// Tuned params survive a reset, the learned state does not.
	m_imu_3dof_reset(&sim.fusion);
	CHECK(sim.fusion.bias_est.params.gate_block_deg_s == 5.0f);
	CHECK(sim.fusion.bias_est.commits == 0);
	CHECK(sim.fusion.gyro_bias.value.y == 0.0f);
}
