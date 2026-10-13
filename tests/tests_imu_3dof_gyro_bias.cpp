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
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>


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

	/*!
	 * Worn-head model, fit to a real worn "hold still" capture (Air 2 Ultra):
	 * per-axis 2 s block means scatter by ~0.25 deg/s and average down faster
	 * than white noise (the head returns), the net head angle stays within a
	 * few degrees, and every few seconds a quick small flick there and back
	 * pushes single samples to 5-10 deg/s.
	 */
	bool worn = false;
	struct Wave
	{
		int axis;
		double amp_rad, w, phase;
	};
	std::vector<Wave> waves;
	double next_flick_s = 3.0;
	double flick_start_s = -10.0;
	int flick_axis = 0;
	double flick_sign = 1.0;

	Sim(bool auto_bias, float gyro_noise_deg_s = 0.05f, float accel_noise_m_s2 = 0.02f)
	    : gyro_noise(0.0f, (float)rad(gyro_noise_deg_s)), accel_noise(0.0f, accel_noise_m_s2)
	{
		int flags = M_IMU_3DOF_USE_GRAVITY_DUR_20MS;
		if (auto_bias) {
			flags |= M_IMU_3DOF_AUTO_GYRO_BIAS;
		}
		m_imu_3dof_init(&fusion, flags);
	}

	void
	wear()
	{
		worn = true;
		std::uniform_real_distribution<double> freq(0.15, 1.2);
		std::uniform_real_distribution<double> phase(0.0, 2.0 * M_PI);
		for (int axis = 0; axis < 3; axis++) {
			for (int k = 0; k < 4; k++) {
				waves.push_back({axis, rad(0.2), 2.0 * M_PI * freq(rng), phase(rng)});
			}
		}
	}

	//! Head-motion angular rate at the current time (rad/s), zero net angle.
	xrt_vec3
	head_rate()
	{
		xrt_vec3 r = {};
		if (!worn) {
			return r;
		}
		double t = (double)now_ns / 1e9;
		float *c = &r.x;
		for (auto &w : waves) {
			c[w.axis] += (float)(w.amp_rad * w.w * std::cos(w.w * t + w.phase));
		}
		if (t >= next_flick_s) {
			flick_start_s = t;
			flick_axis = (int)(rng() % 3);
			flick_sign = (rng() & 1) ? 1.0 : -1.0;
			next_flick_s = t + 2.0 + (double)(rng() % 6000) / 1000.0;
		}
		// 1 deg out and back over 0.4 s: one sine period of rate, peak ~7.9 deg/s.
		double ft = t - flick_start_s;
		if (ft < 0.4) {
			c[flick_axis] +=
			    (float)(flick_sign * rad(1.0) * (M_PI / 0.4) * std::sin(2.0 * M_PI * ft / 0.4));
		}
		return r;
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

		xrt_vec3 head = head_rate();
		true_yaw += head.y / kRateHz;
		xrt_vec3 gyro = {
		    head.x + bias.x + gyro_noise(rng),
		    head.y + (float)yaw_rate + bias.y + gyro_noise(rng),
		    head.z + bias.z + gyro_noise(rng),
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

	//! Least-squares yaw drift rate of the fused-minus-true error over @p seconds (deg/min).
	double
	drift_deg_min(double seconds, double rate_deg_s = 0.0)
	{
		double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
		for (int i = 0; i < (int)(seconds * kRateHz); i++) {
			step(rad(rate_deg_s));
			if (i % 100 != 0) {
				continue;
			}
			double x = i / kRateHz;
			double y = yaw_error_deg();
			n++, sx += x, sy += y, sxx += x * x, sxy += x * y;
		}
		return 60.0 * (n * sxy - sx * sy) / (n * sxx - sx * sx);
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

static constexpr float kRealBias[3] = {-0.06f, 0.04f, 0.38f}; //!< Air 2 Ultra residual after factory cal (deg/s).

static xrt_vec3
real_bias()
{
	return {(float)rad(kRealBias[0]), (float)rad(kRealBias[1]), (float)rad(kRealBias[2])};
}

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

TEST_CASE("imu_3dof_gyro_bias: worn, real-capture statistics")
{
	// The real failure: a 0.38 deg/s stale bias on a horizontal axis plus head
	// sway kept every block above the old 0.25 deg/s magnitude gate.
	Sim base(false);
	Sim sim(true);
	base.wear();
	sim.wear();
	base.bias = real_bias();
	sim.bias = real_bias();

	sim.rest(240);
	double drift = sim.drift_deg_min(120);
	base.rest(240);
	double base_drift = base.drift_deg_min(120);

	xrt_vec3 b = sim.fusion.gyro_bias.value;
	std::printf("worn: bias [%.3f %.3f %.3f] deg/s (true %.2f %.2f %.2f), commits %u, rejects %u/%u/%u\n", deg(b.x),
	            deg(b.y), deg(b.z), kRealBias[0], kRealBias[1], kRealBias[2], sim.fusion.bias_est.commits,
	            sim.fusion.bias_est.rejects_sample, sim.fusion.bias_est.rejects_block,
	            sim.fusion.bias_est.rejects_yaw);
	std::printf("  yaw drift over minutes 4-6: baseline %.2f deg/min, estimator %.2f deg/min\n", base_drift, drift);

	CHECK(sim.fusion.bias_est.commits > 20);
	CHECK(std::abs(sim.yaw_bias_error_deg_s()) < 0.02);
	CHECK(sim.bias_error_len_deg_s() < 0.08);
	CHECK(std::abs(base_drift) > 2.0);
	CHECK(std::abs(drift) < 1.0);
}

TEST_CASE("imu_3dof_gyro_bias: on a table, a large vertical offset is learned")
{
	// Face up on a desk the 0.38 deg/s axis is the yaw axis: far above the yaw
	// gate, but the blocks barely scatter, which no head can do.
	Sim sim(true);
	sim.bias = {(float)rad(kRealBias[0]), (float)rad(kRealBias[2]), (float)rad(kRealBias[1])};
	sim.rest(30);
	double err_30s = sim.bias_error_len_deg_s();
	sim.rest(90);
	double drift = sim.drift_deg_min(120);

	std::printf("table: bias error after 30 s %.4f, after 2 min %.4f deg/s, drift %.3f deg/min, quiet %d\n",
	            err_30s, sim.bias_error_len_deg_s(), drift, sim.fusion.bias_est.is_quiet);
	CHECK(sim.fusion.bias_est.is_quiet);
	CHECK(sim.bias_error_len_deg_s() < 0.005);
	CHECK(std::abs(drift) < 0.2);
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

	// Learning lags by the 32 s held window, then converges within a few blocks.
	CHECK(std::abs(err_60) < 0.01);
	CHECK(std::abs(err_120) < 0.003);
	CHECK(std::abs(yaw_gain) < 3.0);
}

TEST_CASE("imu_3dof_gyro_bias: motion bursts never adapt")
{
	Sim sim(true);
	sim.bias = {0, (float)rad(0.05), 0};

	// 10 minutes of 1 s head turns (+-30 deg/s) separated by 2 s still - after
	// settling never a full block, so the estimator must never learn.
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

/*
 * The documented trade-off: a directed yaw rotation is only distinguishable
 * from bias by its rate and by not scattering like a still head. Above the
 * long-window yaw gate nothing leaks; below it, a pan is (partly) learned as
 * bias and leaks back out as drift after it ends, bounded by the deviation
 * clamp.
 */
static void
run_pan(double rate_deg_s, double seconds, double *out_yaw_err_end, double *out_yaw_err_after, double *out_bias)
{
	Sim sim(true);
	sim.wear();
	sim.bias = real_bias();
	sim.rest(300); // Converged.
	double yaw0 = sim.yaw_error_deg();

	sim.rotate(rate_deg_s, seconds);
	*out_yaw_err_end = sim.yaw_error_deg() - yaw0;
	*out_bias = sim.yaw_bias_error_deg_s();

	sim.rest(300); // Let it re-learn.
	*out_yaw_err_after = sim.yaw_error_deg() - yaw0;
}

TEST_CASE("imu_3dof_gyro_bias: slow pans while worn")
{
	struct
	{
		double rate;
		double seconds;
	} pans[] = {
	    {30.0, 6.0}, {5.0, 20.0}, {2.0, 30.0}, {1.0, 30.0},  {0.5, 30.0},
	    {0.5, 8.0},  {0.3, 60.0}, {0.2, 60.0}, {0.1, 120.0}, {0.05, 300.0},
	};

	// Same seed and sample count without the pan: what the worn estimate drifts anyway.
	double control_end = 0, control_after = 0, control_bias = 0;
	run_pan(0.0, 60.0, &control_end, &control_after, &control_bias);
	std::printf("worn, no pan: yaw err %.3f deg after 60+300 s\n", control_after);

	std::printf("worn pan rate  dur  (angle)  | leak at end | bias err at end | leak 300 s later\n");
	for (auto &p : pans) {
		double end = 0, after = 0, bias = 0;
		run_pan(p.rate, p.seconds, &end, &after, &bias);
		run_pan(0.0, p.seconds, &control_end, &control_after, &control_bias);
		double leak_end = end - control_end;
		double leak_after = after - control_after;
		std::printf("%5.2f deg/s %4.0fs (%5.1f deg) | %8.3f deg | %9.4f deg/s | %8.3f deg\n", p.rate, p.seconds,
		            p.rate * p.seconds, leak_end, bias - control_bias, leak_after);

		if (p.rate >= 0.3) {
			// Above the long-window yaw gate: nothing is learned from the pan. (The
			// leak columns then only show the worn estimate's own noise, ~1 deg/5 min.)
			CHECK(std::abs(bias - control_bias) < 0.01);
		}
		// Whatever is learned stays bounded by the rate.
		CHECK(std::abs(bias - control_bias) <= p.rate + 0.01);
	}
}

TEST_CASE("imu_3dof_gyro_bias: off-head limits and reset")
{
	// A turntable: perfectly smooth 0.8 deg/s about gravity is "quiet", so it is
	// taken for bias (nothing tells them apart), but 1.5 deg/s is over the clamp.
	Sim slow(true);
	slow.bias = {0, (float)rad(0.8), 0};
	slow.rest(300);
	Sim fast(true);
	fast.bias = {0, (float)rad(1.5), 0};
	fast.rest(300);
	// Horizontal offsets are learned up to the deviation clamp.
	Sim horiz(true);
	horiz.bias = {(float)rad(1.5), 0, 0};
	horiz.rest(300);

	std::printf("turntable 0.8 -> %.4f, turntable 1.5 -> %.4f, horizontal 1.5 -> %.4f deg/s\n",
	            deg(slow.fusion.gyro_bias.value.y), deg(fast.fusion.gyro_bias.value.y),
	            deg(horiz.fusion.gyro_bias.value.x));
	CHECK(deg(slow.fusion.gyro_bias.value.y) == Catch::Approx(0.8).margin(0.005));
	CHECK(fast.fusion.bias_est.commits == 0);
	CHECK(deg(horiz.fusion.gyro_bias.value.x) == Catch::Approx(1.0).margin(1e-3));

	// Tuned params survive a reset, the learned state does not.
	horiz.fusion.bias_est.params.gate_block_deg_s = 5.0f;
	m_imu_3dof_reset(&horiz.fusion);
	CHECK(horiz.fusion.bias_est.params.gate_block_deg_s == 5.0f);
	CHECK(horiz.fusion.bias_est.commits == 0);
	CHECK(horiz.fusion.gyro_bias.value.x == 0.0f);
}

TEST_CASE("imu_3dof_gyro_bias: params parse and print round-trip")
{
	struct m_imu_3dof_gyro_bias_params p;
	m_imu_3dof_gyro_bias_params_default(&p);

	CHECK(m_imu_3dof_gyro_bias_params_parse(&p, "block_s=3, gate_yaw_deg_s=0.5 held_blocks=4,enabled=0"));
	CHECK(p.block_s == 3.0f);
	CHECK(p.gate_yaw_deg_s == 0.5f);
	CHECK(p.held_blocks == 4);
	CHECK_FALSE(p.enabled);

	CHECK_FALSE(m_imu_3dof_gyro_bias_params_parse(&p, "nope=1"));
	CHECK_FALSE(m_imu_3dof_gyro_bias_params_parse(&p, "block_s"));
	CHECK_FALSE(m_imu_3dof_gyro_bias_params_parse(&p, "block_s=abc"));

	char buf[1024];
	m_imu_3dof_gyro_bias_params_print(&p, buf, sizeof(buf));
	struct m_imu_3dof_gyro_bias_params q;
	m_imu_3dof_gyro_bias_params_default(&q);
	REQUIRE(m_imu_3dof_gyro_bias_params_parse(&q, buf));
	CHECK(q.block_s == p.block_s);
	CHECK(q.held_blocks == p.held_blocks);
	CHECK(q.enabled == p.enabled);
	CHECK(q.max_dev_deg_s == p.max_dev_deg_s);
}

/*
 * Replay of a real XREAL_AIR_IMU_DUMP capture, hidden by default. Run with
 *   M_IMU_3DOF_CAPTURE=~/xreal-captures/xreal-imu-20261012-212557.csv \
 *     tests_imu_3dof_gyro_bias "[.capture]"
 * The capture is 5.4 min worn, holding still; its whole-capture mean rate is
 * [-0.06 0.04 0.38] deg/s.
 */
TEST_CASE("imu_3dof_gyro_bias: real worn capture", "[.capture]")
{
	const char *path = std::getenv("M_IMU_3DOF_CAPTURE");
	if (path == nullptr) {
		SKIP("M_IMU_3DOF_CAPTURE not set");
	}
	FILE *file = std::fopen(path, "r");
	REQUIRE(file != nullptr);

	struct m_imu_3dof fusion;
	m_imu_3dof_init(&fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS | M_IMU_3DOF_AUTO_GYRO_BIAS);

	char line[1024];
	uint64_t count = 0;
	int64_t last_ns = 0;
	while (std::fgets(line, sizeof(line), file) != nullptr) {
		if (line[0] == '#' || std::strncmp(line, "device_ns", 9) == 0) {
			continue;
		}
		unsigned long long dev = 0;
		long long fus = 0;
		xrt_vec3 g, a;
		if (std::sscanf(line, "%llu,%lld,%*d,%*f,%f,%f,%f,%f,%f,%f", &dev, &fus, &g.x, &g.y, &g.z, &a.x, &a.y,
		                &a.z) != 8 ||
		    fus < last_ns) {
			continue;
		}
		last_ns = fus;
		m_imu_3dof_update(&fusion, (uint64_t)fus, &a, &g);
		count++;
	}
	std::fclose(file);

	xrt_vec3 b = fusion.gyro_bias.value;
	std::printf("capture: %llu samples, %u blocks learned, bias [%.4f %.4f %.4f] deg/s\n",
	            (unsigned long long)count, fusion.bias_est.commits, deg(b.x), deg(b.y), deg(b.z));
	CHECK(fusion.bias_est.commits > 0);
	CHECK(deg(b.y) == Catch::Approx(0.042).margin(0.015));
	CHECK(deg(b.z) == Catch::Approx(0.378).margin(0.05));

	m_imu_3dof_close(&fusion);
}
