// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tests for persisting the Xreal Air learned gyro bias across sessions.
 */

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_mathinclude.h"

#include "util/u_file.h"

#include "xreal_air/xreal_air_gyro_bias_store.h"

#include "catch_amalgamated.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <unistd.h>


static double
rad(double deg)
{
	return deg * M_PI / 180.0;
}

//! A fresh XDG_STATE_HOME for one test, removed again after.
struct StateDir
{
	std::string dir;
	std::string old;
	bool had_old = false;

	StateDir()
	{
		char tmpl[] = "/tmp/xreal_air_bias_store_XXXXXX";
		REQUIRE(mkdtemp(tmpl) != nullptr);
		dir = tmpl;
		const char *o = std::getenv("XDG_STATE_HOME");
		had_old = o != nullptr;
		old = had_old ? o : "";
		setenv("XDG_STATE_HOME", dir.c_str(), 1);
	}

	~StateDir()
	{
		std::string cmd = "rm -rf '" + dir + "'";
		(void)std::system(cmd.c_str());
		if (had_old) {
			setenv("XDG_STATE_HOME", old.c_str(), 1);
		} else {
			unsetenv("XDG_STATE_HOME");
		}
	}
};

//! A device lying still with a gyro bias, fed at 1 kHz.
struct Still
{
	struct m_imu_3dof fusion = {};
	uint64_t now_ns = 1000000000;
	xrt_vec3 bias;

	explicit Still(xrt_vec3 b) : bias(b)
	{
		m_imu_3dof_init(&fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS | M_IMU_3DOF_AUTO_GYRO_BIAS);
	}

	~Still()
	{
		m_imu_3dof_close(&fusion);
	}

	void
	run(double seconds, struct xreal_air_gyro_bias_store *store)
	{
		xrt_vec3 accel = {0, (float)MATH_GRAVITY_M_S2, 0};
		for (int i = 0; i < (int)(seconds * 1000.0); i++) {
			now_ns += 1000000;
			m_imu_3dof_update(&fusion, now_ns, &accel, &bias);
			xreal_air_gyro_bias_store_tick(store, &fusion, 40.0f, (int64_t)now_ns);
		}
	}
};

static bool
wait_for_file(const char *path)
{
	for (int i = 0; i < 200; i++) {
		if (access(path, F_OK) == 0) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
}

static void
write_file(const char *path, const std::string &text)
{
	REQUIRE(u_file_write_atomic(path, text.data(), text.size()) == 0);
}

TEST_CASE("xreal_air_gyro_bias_store: learn, save, reload, seed")
{
	StateDir state;
	struct m_imu_3dof_gyro_bias_params params;
	m_imu_3dof_gyro_bias_params_default(&params);

	const xrt_vec3 truth = {(float)rad(-0.02), (float)rad(0.053), (float)rad(0.30)};
	std::string path;

	{
		struct xreal_air_gyro_bias_store *store =
		    xreal_air_gyro_bias_store_create("ZBBM5DZFMP", &params, U_LOGGING_INFO);
		REQUIRE(store != nullptr);
		path = xreal_air_gyro_bias_store_path(store);
		CHECK(path == state.dir + "/monado/xreal_air_gyro_bias_ZBBM5DZFMP.txt");

		struct m_imu_3dof_gyro_bias_record rec;
		CHECK_FALSE(xreal_air_gyro_bias_store_get_loaded(store, &rec));

		// Nothing learned yet: nothing saved, not even at destroy.
		Still still(truth);
		still.run(10, store);
		CHECK(still.fusion.bias_est.commits == 0);
		CHECK(access(path.c_str(), F_OK) != 0);

		// The first learned block is saved right away, by the writer thread.
		still.run(40, store);
		REQUIRE(still.fusion.bias_est.commits > 0);
		CHECK(wait_for_file(path.c_str()));

		// Destroy saves the final estimate.
		still.run(60, store);
		xreal_air_gyro_bias_store_destroy(&store, &still.fusion, 40.0f);
		CHECK(store == nullptr);
	}

	size_t size = 0;
	char *content = u_file_read_content_from_path(path.c_str(), &size);
	REQUIRE(content != nullptr);
	struct m_imu_3dof_gyro_bias_record saved;
	REQUIRE(m_imu_3dof_gyro_bias_record_parse(content, &params, &saved, nullptr, 0));
	std::free(content);
	CHECK(std::string(saved.device) == "ZBBM5DZFMP");
	CHECK(saved.bias_deg_s.y == Catch::Approx(0.053).margin(0.002));
	CHECK(saved.bias_deg_s.z == Catch::Approx(0.30).margin(0.002));
	CHECK(saved.temp_c == 40.0f);
	CHECK(saved.learned_s > 50.0);
	CHECK(std::abs(saved.saved_unix_s - (int64_t)std::time(nullptr)) < 60);

	{
		struct xreal_air_gyro_bias_store *store =
		    xreal_air_gyro_bias_store_create("ZBBM5DZFMP", &params, U_LOGGING_INFO);
		REQUIRE(store != nullptr);
		struct m_imu_3dof_gyro_bias_record loaded;
		REQUIRE(xreal_air_gyro_bias_store_get_loaded(store, &loaded));
		CHECK(loaded.bias_deg_s.y == saved.bias_deg_s.y);

		Still next(truth);
		xreal_air_gyro_bias_store_seed(store, &next.fusion, 40.0f);
		CHECK(next.fusion.bias_est.seeded);
		CHECK(next.fusion.gyro_bias.value.y == Catch::Approx(rad(saved.bias_deg_s.y)));
		CHECK(next.fusion.bias_est.sd_deg_s == Catch::Approx(params.seed_min_sd_deg_s).margin(1e-3));

		// Learning goes on, and the total learned time accumulates over sessions.
		next.run(60, store);
		REQUIRE(next.fusion.bias_est.commits > 0);
		xreal_air_gyro_bias_store_destroy(&store, &next.fusion, 41.0f);

		content = u_file_read_content_from_path(path.c_str(), &size);
		REQUIRE(content != nullptr);
		struct m_imu_3dof_gyro_bias_record again;
		REQUIRE(m_imu_3dof_gyro_bias_record_parse(content, &params, &again, nullptr, 0));
		std::free(content);
		CHECK(again.learned_s > saved.learned_s);
		CHECK(again.temp_c == 41.0f);
	}
}

TEST_CASE("xreal_air_gyro_bias_store: bad files are ignored")
{
	StateDir state;
	struct m_imu_3dof_gyro_bias_params params;
	m_imu_3dof_gyro_bias_params_default(&params);

	struct m_imu_3dof_gyro_bias_record rec = {};
	std::snprintf(rec.device, sizeof(rec.device), "OTHER");
	rec.bias_deg_s = {0.0f, 0.05f, 0.3f};
	rec.sd_deg_s = 0.01f;
	rec.temp_c = 40.0f;
	rec.saved_unix_s = (int64_t)std::time(nullptr);
	char buf[1024];
	m_imu_3dof_gyro_bias_record_format(&rec, buf, sizeof(buf));

	// Odd characters in the id do not escape the directory.
	struct xreal_air_gyro_bias_store *store = xreal_air_gyro_bias_store_create("../a b", &params, U_LOGGING_INFO);
	REQUIRE(store != nullptr);
	std::string path = xreal_air_gyro_bias_store_path(store);
	CHECK(path == state.dir + "/monado/xreal_air_gyro_bias____a_b.txt");
	xreal_air_gyro_bias_store_destroy(&store, nullptr, 0.0f);

	// Saved for another device.
	write_file(path.c_str(), buf);
	store = xreal_air_gyro_bias_store_create("../a b", &params, U_LOGGING_INFO);
	REQUIRE(store != nullptr);
	CHECK_FALSE(xreal_air_gyro_bias_store_get_loaded(store, &rec));

	// Seeding without a loaded record leaves the fusion alone.
	struct m_imu_3dof fusion;
	m_imu_3dof_init(&fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS | M_IMU_3DOF_AUTO_GYRO_BIAS);
	xreal_air_gyro_bias_store_seed(store, &fusion, 40.0f);
	CHECK_FALSE(fusion.bias_est.seeded);
	CHECK(fusion.gyro_bias.value.y == 0.0f);
	m_imu_3dof_close(&fusion);
	xreal_air_gyro_bias_store_destroy(&store, nullptr, 0.0f);

	// Garbage.
	write_file(path.c_str(), "version=1\nbias_deg_s=1 2 3\n\x01\x02");
	store = xreal_air_gyro_bias_store_create("../a b", &params, U_LOGGING_INFO);
	REQUIRE(store != nullptr);
	CHECK_FALSE(xreal_air_gyro_bias_store_get_loaded(store, &rec));
	xreal_air_gyro_bias_store_destroy(&store, nullptr, 0.0f);

	// No HOME and no XDG_STATE_HOME: no store at all.
	const char *home = std::getenv("HOME");
	std::string home_copy = home != nullptr ? home : "";
	unsetenv("HOME");
	unsetenv("XDG_STATE_HOME");
	CHECK(xreal_air_gyro_bias_store_create("x", &params, U_LOGGING_INFO) == nullptr);
	if (home != nullptr) {
		setenv("HOME", home_copy.c_str(), 1);
	}
}
