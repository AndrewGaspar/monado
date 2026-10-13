// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Windowed minimum skew clock tracker tests.
 * @author Andrew Gaspar
 */

#include "math/m_clock_tracking.h"

#include "catch_amalgamated.hpp"

#include "tests_clock_tracking_data.h"

#include <cstdint>

static void
check_constant_offset(int64_t offset_ns)
{
	auto *t = m_clock_windowed_skew_tracker_alloc(64);
	REQUIRE(t != nullptr);

	const int64_t remote0 = 1726142498364;
	for (int64_t i = 0; i < 500; i++) {
		const int64_t remote = remote0 + i * 16666667;
		// A little transport jitter on top of the true offset, never earlier than it.
		const int64_t jitter = (i * 7919) % 3000000;
		m_clock_windowed_skew_tracker_push(t, remote + offset_ns + jitter, remote);

		timepoint_ns local = 0;
		REQUIRE(m_clock_windowed_skew_tracker_to_local(t, remote, &local));
		CHECK(local >= remote + offset_ns);
		CHECK(local <= remote + offset_ns + 3000000);
	}

	m_clock_windowed_skew_tracker_destroy(t);
}

TEST_CASE("m_clock_windowed_skew_tracker constant offsets")
{
	SECTION("local ahead of remote (positive skew)")
	{
		check_constant_offset(88000000000ll);
	}
	SECTION("remote ahead of local (negative skew)")
	{
		check_constant_offset(-18000000000ll);
	}
}

/*!
 * Replay real observations: the mapped remote time must land just before the local arrival
 * time (by at most @p max_latency_ns), never after it.
 */
template <size_t N>
static void
replay(const clock_obs (&obs)[N], size_t window, int64_t max_latency_ns)
{
	auto *t = m_clock_windowed_skew_tracker_alloc(window);
	REQUIRE(t != nullptr);

	for (size_t i = 0; i < N; i++) {
		m_clock_windowed_skew_tracker_push(t, obs[i].local_ns, obs[i].remote_ns);

		timepoint_ns local = 0;
		REQUIRE(m_clock_windowed_skew_tracker_to_local(t, obs[i].remote_ns, &local));
		CAPTURE(i, obs[i].local_ns, obs[i].remote_ns, local);
		// The tracker smooths towards the minimum delay, the first few can land slightly late.
		CHECK(local <= obs[i].local_ns + (i < 16 ? 5000000 : 1000000));
		CHECK(obs[i].local_ns - local <= max_latency_ns);
	}

	m_clock_windowed_skew_tracker_destroy(t);
}

TEST_CASE("m_clock_windowed_skew_tracker real XREAL camera clock")
{
	replay(camera_obs, 64, 50000000);
}

TEST_CASE("m_clock_windowed_skew_tracker real XREAL IMU clock")
{
	replay(imu_obs, 512, 20000000);
}
