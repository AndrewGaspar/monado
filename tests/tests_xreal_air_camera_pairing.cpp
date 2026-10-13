// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra stereo frame pairing tests, on recorded telemetry.
 * @author Andrew Gaspar
 */

#include "xreal_air/xreal_air_camera_pairing.h"

#include "catch_amalgamated.hpp"

#include "tests_xreal_air_camera_pairing_data.h"

#include <cstdlib>
#include <vector>

struct result
{
	std::vector<std::pair<const pairing_frame *, const pairing_frame *>> pairs;
	std::vector<const pairing_frame *> unpaired;
};

static void
on_pair(void *ctx, void *left, void *right)
{
	static_cast<result *>(ctx)->pairs.emplace_back(static_cast<const pairing_frame *>(left),
	                                               static_cast<const pairing_frame *>(right));
}

static void
on_unpaired(void *ctx, void *item)
{
	static_cast<result *>(ctx)->unpaired.push_back(static_cast<const pairing_frame *>(item));
}

template <size_t N>
static result
run(const pairing_frame (&frames)[N], int64_t tolerance_ns = XREAL_AIR_CAM_PAIRER_DEFAULT_TOLERANCE_NS)
{
	result res;
	xreal_air_cam_pairer p;
	xreal_air_cam_pairer_init(&p, tolerance_ns, on_pair, on_unpaired, &res);

	for (const auto &f : frames) {
		const xreal_air_cam_pair_key key = {
		    (xreal_air_cam_id)f.cam,
		    f.device_ns,
		    f.pair_ts,
		};
		xreal_air_cam_pairer_push(&p, &key, (void *)&f);
	}
	xreal_air_cam_pairer_flush(&p);

	CHECK(p.mismatches == 0);
	CHECK(2 * res.pairs.size() + res.unpaired.size() == N);
	return res;
}

static void
check_true_pairs(const result &res)
{
	for (const auto &pr : res.pairs) {
		CAPTURE(pr.first->device_ns, pr.second->device_ns, pr.first->counter, pr.second->counter);
		CHECK(pr.first->cam == XREAL_AIR_CAM_LEFT);
		CHECK(pr.second->cam == XREAL_AIR_CAM_RIGHT);
		CHECK(pr.first->pair_ts == pr.second->pair_ts);
		CHECK(std::llabs((long long)(pr.first->device_ns - pr.second->device_ns)) < 100000);
	}
}

TEST_CASE("pairer: counter shifted by one exposure (counter increments on the right camera)")
{
	// The data really is shifted: the two frames sharing a counter are an exposure apart.
	CHECK(shifted_frames[2].counter == shifted_frames[3].counter);
	CHECK(std::llabs((long long)(shifted_frames[2].device_ns - shifted_frames[3].device_ns)) > 30000000);

	const result res = run(shifted_frames);
	CHECK(res.pairs.size() == (size_t)shifted_expected_pairs);
	check_true_pairs(res);

	// So the true partners carry different counters.
	size_t cross_counter = 0;
	for (const auto &pr : res.pairs) {
		cross_counter += pr.first->counter != pr.second->counter;
	}
	CHECK(cross_counter > res.pairs.size() / 2);
}

TEST_CASE("pairer: counter aligned with pairs, either arrival order")
{
	const result res = run(aligned_frames);
	CHECK(res.pairs.size() == (size_t)aligned_expected_pairs);
	check_true_pairs(res);
	for (const auto &pr : res.pairs) {
		CHECK(pr.first->counter == pr.second->counter);
	}
}

TEST_CASE("pairer: pair_ts and exposure time disagreeing is not a pair")
{
	const pairing_frame frames[] = {
	    {0, 1000000000, 5000, 1},
	    {1, 1000010000, 6000, 1}, // Same exposure, different pair_ts.
	    {0, 1033000000, 7000, 2},
	    {1, 1066000000, 7000, 3}, // Same pair_ts, an exposure apart.
	};
	result res;
	xreal_air_cam_pairer p;
	xreal_air_cam_pairer_init(&p, XREAL_AIR_CAM_PAIRER_DEFAULT_TOLERANCE_NS, on_pair, on_unpaired, &res);
	for (const auto &f : frames) {
		const xreal_air_cam_pair_key key = {(xreal_air_cam_id)f.cam, f.device_ns, f.pair_ts};
		xreal_air_cam_pairer_push(&p, &key, (void *)&f);
	}
	xreal_air_cam_pairer_flush(&p);

	CHECK(res.pairs.empty());
	CHECK(res.unpaired.size() == 4);
	CHECK(p.mismatches == 2);
}
