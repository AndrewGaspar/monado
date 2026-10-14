// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra camera descramble / orientation / telemetry tests.
 * @author Andrew Gaspar
 */

#include "xreal_air/xreal_air_camera_decode.h"

#include "catch_amalgamated.hpp"

#include <cstring>
#include <random>
#include <vector>

/*!
 * A synthetic readout: random bright pixels everywhere except the first readout line, which is
 * black like the fisheye vignette the phase detection keys on.
 */
static std::vector<uint8_t>
make_readout(std::mt19937 &rng)
{
	std::uniform_int_distribution<int> dist(16, 255);
	std::vector<uint8_t> readout(XREAL_AIR_CAM_IMAGE_SIZE);
	for (auto &px : readout) {
		px = (uint8_t)dist(rng);
	}
	std::memset(readout.data(), 0, XREAL_AIR_CAM_READOUT_LINE);
	return readout;
}

//! Scramble the way the device does: logical chunk t lives in raw chunk map[(phase + t) % 128].
static std::vector<uint8_t>
scramble(const std::vector<uint8_t> &readout, uint32_t phase, const uint8_t telemetry[XREAL_AIR_CAM_TELEMETRY_RAW_LEN])
{
	std::vector<uint8_t> raw(XREAL_AIR_CAM_RAW_SIZE, 0x5c);
	for (uint32_t t = 0; t < XREAL_AIR_CAM_CHUNK_COUNT; t++) {
		const uint32_t dst = xreal_air_cam_chunk_map[(phase + t) % XREAL_AIR_CAM_CHUNK_COUNT];
		std::memcpy(raw.data() + dst * XREAL_AIR_CAM_CHUNK_SIZE, readout.data() + t * XREAL_AIR_CAM_CHUNK_SIZE,
		            XREAL_AIR_CAM_CHUNK_SIZE);
	}
	std::memcpy(raw.data() + XREAL_AIR_CAM_TELEMETRY_OFFSET, telemetry, XREAL_AIR_CAM_TELEMETRY_RAW_LEN);
	return raw;
}

TEST_CASE("xreal_air_cam_chunk_map is a permutation")
{
	std::vector<bool> seen(XREAL_AIR_CAM_CHUNK_COUNT, false);
	for (uint32_t i = 0; i < XREAL_AIR_CAM_CHUNK_COUNT; i++) {
		const uint8_t v = xreal_air_cam_chunk_map[i];
		REQUIRE(v < XREAL_AIR_CAM_CHUNK_COUNT);
		CHECK_FALSE(seen[v]);
		seen[v] = true;
	}
}

TEST_CASE("xreal_air_cam_descramble recovers the readout at any phase")
{
	std::mt19937 rng(0x5eed);
	uint8_t telemetry[XREAL_AIR_CAM_TELEMETRY_RAW_LEN] = {};

	for (uint32_t phase : {0u, 1u, 63u, 64u, 126u, 127u}) {
		CAPTURE(phase);
		const auto readout = make_readout(rng);
		const auto raw = scramble(readout, phase, telemetry);

		std::vector<uint8_t> out(XREAL_AIR_CAM_IMAGE_SIZE, 0);
		CHECK(xreal_air_cam_descramble(raw.data(), out.data()) == phase);
		CHECK((out == readout)); // parenthesised: Catch2 must not stringify a 300 KB frame
	}

	std::uniform_int_distribution<uint32_t> phase_dist(0, XREAL_AIR_CAM_CHUNK_COUNT - 1);
	for (int i = 0; i < 16; i++) {
		const uint32_t phase = phase_dist(rng);
		CAPTURE(phase);
		const auto readout = make_readout(rng);
		const auto raw = scramble(readout, phase, telemetry);

		std::vector<uint8_t> out(XREAL_AIR_CAM_IMAGE_SIZE, 0);
		CHECK(xreal_air_cam_descramble(raw.data(), out.data()) == phase);
		CHECK((out == readout));
	}
}

TEST_CASE("xreal_air_cam_orient maps readout lines to image columns")
{
	std::mt19937 rng(42);
	const auto readout = make_readout(rng);
	const size_t stride = XREAL_AIR_CAM_WIDTH;

	std::vector<uint8_t> right(XREAL_AIR_CAM_WIDTH * XREAL_AIR_CAM_HEIGHT, 0);
	std::vector<uint8_t> left(XREAL_AIR_CAM_WIDTH * XREAL_AIR_CAM_HEIGHT, 0);
	xreal_air_cam_orient(readout.data(), XREAL_AIR_CAM_RIGHT, right.data(), stride);
	xreal_air_cam_orient(readout.data(), XREAL_AIR_CAM_LEFT, left.data(), stride);

	for (uint32_t r = 0; r < XREAL_AIR_CAM_READOUT_LINES; r += 7) {
		for (uint32_t c = 0; c < XREAL_AIR_CAM_READOUT_LINE; c += 5) {
			const uint8_t px = readout[r * XREAL_AIR_CAM_READOUT_LINE + c];
			// Right: (x, y) = (479 - r, c).
			CHECK(right[c * stride + (479 - r)] == px);
			// Left: (x, y) = (r, 639 - c), i.e. the right mapping rotated 180 degrees.
			CHECK(left[(639 - c) * stride + r] == px);
		}
	}
}

TEST_CASE("xreal_air_cam_decode equals descramble + orient, with padded stride")
{
	std::mt19937 rng(7);
	uint8_t telemetry[XREAL_AIR_CAM_TELEMETRY_RAW_LEN] = {};
	const auto readout = make_readout(rng);
	const uint32_t phase = 91;
	const auto raw = scramble(readout, phase, telemetry);

	const size_t stride = XREAL_AIR_CAM_WIDTH + 32;
	for (auto cam : {XREAL_AIR_CAM_LEFT, XREAL_AIR_CAM_RIGHT}) {
		CAPTURE(cam);
		std::vector<uint8_t> expected(stride * XREAL_AIR_CAM_HEIGHT, 0xee);
		std::vector<uint8_t> actual(stride * XREAL_AIR_CAM_HEIGHT, 0xee);

		xreal_air_cam_orient(readout.data(), cam, expected.data(), stride);
		CHECK(xreal_air_cam_decode(raw.data(), cam, actual.data(), stride) == phase);
		CHECK((actual == expected));

		// The stride padding is untouched.
		for (size_t y = 0; y < XREAL_AIR_CAM_HEIGHT; y += 50) {
			CHECK(actual[y * stride + XREAL_AIR_CAM_WIDTH] == 0xee);
		}
	}
}

TEST_CASE("xreal_air_cam telemetry parse and identify")
{
	uint8_t telemetry[XREAL_AIR_CAM_TELEMETRY_RAW_LEN] = {};
	const uint64_t device_ns = 4310123456789ull;
	for (int i = 0; i < 8; i++) {
		telemetry[i] = (uint8_t)(device_ns >> (8 * i));
	}
	telemetry[18] = 0x34;
	telemetry[19] = 0x12;
	telemetry[22] = 0xb4; // 7992500 = 0x79f4b4
	telemetry[23] = 0xf4;
	telemetry[24] = 0x79;
	telemetry[26] = 100;
	telemetry[62] = 0x01;
	telemetry[63] = 0x02;

	std::vector<uint8_t> raw(XREAL_AIR_CAM_RAW_SIZE, 0);
	std::memcpy(raw.data() + XREAL_AIR_CAM_TELEMETRY_OFFSET, telemetry, sizeof(telemetry));

	struct xreal_air_cam_telemetry t;
	REQUIRE(xreal_air_cam_parse_telemetry(raw.data(), raw.size(), &t));
	CHECK(t.device_ns == device_ns);
	CHECK(t.counter == 0x1234);
	CHECK(t.exposure == 7992500);
	CHECK(t.gain == 100);
	CHECK(t.pair_ts == 0x0201);
	CHECK(std::memcmp(t.raw, telemetry, sizeof(telemetry)) == 0);

	CHECK_FALSE(xreal_air_cam_parse_telemetry(raw.data(), XREAL_AIR_CAM_TELEMETRY_OFFSET + 10, &t));

	// Byte 59 (0x3b) decides, byte 58 (always 0 live) is ignored.
	t.id_byte58 = 0x21;
	t.id_byte59 = 1;
	CHECK(xreal_air_cam_identify(&t) == XREAL_AIR_CAM_RIGHT);
	t.id_byte58 = 0x20;
	t.id_byte59 = 0;
	CHECK(xreal_air_cam_identify(&t) == XREAL_AIR_CAM_LEFT);
	t.id_byte58 = 0;
	t.id_byte59 = 1;
	CHECK(xreal_air_cam_identify(&t) == XREAL_AIR_CAM_RIGHT);
	t.id_byte59 = 0;
	CHECK(xreal_air_cam_identify(&t) == XREAL_AIR_CAM_LEFT);
}
