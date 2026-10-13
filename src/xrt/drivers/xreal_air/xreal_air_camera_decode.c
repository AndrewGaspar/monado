// Copyright 2025, Luka Panio
// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra SLAM camera frame decoding (descramble, telemetry, orientation).
 * @author Luka Panio <lukapanio@gmail.com>
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_camera_decode.h"

#include <string.h>

const uint8_t xreal_air_cam_chunk_map[XREAL_AIR_CAM_CHUNK_COUNT] = {
    119, 54, 21, 0,  108, 22,  51,  63,  93,  99,  67,  7,   32, 112, 52,  43, 14, 35,  75,  116, 64,  71,
    44,  89, 18, 88, 26,  61,  70,  56,  90,  79,  87,  120, 81, 101, 121, 17, 72, 31,  53,  124, 127, 113,
    111, 36, 48, 19, 37,  83,  126, 74,  109, 5,   84,  41,  76, 30,  110, 29, 12, 115, 28,  102, 105, 62,
    103, 20, 3,  68, 49,  77,  117, 125, 106, 60,  69,  98,  9,  16,  78,  47, 40, 2,   118, 34,  13,  50,
    46,  80, 85, 66, 42,  123, 122, 96,  11,  25,  97,  39,  6,  86,  1,   8,  82, 92,  59,  104, 24,  15,
    73,  65, 38, 58, 10,  23,  33,  55,  57,  107, 100, 94,  27, 95,  45,  91, 4,  114,
};

#define PHASE_SUM_LEN 128

static uint64_t
read_le(const uint8_t *p, size_t len)
{
	uint64_t v = 0;
	for (size_t i = 0; i < len; i++) {
		v |= (uint64_t)p[i] << (8 * i);
	}
	return v;
}

bool
xreal_air_cam_parse_telemetry(const uint8_t *raw, size_t size, struct xreal_air_cam_telemetry *out)
{
	if (size < XREAL_AIR_CAM_TELEMETRY_OFFSET + XREAL_AIR_CAM_TELEMETRY_RAW_LEN) {
		return false;
	}

	const uint8_t *t = raw + XREAL_AIR_CAM_TELEMETRY_OFFSET;

	out->device_ns = read_le(t + 0, 8);
	out->counter = (uint16_t)read_le(t + 18, 2);
	out->exposure = (uint32_t)read_le(t + 22, 3);
	out->gain = t[26];
	out->id_byte58 = t[58];
	out->id_byte59 = t[59];
	out->pair_ts = read_le(t + 62, 8);
	memcpy(out->raw, t, XREAL_AIR_CAM_TELEMETRY_RAW_LEN);

	return true;
}

enum xreal_air_cam_id
xreal_air_cam_identify(const struct xreal_air_cam_telemetry *t)
{
	return t->id_byte59 != 0 ? XREAL_AIR_CAM_RIGHT : XREAL_AIR_CAM_LEFT;
}

uint32_t
xreal_air_cam_find_phase(const uint8_t *raw)
{
	uint32_t best_chunk = 0;
	uint32_t best_sum = UINT32_MAX;

	for (uint32_t i = 0; i < XREAL_AIR_CAM_CHUNK_COUNT; i++) {
		const uint8_t *chunk = raw + (size_t)i * XREAL_AIR_CAM_CHUNK_SIZE;
		uint32_t sum = 0;
		for (uint32_t j = 0; j < PHASE_SUM_LEN; j++) {
			sum += chunk[j];
		}

		if (sum < best_sum) {
			best_sum = sum;
			best_chunk = i;
		}
	}

	for (uint32_t i = 0; i < XREAL_AIR_CAM_CHUNK_COUNT; i++) {
		if (xreal_air_cam_chunk_map[i] == best_chunk) {
			return i;
		}
	}

	// The map is a permutation of 0..127, so this is unreachable.
	return 0;
}

uint32_t
xreal_air_cam_descramble(const uint8_t *raw, uint8_t *out_readout)
{
	const uint32_t phase = xreal_air_cam_find_phase(raw);

	// Chunks are 2400 bytes and lines 640, the stream is contiguous so a plain copy of each
	// chunk to its logical position reassembles the readout.
	for (uint32_t t = 0; t < XREAL_AIR_CAM_CHUNK_COUNT; t++) {
		const uint32_t src = xreal_air_cam_chunk_map[(phase + t) % XREAL_AIR_CAM_CHUNK_COUNT];
		memcpy(out_readout + (size_t)t * XREAL_AIR_CAM_CHUNK_SIZE, raw + (size_t)src * XREAL_AIR_CAM_CHUNK_SIZE,
		       XREAL_AIR_CAM_CHUNK_SIZE);
	}

	return phase;
}

/*!
 * Scatter one span of readout line @p r, starting at pixel @p c0, into the upright image.
 */
static inline void
orient_span(
    const uint8_t *src, size_t len, uint32_t r, uint32_t c0, enum xreal_air_cam_id cam, uint8_t *out, size_t stride)
{
	if (cam == XREAL_AIR_CAM_RIGHT) {
		// (x, y) = (479 - r, c)
		const uint32_t x = (XREAL_AIR_CAM_READOUT_LINES - 1) - r;
		uint8_t *dst = out + (size_t)c0 * stride + x;
		for (size_t k = 0; k < len; k++) {
			dst[k * stride] = src[k];
		}
	} else {
		// (x, y) = (r, 639 - c)
		const uint32_t x = r;
		uint8_t *dst = out + (size_t)((XREAL_AIR_CAM_READOUT_LINE - 1) - c0) * stride + x;
		for (size_t k = 0; k < len; k++) {
			*(dst - (ptrdiff_t)(k * stride)) = src[k];
		}
	}
}

void
xreal_air_cam_orient(const uint8_t *readout, enum xreal_air_cam_id cam, uint8_t *out, size_t out_stride)
{
	for (uint32_t r = 0; r < XREAL_AIR_CAM_READOUT_LINES; r++) {
		orient_span(readout + (size_t)r * XREAL_AIR_CAM_READOUT_LINE, XREAL_AIR_CAM_READOUT_LINE, r, 0, cam,
		            out, out_stride);
	}
}

uint32_t
xreal_air_cam_decode(const uint8_t *raw, enum xreal_air_cam_id cam, uint8_t *out, size_t out_stride)
{
	const uint32_t phase = xreal_air_cam_find_phase(raw);

	// Walk the logical byte stream chunk by chunk, splitting at readout line boundaries.
	size_t pos = 0;
	for (uint32_t t = 0; t < XREAL_AIR_CAM_CHUNK_COUNT; t++) {
		const uint32_t src = xreal_air_cam_chunk_map[(phase + t) % XREAL_AIR_CAM_CHUNK_COUNT];
		const uint8_t *chunk = raw + (size_t)src * XREAL_AIR_CAM_CHUNK_SIZE;

		size_t off = 0;
		while (off < XREAL_AIR_CAM_CHUNK_SIZE) {
			const uint32_t r = (uint32_t)(pos / XREAL_AIR_CAM_READOUT_LINE);
			const uint32_t c = (uint32_t)(pos % XREAL_AIR_CAM_READOUT_LINE);
			size_t len = XREAL_AIR_CAM_READOUT_LINE - c;
			if (len > XREAL_AIR_CAM_CHUNK_SIZE - off) {
				len = XREAL_AIR_CAM_CHUNK_SIZE - off;
			}

			orient_span(chunk + off, len, r, c, cam, out, out_stride);
			off += len;
			pos += len;
		}
	}

	return phase;
}
