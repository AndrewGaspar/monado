// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra stereo frame pairing.
 *
 * The two cameras expose simultaneously (device timestamps ~15 us apart) and their frames
 * arrive one after the other, in either order. The telemetry "pair counter" at offset 18 is NOT
 * a pair key: which camera it increments on varies from stream start to stream start, so a
 * counter value can hold a right frame and the NEXT exposure's left frame (33 ms apart). The
 * per-pair timestamp at offset 62 is the key, cross-checked by device timestamp proximity.
 *
 * Pure bookkeeping, no device access, so it can be unit tested on recorded telemetry.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xreal_air_camera_decode.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Frames kept waiting for their partner, per camera.
#define XREAL_AIR_CAM_PAIRER_DEPTH 4

//! Default maximum device timestamp difference within a pair.
#define XREAL_AIR_CAM_PAIRER_DEFAULT_TOLERANCE_NS (1000 * 1000)

//! Pending frames older than this (device time) than the newest frame are given up on.
#define XREAL_AIR_CAM_PAIRER_EXPIRY_NS (50 * 1000 * 1000)

struct xreal_air_cam_pair_key
{
	enum xreal_air_cam_id cam;
	uint64_t device_ns; //!< Telemetry offset 0.
	uint64_t pair_ts;   //!< Telemetry offset 62.
};

/*!
 * Called with the caller's items: a pair (left, right), or a frame that will never be paired.
 */
typedef void (*xreal_air_cam_pairer_pair_func_t)(void *ctx, void *left, void *right);
typedef void (*xreal_air_cam_pairer_unpaired_func_t)(void *ctx, void *item);

struct xreal_air_cam_pairer
{
	int64_t tolerance_ns;

	xreal_air_cam_pairer_pair_func_t on_pair;
	xreal_air_cam_pairer_unpaired_func_t on_unpaired;
	void *ctx;

	struct
	{
		bool used;
		struct xreal_air_cam_pair_key key;
		void *item;
	} pending[2][XREAL_AIR_CAM_PAIRER_DEPTH];

	//! Frames whose pair_ts matched a partner's but the device times did not, or vice versa.
	uint64_t mismatches;
	uint64_t pairs;
	uint64_t unpaired;
};

void
xreal_air_cam_pairer_init(struct xreal_air_cam_pairer *p,
                          int64_t tolerance_ns,
                          xreal_air_cam_pairer_pair_func_t on_pair,
                          xreal_air_cam_pairer_unpaired_func_t on_unpaired,
                          void *ctx);

/*!
 * Offer a frame. Calls on_pair if it completes a pair, and on_unpaired for any frames that can no
 * longer be paired (expired, evicted, or an unknown camera).
 */
void
xreal_air_cam_pairer_push(struct xreal_air_cam_pairer *p, const struct xreal_air_cam_pair_key *key, void *item);

//! Give up on everything pending (calls on_unpaired for each).
void
xreal_air_cam_pairer_flush(struct xreal_air_cam_pairer *p);

#ifdef __cplusplus
}
#endif
