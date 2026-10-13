// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra stereo frame pairing.
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_camera_pairing.h"

#include <string.h>

static int64_t
abs_diff(uint64_t a, uint64_t b)
{
	return a > b ? (int64_t)(a - b) : (int64_t)(b - a);
}

static void
give_up(struct xreal_air_cam_pairer *p, int cam, int slot)
{
	p->pending[cam][slot].used = false;
	p->unpaired++;
	p->on_unpaired(p->ctx, p->pending[cam][slot].item);
}

void
xreal_air_cam_pairer_init(struct xreal_air_cam_pairer *p,
                          int64_t tolerance_ns,
                          xreal_air_cam_pairer_pair_func_t on_pair,
                          xreal_air_cam_pairer_unpaired_func_t on_unpaired,
                          void *ctx)
{
	memset(p, 0, sizeof(*p));
	p->tolerance_ns = tolerance_ns;
	p->on_pair = on_pair;
	p->on_unpaired = on_unpaired;
	p->ctx = ctx;
}

void
xreal_air_cam_pairer_push(struct xreal_air_cam_pairer *p, const struct xreal_air_cam_pair_key *key, void *item)
{
	if (key->cam != XREAL_AIR_CAM_LEFT && key->cam != XREAL_AIR_CAM_RIGHT) {
		p->unpaired++;
		p->on_unpaired(p->ctx, item);
		return;
	}

	const int self = (int)key->cam;
	const int other = 1 - self;

	// Give up on frames this new one has left behind.
	for (int c = 0; c < 2; c++) {
		for (int s = 0; s < XREAL_AIR_CAM_PAIRER_DEPTH; s++) {
			if (p->pending[c][s].used &&
			    p->pending[c][s].key.device_ns + XREAL_AIR_CAM_PAIRER_EXPIRY_NS < key->device_ns) {
				give_up(p, c, s);
			}
		}
	}

	// Look for the partner: same pair timestamp AND (nearly) the same exposure time.
	for (int s = 0; s < XREAL_AIR_CAM_PAIRER_DEPTH; s++) {
		if (!p->pending[other][s].used) {
			continue;
		}

		const struct xreal_air_cam_pair_key *o = &p->pending[other][s].key;
		const bool same_pair_ts = o->pair_ts == key->pair_ts;
		const bool close = abs_diff(o->device_ns, key->device_ns) <= p->tolerance_ns;

		if (same_pair_ts && close) {
			void *partner = p->pending[other][s].item;
			p->pending[other][s].used = false;
			p->pairs++;
			if (self == XREAL_AIR_CAM_LEFT) {
				p->on_pair(p->ctx, item, partner);
			} else {
				p->on_pair(p->ctx, partner, item);
			}
			return;
		}

		if (same_pair_ts != close) {
			p->mismatches++;
		}
	}

	// Wait for the partner, evicting the oldest pending frame of this camera if full.
	int slot = -1;
	for (int s = 0; s < XREAL_AIR_CAM_PAIRER_DEPTH; s++) {
		if (!p->pending[self][s].used) {
			slot = s;
			break;
		}
	}
	if (slot < 0) {
		slot = 0;
		for (int s = 1; s < XREAL_AIR_CAM_PAIRER_DEPTH; s++) {
			if (p->pending[self][s].key.device_ns < p->pending[self][slot].key.device_ns) {
				slot = s;
			}
		}
		give_up(p, self, slot);
	}

	p->pending[self][slot].used = true;
	p->pending[self][slot].key = *key;
	p->pending[self][slot].item = item;
}

void
xreal_air_cam_pairer_flush(struct xreal_air_cam_pairer *p)
{
	for (int c = 0; c < 2; c++) {
		for (int s = 0; s < XREAL_AIR_CAM_PAIRER_DEPTH; s++) {
			if (p->pending[c][s].used) {
				give_up(p, c, s);
			}
		}
	}
}
