// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Fixed size overwriting record ring for the flight recorder.
 *
 * Index validated, like a seqlock with the ring index as the sequence: the producer stores the slot
 * words after a release fence and then publishes head = index + 1, the reader copies the slot words
 * and re-reads head after an acquire fence. If the producer had started on the record that reuses the
 * slot (index + capacity), the reader sees head >= index + capacity and drops the copy. Slot words are
 * relaxed atomics, so a concurrent overwrite is a well defined (if useless) read, not a data race.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_flight_ring.h"

#include "os/os_threading.h"

#include <stdlib.h>
#include <string.h>

struct xreal_air_flight_ring
{
	uint64_t *words;
	size_t record_size;
	size_t slot_words; //!< record_size rounded up to whole 64 bit words.
	uint64_t capacity; //!< Power of two.
	uint64_t mask;
	uint64_t head; //!< Atomic: the index of the next push.
	struct os_mutex producer;
};

struct xreal_air_flight_ring *
xreal_air_flight_ring_create(size_t record_size, uint64_t min_records)
{
	if (record_size == 0 || min_records == 0 || min_records > (1ull << 32)) {
		return NULL;
	}

	uint64_t capacity = 1;
	while (capacity < min_records) {
		capacity <<= 1;
	}

	struct xreal_air_flight_ring *r = calloc(1, sizeof(*r));
	if (r == NULL) {
		return NULL;
	}
	r->record_size = record_size;
	r->slot_words = (record_size + sizeof(uint64_t) - 1) / sizeof(uint64_t);
	r->capacity = capacity;
	r->mask = capacity - 1;
	r->words = calloc(capacity * r->slot_words, sizeof(uint64_t));
	if (r->words == NULL || os_mutex_init(&r->producer) != 0) {
		free(r->words);
		free(r);
		return NULL;
	}
	return r;
}

void
xreal_air_flight_ring_destroy(struct xreal_air_flight_ring **ring_ptr)
{
	struct xreal_air_flight_ring *r = *ring_ptr;
	if (r == NULL) {
		return;
	}
	os_mutex_destroy(&r->producer);
	free(r->words);
	free(r);
	*ring_ptr = NULL;
}

uint64_t
xreal_air_flight_ring_capacity(const struct xreal_air_flight_ring *ring)
{
	return ring->capacity;
}

size_t
xreal_air_flight_ring_bytes(const struct xreal_air_flight_ring *ring)
{
	return (size_t)ring->capacity * ring->slot_words * sizeof(uint64_t);
}

void
xreal_air_flight_ring_push(struct xreal_air_flight_ring *ring, const void *record)
{
	os_mutex_lock(&ring->producer);

	const uint64_t index = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
	uint64_t *slot = ring->words + (index & ring->mask) * ring->slot_words;

	// Orders the publication of the previous records before the overwrite below (see the file comment).
	__atomic_thread_fence(__ATOMIC_RELEASE);

	const uint8_t *src = (const uint8_t *)record;
	size_t left = ring->record_size;
	for (size_t w = 0; w < ring->slot_words; w++) {
		uint64_t v = 0;
		const size_t n = left < sizeof(v) ? left : sizeof(v);
		memcpy(&v, src + w * sizeof(v), n);
		left -= n;
		__atomic_store_n(&slot[w], v, __ATOMIC_RELAXED);
	}

	__atomic_store_n(&ring->head, index + 1, __ATOMIC_RELEASE);

	os_mutex_unlock(&ring->producer);
}

uint64_t
xreal_air_flight_ring_head(const struct xreal_air_flight_ring *ring)
{
	return __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
}

uint64_t
xreal_air_flight_ring_oldest(const struct xreal_air_flight_ring *ring, uint64_t slack)
{
	const uint64_t head = xreal_air_flight_ring_head(ring);
	const uint64_t keep = slack < ring->capacity ? ring->capacity - slack : 0;
	return head > keep ? head - keep : 0;
}

enum xreal_air_flight_ring_read
xreal_air_flight_ring_read(const struct xreal_air_flight_ring *ring, uint64_t index, void *out)
{
	const uint64_t head = __atomic_load_n(&ring->head, __ATOMIC_ACQUIRE);
	if (index >= head) {
		return XREAL_AIR_FLIGHT_RING_NOT_YET;
	}
	// At head - index == capacity the producer may be writing this slot right now.
	if (head - index >= ring->capacity) {
		return XREAL_AIR_FLIGHT_RING_OVERWRITTEN;
	}

	const uint64_t *slot = ring->words + (index & ring->mask) * ring->slot_words;
	uint8_t *dst = (uint8_t *)out;
	size_t left = ring->record_size;
	for (size_t w = 0; w < ring->slot_words; w++) {
		const uint64_t v = __atomic_load_n(&slot[w], __ATOMIC_RELAXED);
		const size_t n = left < sizeof(v) ? left : sizeof(v);
		memcpy(dst + w * sizeof(v), &v, n);
		left -= n;
	}

	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	const uint64_t head_after = __atomic_load_n(&ring->head, __ATOMIC_RELAXED);
	if (head_after - index >= ring->capacity) {
		return XREAL_AIR_FLIGHT_RING_OVERWRITTEN;
	}
	return XREAL_AIR_FLIGHT_RING_OK;
}
