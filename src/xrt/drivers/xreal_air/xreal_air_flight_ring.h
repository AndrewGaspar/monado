// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Fixed size overwriting record ring for the flight recorder (xreal_air_flight.h).
 *
 * Always the newest records: a push never waits for the reader and never fails, it overwrites the
 * oldest record once the ring is full. Producers are serialized by a mutex of their own (pushes from
 * several threads are fine, a push only ever waits for another push). The one reader copies records
 * out by index and is told when a record was overwritten before or while it read it, so it never
 * hands out a torn record and knows exactly how many it missed.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct xreal_air_flight_ring;

enum xreal_air_flight_ring_read
{
	XREAL_AIR_FLIGHT_RING_OK,          //!< Copied.
	XREAL_AIR_FLIGHT_RING_NOT_YET,     //!< Not pushed yet.
	XREAL_AIR_FLIGHT_RING_OVERWRITTEN, //!< Gone, the producer has wrapped past it.
};

/*!
 * A ring of at least @p min_records records of @p record_size bytes (rounded up to a power of two).
 *
 * @return NULL on allocation failure or @p min_records == 0.
 */
struct xreal_air_flight_ring *
xreal_air_flight_ring_create(size_t record_size, uint64_t min_records);

void
xreal_air_flight_ring_destroy(struct xreal_air_flight_ring **ring_ptr);

//! How many records it holds.
uint64_t
xreal_air_flight_ring_capacity(const struct xreal_air_flight_ring *ring);

//! Bytes allocated for the records.
size_t
xreal_air_flight_ring_bytes(const struct xreal_air_flight_ring *ring);

//! Copy @p record in as the newest, overwriting the oldest when full. Never waits for the reader.
void
xreal_air_flight_ring_push(struct xreal_air_flight_ring *ring, const void *record);

//! The index the next push gets: records [head - capacity, head) may still be readable.
uint64_t
xreal_air_flight_ring_head(const struct xreal_air_flight_ring *ring);

/*!
 * The oldest index worth starting to read from: the producer can push @p slack more records before
 * it reaches it (gives the reader time to catch up before the wrap).
 */
uint64_t
xreal_air_flight_ring_oldest(const struct xreal_air_flight_ring *ring, uint64_t slack);

//! Copy record @p index out to @p out (record_size bytes). Single reader.
enum xreal_air_flight_ring_read
xreal_air_flight_ring_read(const struct xreal_air_flight_ring *ring, uint64_t index, void *out);

#ifdef __cplusplus
}
#endif
