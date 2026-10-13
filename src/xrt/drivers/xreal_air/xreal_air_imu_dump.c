// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Non-blocking CSV dump of the Xreal Air IMU stream, for offline fusion tuning.
 * @ingroup drv_xreal_air
 *
 * The sensor thread runs at ~1 kHz and must never wait on the disk, so records
 * go into a fixed single-producer/single-consumer ring and a writer thread
 * formats and writes them a few times a second.
 */

#include "os/os_threading.h"
#include "os/os_time.h"

#include "util/u_misc.h"
#include "util/u_logging.h"

#include "xreal_air_imu_dump.h"

#include <inttypes.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>


//! ~16 s of samples at 1 kHz.
#define DUMP_RING_SIZE (1u << 14)
#define DUMP_RING_MASK (DUMP_RING_SIZE - 1u)

#define DUMP_DRAIN_PERIOD_NS (100 * U_TIME_1MS_IN_NS)

struct xreal_air_imu_dump
{
	FILE *file;
	char path[1024];

	struct os_thread_helper oth;

	//! Written by the producer only.
	_Atomic uint64_t head;
	//! Written by the writer thread only.
	_Atomic uint64_t tail;
	//! Records dropped because the ring was full.
	_Atomic uint64_t dropped;
	uint64_t dropped_reported;

	struct xreal_air_imu_dump_record ring[DUMP_RING_SIZE];
};

static void
write_record(FILE *file, const struct xreal_air_imu_dump_record *r)
{
	fprintf(file,
	        "%" PRIu64 ",%" PRId64 ",%" PRId64
	        ",%.3f,"
	        "%.9g,%.9g,%.9g,"
	        "%.9g,%.9g,%.9g,"
	        "%.6g,%.6g,%.6g,"
	        "%.9g,%.9g,%.9g,"
	        "%.9g,%.9g,%.9g,%.9g,"
	        "%d\n",
	        r->device_ns, r->fusion_ns, r->host_ns, (double)r->temperature_c,       //
	        (double)r->gyro.x, (double)r->gyro.y, (double)r->gyro.z,                //
	        (double)r->accel.x, (double)r->accel.y, (double)r->accel.z,             //
	        (double)r->mag.x, (double)r->mag.y, (double)r->mag.z,                   //
	        (double)r->bias.x, (double)r->bias.y, (double)r->bias.z,                //
	        (double)r->rot.x, (double)r->rot.y, (double)r->rot.z, (double)r->rot.w, //
	        r->still ? 1 : 0);
}

static void
drain(struct xreal_air_imu_dump *d)
{
	uint64_t head = atomic_load_explicit(&d->head, memory_order_acquire);
	uint64_t tail = atomic_load_explicit(&d->tail, memory_order_relaxed);

	uint64_t dropped = atomic_load_explicit(&d->dropped, memory_order_relaxed);
	if (dropped != d->dropped_reported) {
		fprintf(d->file, "# dropped %" PRIu64 " records (total)\n", dropped);
		d->dropped_reported = dropped;
	}

	while (tail != head) {
		write_record(d->file, &d->ring[tail & DUMP_RING_MASK]);
		tail++;
		// Hand the slot back right away so the producer has room while we format.
		atomic_store_explicit(&d->tail, tail, memory_order_release);
	}

	fflush(d->file);
}

static void *
writer_thread(void *ptr)
{
	struct xreal_air_imu_dump *d = (struct xreal_air_imu_dump *)ptr;

	os_thread_helper_name(&d->oth, "Xreal Air IMU dump");

	os_thread_helper_lock(&d->oth);
	while (os_thread_helper_is_running_locked(&d->oth)) {
		os_thread_helper_unlock(&d->oth);

		drain(d);
		os_nanosleep(DUMP_DRAIN_PERIOD_NS);

		os_thread_helper_lock(&d->oth);
	}
	os_thread_helper_unlock(&d->oth);

	drain(d);

	return NULL;
}

struct xreal_air_imu_dump *
xreal_air_imu_dump_create(const char *path, const char *header)
{
	struct xreal_air_imu_dump *d = U_TYPED_CALLOC(struct xreal_air_imu_dump);
	if (d == NULL) {
		return NULL;
	}

	struct stat st;
	if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
		time_t now = time(NULL);
		struct tm tm;
		localtime_r(&now, &tm);
		char stamp[32];
		strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);
		snprintf(d->path, sizeof(d->path), "%s/xreal-imu-%s.csv", path, stamp);
	} else {
		snprintf(d->path, sizeof(d->path), "%s", path);
	}

	d->file = fopen(d->path, "w");
	if (d->file == NULL) {
		U_LOG_E("Could not open IMU dump file '%s'", d->path);
		free(d);
		return NULL;
	}

	// Big stdio buffer: the writer thread does few, large writes.
	setvbuf(d->file, NULL, _IOFBF, 1 << 20);

	fprintf(d->file, "# xreal_air imu dump v1: %s\n", header != NULL ? header : "");
	fprintf(d->file,
	        "device_ns,fusion_ns,host_ns,temp_c,"
	        "gx,gy,gz,ax,ay,az,mx,my,mz,bx,by,bz,"
	        "qx,qy,qz,qw,still\n");
	fflush(d->file);

	if (os_thread_helper_init(&d->oth) != 0) {
		fclose(d->file);
		free(d);
		return NULL;
	}

	if (os_thread_helper_start(&d->oth, writer_thread, d) != 0) {
		os_thread_helper_destroy(&d->oth);
		fclose(d->file);
		free(d);
		return NULL;
	}

	return d;
}

void
xreal_air_imu_dump_push(struct xreal_air_imu_dump *d, const struct xreal_air_imu_dump_record *r)
{
	uint64_t head = atomic_load_explicit(&d->head, memory_order_relaxed);
	uint64_t tail = atomic_load_explicit(&d->tail, memory_order_acquire);

	if (head - tail >= DUMP_RING_SIZE) {
		atomic_fetch_add_explicit(&d->dropped, 1, memory_order_relaxed);
		return;
	}

	d->ring[head & DUMP_RING_MASK] = *r;
	atomic_store_explicit(&d->head, head + 1, memory_order_release);
}

const char *
xreal_air_imu_dump_path(const struct xreal_air_imu_dump *d)
{
	return d->path;
}

void
xreal_air_imu_dump_destroy(struct xreal_air_imu_dump **d_ptr)
{
	struct xreal_air_imu_dump *d = *d_ptr;
	if (d == NULL) {
		return;
	}
	*d_ptr = NULL;

	// Stops the writer, which drains what is left on the way out.
	os_thread_helper_stop_and_wait(&d->oth);
	os_thread_helper_destroy(&d->oth);

	uint64_t dropped = atomic_load(&d->dropped);
	if (dropped > 0) {
		U_LOG_W("IMU dump '%s' dropped %" PRIu64 " records", d->path, dropped);
	}

	fclose(d->file);
	free(d);
}
