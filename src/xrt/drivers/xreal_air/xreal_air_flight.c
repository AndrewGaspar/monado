// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
#ifndef _GNU_SOURCE
#define _GNU_SOURCE // syscall(SYS_gettid)
#endif
/*!
 * @file
 * @brief  Xreal Air 2 Ultra 6DoF flight recorder: on-demand diagnostic captures with look-back.
 *
 * Threads: the producers (sensor, camera and pose query threads) only ever copy into the rings. One
 * writer thread does everything else: it watches the control directory, and while a capture runs
 * it reads the rings behind the producers, formats and writes. It runs at nice 10, so on a starved
 * machine it is the one that falls behind, and the rings are what it falls behind into.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_flight.h"
#include "xreal_air_flight_ring.h"
#include "xreal_air_slam_guard.h"

#include "os/os_threading.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_git_tag.h"
#include "util/u_misc.h"
#include "xrt/xrt_frame.h"

#ifdef XREAL_AIR_HAVE_ZSTD
#include <zstd.h>
#endif

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/inotify.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

DEBUG_GET_ONCE_BOOL_OPTION(xreal_air_flight, "XREAL_AIR_FLIGHT", true)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_flight_lookback_s, "XREAL_AIR_FLIGHT_LOOKBACK_S", 60)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_flight_frames_s, "XREAL_AIR_FLIGHT_FRAMES_S", 0)
DEBUG_GET_ONCE_OPTION(xreal_air_flight_ctl, "XREAL_AIR_FLIGHT_CTL", NULL)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_flight_min_free_mb, "XREAL_AIR_FLIGHT_MIN_FREE_MB", 2048)
DEBUG_GET_ONCE_NUM_OPTION(xreal_air_flight_max_mb, "XREAL_AIR_FLIGHT_MAX_MB", 8192)

#define FL_INFO(f, ...) U_LOG_IFL_I((f)->cfg.log_level, "Xreal Air flight recorder: " __VA_ARGS__)
// Lifecycle lines, at WARN so they show at the driver's default log level.
#define FL_NOTE(f, ...) U_LOG_IFL_W((f)->cfg.log_level, "Xreal Air flight recorder: " __VA_ARGS__)
#define FL_ERROR(f, ...) U_LOG_IFL_E((f)->cfg.log_level, "Xreal Air flight recorder: " __VA_ARGS__)

//! Ring sizes per second of look-back, with headroom over the nominal rates.
#define IMU_PER_S 1100    // 1 kHz
#define CAM_ROWS_PER_S 72 // 2 x ~28 Hz, plus unpaired frames
#define POSES_PER_S 40    // ~28 Hz
#define HEADS_PER_S 1024  // The compositor and every client ask, a few hundred a second.
#define EVENTS 4096       // Not per second: events are rare, keep the last few thousand.
#define PAIRS_PER_S 30    // ~28.2 Hz
//! Frame ring while a capture records frames without look-back: what the writer may fall behind.
#define LIVE_PAIRS 96
//! The reader starts this far (a fraction of the ring) ahead of the producer's wrap.
#define SLACK_DIV 16
//! Max timing columns kept (t_slam's two plus Basalt's stages).
#define MAX_TIMING 15
//! Records / pairs written per stream per round, so the control files are never waited on long.
#define READ_BUDGET 4096
#define PAIR_BUDGET 4

#define IDLE_POLL_MS 500
#define ACTIVE_POLL_MS 50
#define STATUS_INTERVAL_NS ((int64_t)1000 * U_TIME_1MS_IN_NS)
#define MAX_CMD_BYTES 4096
#define DEFAULT_MAX_S 600

//! frames.bin: per pair this header, then the left and the right image (size[0] and size[1] bytes).
#define FRAMES_MAGIC 0x50465258u // "XRFP" little endian
#define FRAMES_CODEC_RAW 0
#define FRAMES_CODEC_ZSTD 1

struct frames_pair_header
{
	uint32_t magic;
	uint32_t codec;
	int64_t euroc_ns;
	uint32_t width, height;
	uint32_t size[2];
};

enum stream
{
	S_IMU,
	S_CAM,
	S_POSE,
	S_TIMING,
	S_HEAD,
	S_EVENT,
	S_COUNT,
};

static const char *const stream_names[S_COUNT] = {"imu",         "camera_frames", "slam_poses",
                                                  "slam_timing", "head_pose",     "events"};

struct timing_record
{
	uint32_t count;
	int64_t ts[MAX_TIMING];
};

struct event_record
{
	int64_t ts_ns;
	char type[16];
	char text[104];
};

union any_record {
	struct xreal_air_record_imu imu;
	struct xreal_air_record_cam_frame cam;
	struct xreal_air_flight_slam_pose pose;
	struct timing_record timing;
	struct xreal_air_flight_head head;
	struct event_record event;
};

struct frame_slot
{
	int64_t euroc_ns;
	struct xrt_frame *frames[2];
};

//! Command parameters, key=value lines.
struct params
{
	char buf[MAX_CMD_BYTES];
	const char *keys[32];
	const char *values[32];
	int count;
};

//! A capture in progress, writer thread only.
struct capture
{
	bool active;
	char dir[PATH_MAX];
	FILE *imu_raw, *imu0, *cam_frames, *poses, *timing, *head, *events, *frames;
	bool timing_header;
	uint64_t cursor[S_COUNT];
	uint64_t written[S_COUNT];
	uint64_t lost[S_COUNT];
	int64_t first_ts[S_COUNT];

	bool with_frames;
	int codec;
	int level;
	uint64_t frame_cursor;
	uint64_t pairs_written;
	uint64_t pairs_lost;
	int64_t first_pair_ns;
	uint64_t frame_bytes;

	int64_t start_ns;
	int64_t lookback_ns;
	int64_t max_ns;
	char started_wall[64];
	int64_t next_check_ns;
	double mb;

	// For the IMU rate in the calibration files.
	uint64_t imu0_rows;
	int64_t imu0_first, imu0_last;

#ifdef XREAL_AIR_HAVE_ZSTD
	ZSTD_CCtx *zctx;
#endif
	uint8_t *zbuf;
	size_t zbuf_size;
	uint8_t *pack; //!< For frames whose stride is not their width.
};

struct xreal_air_flight
{
	struct xreal_air_flight_config cfg;

	struct xreal_air_flight_ring *rings[S_COUNT];

	//! The frame ring: references, so a mutex (held for a few pointer swaps on either side).
	struct
	{
		struct os_mutex lock;
		struct frame_slot *slots;
		uint32_t capacity; //!< 0: frames are dropped right away.
		uint64_t head;
	} frames;
	uint32_t armed_frames_s; //!< Writer thread only.

	//! Calibration and timing column names, rarely written.
	struct os_mutex meta_lock;
	char *calib_document;
	size_t calib_document_len;
	struct xreal_air_parsed_calibration calib;
	bool calib_valid;
	char timing_columns[512];

	struct os_thread_helper oth;
	int inotify_fd;
	int owner_fd; //!< flock()ed control directory: one runtime answers there.

	// Writer thread only.
	struct capture cap;
	int64_t next_status_ns;
	char cmd_id[64];
	char cmd_name[16];
	char cmd_result[256];
	char last_out[PATH_MAX];
	char last_stop_reason[128];
};


/*
 *
 * Small helpers.
 *
 */

static int64_t
now_ns(void)
{
	return (int64_t)os_monotonic_get_ns();
}

static bool
dir_is_missing_or_empty(const char *path)
{
	DIR *dir = opendir(path);
	if (dir == NULL) {
		return errno == ENOENT;
	}
	bool empty = true;
	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL) {
		if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
			empty = false;
			break;
		}
	}
	(void)closedir(dir);
	return empty;
}

//! Free bytes for an unprivileged user at @p path, -1 if unknown.
static int64_t
free_bytes(const char *path)
{
	struct statvfs st;
	if (statvfs(path, &st) != 0) {
		return -1;
	}
	return (int64_t)st.f_bavail * (int64_t)st.f_frsize;
}

static FILE *
open_in(struct xreal_air_flight *f, const char *rel, const char *header)
{
	char path[PATH_MAX + 64];
	(void)snprintf(path, sizeof(path), "%s/%s", f->cap.dir, rel);
	FILE *file = fopen(path, "w");
	if (file == NULL) {
		FL_ERROR(f, "Could not open '%s': %s", path, strerror(errno));
		return NULL;
	}
	// The writer does few large writes; flushed every round anyway.
	(void)setvbuf(file, NULL, _IOFBF, 1 << 18);
	if (header != NULL) {
		(void)fputs(header, file);
	}
	return file;
}

static void
close_file(FILE **file)
{
	if (*file != NULL) {
		(void)fclose(*file);
		*file = NULL;
	}
}

static void
wall_clock_string(char *out, size_t size)
{
	time_t now = time(NULL);
	struct tm tm;
	(void)localtime_r(&now, &tm);
	(void)strftime(out, size, "%Y-%m-%dT%H:%M:%S%z", &tm);
}

static const char *
param(const struct params *p, const char *key, const char *fallback)
{
	for (int i = 0; i < p->count; i++) {
		if (strcmp(p->keys[i], key) == 0) {
			return p->values[i];
		}
	}
	return fallback;
}

static int64_t
param_int(const struct params *p, const char *key, int64_t fallback)
{
	const char *v = param(p, key, NULL);
	if (v == NULL || v[0] == '\0') {
		return fallback;
	}
	char *end = NULL;
	const long long n = strtoll(v, &end, 10);
	return end != v ? (int64_t)n : fallback;
}

static void
parse_params(struct params *p)
{
	p->count = 0;
	char *save = NULL;
	for (char *line = strtok_r(p->buf, "\n", &save); line != NULL && p->count < 32;
	     line = strtok_r(NULL, "\n", &save)) {
		char *eq = strchr(line, '=');
		if (eq == NULL) {
			continue;
		}
		*eq = '\0';
		char *v = eq + 1;
		const size_t len = strlen(v);
		if (len > 0 && v[len - 1] == '\r') {
			v[len - 1] = '\0';
		}
		p->keys[p->count] = line;
		p->values[p->count] = v;
		p->count++;
	}
}

//! Into a CSV field: no commas, quotes or line breaks.
static void
csv_clean(char *s)
{
	for (; *s != '\0'; s++) {
		if (*s == ',' || *s == '"' || *s == '\n' || *s == '\r') {
			*s = *s == ',' ? ';' : ' ';
		}
	}
}

static const char *
verdict_name(int32_t v)
{
	switch (v) {
	case XREAL_AIR_SLAM_POSE_SAME: return "same";
	case XREAL_AIR_SLAM_POSE_FIRST: return "first";
	case XREAL_AIR_SLAM_POSE_OK: return "ok";
	case XREAL_AIR_SLAM_POSE_SUSPECT: return "suspect";
	case XREAL_AIR_SLAM_POSE_RESUMED: return "resumed";
	case XREAL_AIR_SLAM_POSE_REINIT: return "reinit";
	case XREAL_AIR_SLAM_POSE_DIVERGED: return "diverged";
	default: return "?";
	}
}

//! The time a record is "from", for the look-back cut-off.
static int64_t
record_time(enum stream s, const union any_record *r)
{
	switch (s) {
	case S_IMU: return r->imu.host_ns;
	case S_CAM: return r->cam.dequeue_ns;
	case S_POSE: return r->pose.seen_ns;
	case S_TIMING: return r->timing.count > 0 ? r->timing.ts[r->timing.count - 1] : 0;
	case S_HEAD: return r->head.query_ns;
	case S_EVENT: return r->event.ts_ns;
	default: return 0;
	}
}

static void
push_event_at(struct xreal_air_flight *f, int64_t ts, const char *type, const char *fmt, va_list args)
{
	struct event_record e;
	U_ZERO(&e);
	e.ts_ns = ts;
	(void)snprintf(e.type, sizeof(e.type), "%s", type);
	(void)vsnprintf(e.text, sizeof(e.text), fmt, args);
	csv_clean(e.type);
	csv_clean(e.text);
	xreal_air_flight_ring_push(f->rings[S_EVENT], &e);
}

static void
event_at(struct xreal_air_flight *f, int64_t ts, const char *type, const char *fmt, ...) XRT_PRINTF_FORMAT(4, 5);

static void
event_at(struct xreal_air_flight *f, int64_t ts, const char *type, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	push_event_at(f, ts, type, fmt, args);
	va_end(args);
}

#define event_now(f, ...) event_at(f, now_ns(), __VA_ARGS__)


/*
 *
 * Frame ring.
 *
 */

static void
release_slot(struct frame_slot *slot)
{
	xrt_frame_reference(&slot->frames[0], NULL);
	xrt_frame_reference(&slot->frames[1], NULL);
}

/*!
 * Resize the frame ring to @p capacity pairs, keeping the newest ones. Writer thread. The allocation
 * and the releasing happen outside the lock, only the moves inside.
 */
static void
frames_resize(struct xreal_air_flight *f, uint32_t capacity)
{
	os_mutex_lock(&f->frames.lock);
	const uint32_t old_capacity = f->frames.capacity;
	os_mutex_unlock(&f->frames.lock);
	if (capacity == old_capacity) {
		return;
	}

	struct frame_slot *slots = capacity > 0 ? U_TYPED_ARRAY_CALLOC(struct frame_slot, capacity) : NULL;
	if (capacity > 0 && slots == NULL) {
		FL_ERROR(f, "Could not allocate a frame ring of %u pairs", capacity);
		return;
	}

	os_mutex_lock(&f->frames.lock);
	struct frame_slot *old = f->frames.slots;
	const uint32_t old_cap = f->frames.capacity;
	const uint64_t head = f->frames.head;
	uint64_t keep = old_cap < capacity ? old_cap : capacity;
	keep = keep < head ? keep : head;
	for (uint64_t i = head - keep; i < head; i++) {
		slots[i % capacity] = old[i % old_cap];
		U_ZERO(&old[i % old_cap]);
	}
	f->frames.slots = slots;
	f->frames.capacity = capacity;
	os_mutex_unlock(&f->frames.lock);

	for (uint32_t i = 0; i < old_cap; i++) {
		release_slot(&old[i]);
	}
	free(old);
}

//! The capacity the frame ring needs right now.
static uint32_t
frames_wanted(struct xreal_air_flight *f)
{
	uint32_t pairs = f->armed_frames_s * PAIRS_PER_S;
	if (f->cap.active && f->cap.with_frames && pairs < LIVE_PAIRS) {
		pairs = LIVE_PAIRS;
	}
	return pairs;
}


/*
 *
 * Writing, writer thread.
 *
 */

static void
write_record(struct xreal_air_flight *f, enum stream s, const union any_record *r)
{
	struct capture *c = &f->cap;

	switch (s) {
	case S_IMU:
		if (xreal_air_record_write_imu_rows(c->imu_raw, c->imu0, &r->imu)) {
			if (c->imu0_rows++ == 0) {
				c->imu0_first = r->imu.euroc_ns;
			}
			c->imu0_last = r->imu.euroc_ns;
		}
		break;
	case S_CAM: xreal_air_record_write_cam_frame_row(c->cam_frames, &r->cam); break;
	case S_POSE: {
		const struct xreal_air_flight_slam_pose *p = &r->pose;
		(void)fprintf(c->poses,
		              "%" PRId64 ",%" PRId64 ",%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.4f,%.4f,%.4f,%u,%s\n",
		              p->ts_ns, p->seen_ns, (double)(p->seen_ns - p->ts_ns) / U_TIME_1MS_IN_NS,
		              p->pose.position.x, p->pose.position.y, p->pose.position.z, p->pose.orientation.x,
		              p->pose.orientation.y, p->pose.orientation.z, p->pose.orientation.w, p->linear_velocity.x,
		              p->linear_velocity.y, p->linear_velocity.z, p->world_id, verdict_name(p->verdict));
		break;
	}
	case S_TIMING: {
		if (!c->timing_header) {
			os_mutex_lock(&f->meta_lock);
			if (f->timing_columns[0] != '\0') {
				(void)fprintf(c->timing, "#%s\n", f->timing_columns);
				c->timing_header = true;
			}
			os_mutex_unlock(&f->meta_lock);
		}
		const uint32_t n = r->timing.count < MAX_TIMING ? r->timing.count : MAX_TIMING;
		for (uint32_t i = 0; i < n; i++) {
			(void)fprintf(c->timing, "%" PRId64 "%s", r->timing.ts[i], i + 1 < n ? "," : "\n");
		}
		break;
	}
	case S_HEAD: {
		const struct xreal_air_flight_head *h = &r->head;
		(void)fprintf(
		    c->head, "%" PRId64 ",%" PRId64 ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%d,%.7f,%.7f,%.7f,%.7f,%u\n",
		    h->query_ns, h->at_ns, h->pose.position.x, h->pose.position.y, h->pose.position.z,
		    h->pose.orientation.x, h->pose.orientation.y, h->pose.orientation.z, h->pose.orientation.w,
		    h->slam ? 1 : 0, h->fusion.x, h->fusion.y, h->fusion.z, h->fusion.w, h->world_id);
		break;
	}
	case S_EVENT:
		(void)fprintf(c->events, "%" PRId64 ",%s,%s\n", r->event.ts_ns, r->event.type, r->event.text);
		break;
	default: break;
	}
}

//! Write what is new in stream @p s. @return true if the budget ran out (more to do).
static bool
drain_stream(struct xreal_air_flight *f, enum stream s)
{
	struct capture *c = &f->cap;
	struct xreal_air_flight_ring *ring = f->rings[s];
	union any_record r;

	for (int i = 0; i < READ_BUDGET; i++) {
		switch (xreal_air_flight_ring_read(ring, c->cursor[s], &r)) {
		case XREAL_AIR_FLIGHT_RING_NOT_YET: return false;
		case XREAL_AIR_FLIGHT_RING_OVERWRITTEN: {
			// Fell behind: skip ahead, with slack so the next read is not overwritten again.
			uint64_t next =
			    xreal_air_flight_ring_oldest(ring, xreal_air_flight_ring_capacity(ring) / SLACK_DIV);
			if (next <= c->cursor[s]) {
				next = c->cursor[s] + 1;
			}
			c->lost[s] += next - c->cursor[s];
			c->cursor[s] = next;
			break;
		}
		case XREAL_AIR_FLIGHT_RING_OK:
			if (c->written[s] == 0) {
				c->first_ts[s] = record_time(s, &r);
			}
			write_record(f, s, &r);
			c->cursor[s]++;
			c->written[s]++;
			break;
		}
	}
	return true;
}

static bool
write_pair(struct xreal_air_flight *f, const struct frame_slot *slot)
{
	struct capture *c = &f->cap;
	const struct xrt_frame *l = slot->frames[0];
	const struct xrt_frame *r = slot->frames[1];
	if (l == NULL || r == NULL || l->width != r->width || l->height != r->height) {
		return false;
	}

	const uint32_t w = l->width, h = l->height;
	const size_t image = (size_t)w * h;
	struct frames_pair_header hdr = {
	    .magic = FRAMES_MAGIC,
	    .codec = (uint32_t)c->codec,
	    .euroc_ns = slot->euroc_ns,
	    .width = w,
	    .height = h,
	};

	const uint8_t *data[2];
	size_t sizes[2];
	for (int i = 0; i < 2; i++) {
		const struct xrt_frame *xf = slot->frames[i];
		const uint8_t *src = xf->data;
		if (xf->stride != w) {
			if (c->pack == NULL) {
				c->pack = malloc(image * 2);
			}
			if (c->pack == NULL) {
				return false;
			}
			for (uint32_t y = 0; y < h; y++) {
				memcpy(c->pack + i * image + (size_t)y * w, xf->data + (size_t)y * xf->stride, w);
			}
			src = c->pack + i * image;
		}

#ifdef XREAL_AIR_HAVE_ZSTD
		if (c->codec == FRAMES_CODEC_ZSTD) {
			const size_t bound = ZSTD_compressBound(image);
			if (c->zbuf_size < bound * 2) {
				free(c->zbuf);
				c->zbuf = malloc(bound * 2);
				c->zbuf_size = c->zbuf != NULL ? bound * 2 : 0;
			}
			if (c->zbuf == NULL) {
				return false;
			}
			const size_t n = ZSTD_compressCCtx(c->zctx, c->zbuf + i * bound, bound, src, image, c->level);
			if (ZSTD_isError(n)) {
				return false;
			}
			data[i] = c->zbuf + i * bound;
			sizes[i] = n;
			continue;
		}
#endif
		data[i] = src;
		sizes[i] = image;
	}

	hdr.size[0] = (uint32_t)sizes[0];
	hdr.size[1] = (uint32_t)sizes[1];
	if (fwrite(&hdr, sizeof(hdr), 1, c->frames) != 1 || fwrite(data[0], 1, sizes[0], c->frames) != sizes[0] ||
	    fwrite(data[1], 1, sizes[1], c->frames) != sizes[1]) {
		return false;
	}
	c->frame_bytes += sizeof(hdr) + sizes[0] + sizes[1];
	return true;
}

//! Write the queued stereo pairs. @return true if the budget ran out (more to do).
static bool
drain_frames(struct xreal_air_flight *f)
{
	struct capture *c = &f->cap;
	if (!c->with_frames || c->frames == NULL) {
		return false;
	}

	for (int i = 0; i < PAIR_BUDGET; i++) {
		struct frame_slot slot = {0};

		os_mutex_lock(&f->frames.lock);
		const uint64_t head = f->frames.head;
		const uint32_t cap = f->frames.capacity;
		if (c->frame_cursor >= head) {
			os_mutex_unlock(&f->frames.lock);
			return false;
		}
		if (cap == 0 || head - c->frame_cursor > cap) {
			const uint64_t next = cap == 0 ? head : head - cap;
			c->pairs_lost += next - c->frame_cursor;
			c->frame_cursor = next;
			os_mutex_unlock(&f->frames.lock);
			continue;
		}
		const struct frame_slot *src = &f->frames.slots[c->frame_cursor % cap];
		slot.euroc_ns = src->euroc_ns;
		xrt_frame_reference(&slot.frames[0], src->frames[0]);
		xrt_frame_reference(&slot.frames[1], src->frames[1]);
		os_mutex_unlock(&f->frames.lock);

		c->frame_cursor++;
		const bool written = write_pair(f, &slot);
		release_slot(&slot);

		// Without a frame look-back nothing needs a written pair again: give its memory back now
		// rather than when the slot is reused.
		if (f->armed_frames_s == 0) {
			struct frame_slot done = {0};
			os_mutex_lock(&f->frames.lock);
			const uint64_t index = c->frame_cursor - 1;
			if (f->frames.capacity > 0 && f->frames.head - index <= f->frames.capacity) {
				struct frame_slot *held = &f->frames.slots[index % f->frames.capacity];
				done = *held;
				U_ZERO(held);
			}
			os_mutex_unlock(&f->frames.lock);
			release_slot(&done);
		}

		if (written) {
			if (c->pairs_written++ == 0) {
				c->first_pair_ns = slot.euroc_ns;
			}
		} else {
			c->pairs_lost++;
		}
	}
	return true;
}

static void
flush_capture(struct capture *c)
{
	FILE *files[] = {c->imu_raw, c->imu0, c->cam_frames, c->poses, c->timing, c->head, c->events, c->frames};
	for (size_t i = 0; i < ARRAY_SIZE(files); i++) {
		if (files[i] != NULL) {
			(void)fflush(files[i]);
		}
	}
}

static double
capture_mb(struct capture *c)
{
	FILE *files[] = {c->imu_raw, c->imu0, c->cam_frames, c->poses, c->timing, c->head, c->events, c->frames};
	double bytes = 0;
	for (size_t i = 0; i < ARRAY_SIZE(files); i++) {
		if (files[i] != NULL) {
			const long pos = ftell(files[i]);
			bytes += pos > 0 ? (double)pos : 0;
		}
	}
	return bytes / (1024.0 * 1024.0);
}

static const char flight_readme[] =
    "XREAL Air 2 Ultra flight recorder capture (Monado xreal_air driver, xreal_air_flight.h)\n"
    "======================================================================================\n"
    "\n"
    "Started on demand while the runtime ran; the first lookback_s seconds (capture.txt) come\n"
    "from the always-on RAM rings, i.e. from BEFORE the capture was asked for. All times are\n"
    "CLOCK_MONOTONIC ns (the journal's _SOURCE_MONOTONIC_TIMESTAMP is the same clock, in us).\n"
    "\n"
    "capture.txt             What was asked for, what was written and what was lost (written_*,\n"
    "                        lost_*: records the writer did not get to before the ring wrapped).\n"
    "mav0/imu0/data.csv      EuRoC IMU, as in a XREAL_AIR_EUROC_RECORD dataset.\n"
    "xreal/                  As in a XREAL_AIR_EUROC_RECORD dataset (see xreal/README.txt):\n"
    "                        camera_frames.csv (every frame: kernel / dequeue times, so frame\n"
    "                        gaps and dequeue stalls), imu_raw.csv, the calibration files.\n"
    "xreal/frames.bin        The stereo pairs, if recorded: per pair a 32 byte header (magic\n"
    "                        \"XRFP\", codec 0 raw / 1 zstd, int64 EuRoC ns, width, height, the two\n"
    "                        sizes), then the left and the right 8-bit image. Expand with\n"
    "                        scripts/xreal-capture-to-euroc.py (Monado) into mav0/cam0, mav0/cam1;\n"
    "                        the directory then replays with monado-cli xreal-slam-replay.\n"
    "flight/events.csv       ts_ns,type,text: marks (\"I see it NOW\"), capture start / stop,\n"
    "                        SLAM state changes (tracking lost / acquired, first pose, resumed,\n"
    "                        reinit, suspect, diverged), wear, tracker restarts. Not rate limited.\n"
    "                        The whole event ring, so it reaches further back than lookback_s.\n"
    "flight/slam_poses.csv   Each new SLAM pose as the driver first saw it: ts_ns (pose time,\n"
    "                        mid-exposure), seen_ns, latency_ms, position and orientation in\n"
    "                        Basalt's world (Z up, IMU body), velocity, the pose guard's world_id\n"
    "                        and verdict.\n"
    "flight/slam_timing.csv  t_slam's per-stage timestamps of each pose (= SLAM_WRITE_CSVS\n"
    "                        timing.csv; scripts/slam-timing-summary.py reads it).\n"
    "flight/head_pose.csv    Every head pose handed out: query_ns, at_ns (the time asked for),\n"
    "                        the output pose (OpenXR, Y up), slam (1 = followed SLAM, 0 = 3DoF),\n"
    "                        the 3DoF fusion's orientation, world_id.\n";

static void
write_capture_txt(struct xreal_air_flight *f, const char *stop_reason)
{
	struct capture *c = &f->cap;
	FILE *file = open_in(f, "capture.txt", NULL);
	if (file == NULL) {
		return;
	}
	const int64_t now = now_ns();
	(void)fprintf(file, "version=%s\npid=%d\nout=%s\nstarted=%s\nstart_mono_ns=%" PRId64 "\n", u_git_tag,
	              (int)getpid(), c->dir, c->started_wall, c->start_ns);
	(void)fprintf(file, "lookback_s=%.1f\nmax_s=%.0f\n", (double)c->lookback_ns / U_TIME_1S_IN_NS,
	              (double)c->max_ns / U_TIME_1S_IN_NS);
	(void)fprintf(file, "frames=%d\ncodec=%s\nzstd_level=%d\nframes_lookback_s=%u\nrings_s=%u\n",
	              c->with_frames ? 1 : 0, c->codec == FRAMES_CODEC_ZSTD ? "zstd" : "raw", c->level,
	              f->armed_frames_s, f->cfg.lookback_s);
	if (stop_reason != NULL) {
		(void)fprintf(file, "stop_mono_ns=%" PRId64 "\nstop_reason=%s\nduration_live_s=%.1f\n", now,
		              stop_reason, (double)(now - c->start_ns) / U_TIME_1S_IN_NS);
	}
	for (int s = 0; s < S_COUNT; s++) {
		(void)fprintf(file, "written_%s=%" PRIu64 "\nlost_%s=%" PRIu64 "\nfirst_ns_%s=%" PRId64 "\n",
		              stream_names[s], c->written[s], stream_names[s], c->lost[s], stream_names[s],
		              c->written[s] > 0 ? c->first_ts[s] : -1);
	}
	(void)fprintf(file, "written_pairs=%" PRIu64 "\nlost_pairs=%" PRIu64 "\nfirst_pair_ns=%" PRId64 "\n",
	              c->pairs_written, c->pairs_lost, c->pairs_written > 0 ? c->first_pair_ns : -1);
	(void)fprintf(file, "frame_mb=%.1f\ntotal_mb=%.1f\n", (double)c->frame_bytes / (1024.0 * 1024.0),
	              capture_mb(c));
	close_file(&file);
}

static void
write_calibration(struct xreal_air_flight *f)
{
	struct capture *c = &f->cap;

	os_mutex_lock(&f->meta_lock);
	const bool valid = f->calib_valid;
	struct xreal_air_parsed_calibration calib = f->calib;
	char *doc = NULL;
	const size_t doc_len = f->calib_document_len;
	if (valid && f->calib_document != NULL) {
		doc = malloc(doc_len);
		if (doc != NULL) {
			memcpy(doc, f->calib_document, doc_len);
		}
	}
	os_mutex_unlock(&f->meta_lock);

	if (!valid) {
		return;
	}
	double rate = 0;
	if (c->imu0_rows > 1000 && c->imu0_last > c->imu0_first) {
		rate = (double)(c->imu0_rows - 1) * 1e9 / (double)(c->imu0_last - c->imu0_first);
	}
	xreal_air_record_write_calibration(c->dir, f->cfg.log_level, doc, doc_len, &calib, rate);
	free(doc);
}


/*
 *
 * Capture control, writer thread.
 *
 */

//! The first index of ring @p s at or after @p cutoff_ns (all of the readable ring for events).
static uint64_t
lookback_cursor(struct xreal_air_flight *f, enum stream s, int64_t cutoff_ns)
{
	struct xreal_air_flight_ring *ring = f->rings[s];
	uint64_t i = xreal_air_flight_ring_oldest(ring, xreal_air_flight_ring_capacity(ring) / SLACK_DIV);
	const uint64_t head = xreal_air_flight_ring_head(ring);
	union any_record r;
	// The whole event ring: the state changes before the window explain what happens in it.
	if (s == S_EVENT) {
		return i;
	}
	for (; i < head; i++) {
		if (xreal_air_flight_ring_read(ring, i, &r) == XREAL_AIR_FLIGHT_RING_OK &&
		    record_time(s, &r) >= cutoff_ns) {
			break;
		}
	}
	return i;
}

static void
close_capture_files(struct capture *c)
{
	close_file(&c->imu_raw);
	close_file(&c->imu0);
	close_file(&c->cam_frames);
	close_file(&c->poses);
	close_file(&c->timing);
	close_file(&c->head);
	close_file(&c->events);
	close_file(&c->frames);
#ifdef XREAL_AIR_HAVE_ZSTD
	ZSTD_freeCCtx(c->zctx);
	c->zctx = NULL;
#endif
	free(c->zbuf);
	c->zbuf = NULL;
	c->zbuf_size = 0;
	free(c->pack);
	c->pack = NULL;
}

static bool
start_capture(struct xreal_air_flight *f, const struct params *p, char *err, size_t err_size)
{
	struct capture *c = &f->cap;
	if (c->active) {
		(void)snprintf(err, err_size, "already recording to %.400s", c->dir);
		return false;
	}

	const char *out = param(p, "out", "");
	if (out[0] != '/') {
		(void)snprintf(err, err_size, "out= must be an absolute path, got '%s'", out);
		return false;
	}
	if (!dir_is_missing_or_empty(out)) {
		(void)snprintf(err, err_size, "%s exists and is not empty", out);
		return false;
	}
	if (!xreal_air_record_mkdir_p(out)) {
		(void)snprintf(err, err_size, "cannot create %s: %s", out, strerror(errno));
		return false;
	}
	const int64_t free_b = free_bytes(out);
	if (free_b >= 0 && free_b < (int64_t)f->cfg.min_free_mb * 1024 * 1024) {
		(void)rmdir(out);
		(void)snprintf(err, err_size,
		               "only %" PRId64 " MB free at %s, need %u MB (XREAL_AIR_FLIGHT_MIN_FREE_MB)",
		               free_b / (1024 * 1024), out, f->cfg.min_free_mb);
		return false;
	}

	struct capture fresh;
	U_ZERO(&fresh);
	*c = fresh;
	(void)snprintf(c->dir, sizeof(c->dir), "%s", out);

	static const char *const dirs[] = {"mav0/imu0", "xreal", "flight"};
	for (size_t i = 0; i < ARRAY_SIZE(dirs); i++) {
		char path[PATH_MAX + 64];
		(void)snprintf(path, sizeof(path), "%s/%s", c->dir, dirs[i]);
		if (!xreal_air_record_mkdir_p(path)) {
			(void)snprintf(err, err_size, "cannot create %.400s: %s", path, strerror(errno));
			return false;
		}
	}

	c->with_frames = param_int(p, "frames", 1) != 0;
	const char *codec = param(p, "codec", NULL);
#ifdef XREAL_AIR_HAVE_ZSTD
	c->codec = codec != NULL && strcmp(codec, "raw") == 0 ? FRAMES_CODEC_RAW : FRAMES_CODEC_ZSTD;
#else
	if (codec != NULL && strcmp(codec, "zstd") == 0) {
		FL_NOTE(f, "Built without zstd, writing raw frames");
	}
	c->codec = FRAMES_CODEC_RAW;
#endif
	c->level = (int)param_int(p, "level", 1);

	c->imu_raw = open_in(f, "xreal/imu_raw.csv", xreal_air_record_imu_raw_header);
	c->imu0 = open_in(f, "mav0/imu0/data.csv", xreal_air_record_imu0_header);
	c->cam_frames = open_in(f, "xreal/camera_frames.csv", xreal_air_record_cam_frames_header);
	c->poses = open_in(f, "flight/slam_poses.csv",
	                   "ts_ns,seen_ns,latency_ms,px,py,pz,qx,qy,qz,qw,vx,vy,vz,world_id,verdict\n");
	c->timing = open_in(f, "flight/slam_timing.csv", NULL);
	c->head =
	    open_in(f, "flight/head_pose.csv",
	            "query_ns,at_ns,px,py,pz,qx,qy,qz,qw,slam,fusion_qx,fusion_qy,fusion_qz,fusion_qw,world_id\n");
	c->events = open_in(f, "flight/events.csv", "ts_ns,type,text\n");
	if (c->with_frames) {
		c->frames = open_in(f, "xreal/frames.bin", NULL);
	}
	FILE *readme = open_in(f, "xreal/README.txt", xreal_air_record_readme);
	close_file(&readme);
	readme = open_in(f, "README.txt", flight_readme);
	close_file(&readme);

	if (c->imu_raw == NULL || c->imu0 == NULL || c->cam_frames == NULL || c->poses == NULL || c->timing == NULL ||
	    c->head == NULL || c->events == NULL || (c->with_frames && c->frames == NULL)) {
		close_capture_files(c);
		(void)snprintf(err, err_size, "cannot create the files in %.400s", c->dir);
		return false;
	}
#ifdef XREAL_AIR_HAVE_ZSTD
	if (c->codec == FRAMES_CODEC_ZSTD) {
		c->zctx = ZSTD_createCCtx();
	}
#endif

	const int64_t now = now_ns();
	const int64_t ring_ns = (int64_t)f->cfg.lookback_s * U_TIME_1S_IN_NS;
	int64_t lookback_ns = param_int(p, "lookback_s", f->cfg.lookback_s) * U_TIME_1S_IN_NS;
	lookback_ns = lookback_ns < 0 ? 0 : (lookback_ns > ring_ns ? ring_ns : lookback_ns);
	int64_t max_s = param_int(p, "max_s", f->cfg.default_max_s);
	max_s = max_s <= 0 || max_s > 24 * 3600 ? f->cfg.default_max_s : max_s;

	c->start_ns = now;
	c->lookback_ns = lookback_ns;
	c->max_ns = max_s * U_TIME_1S_IN_NS;
	c->next_check_ns = now;
	wall_clock_string(c->started_wall, sizeof(c->started_wall));

	for (int s = 0; s < S_COUNT; s++) {
		c->cursor[s] = lookback_cursor(f, (enum stream)s, now - lookback_ns);
	}

	c->active = true;
	if (c->with_frames) {
		frames_resize(f, frames_wanted(f));
		os_mutex_lock(&f->frames.lock);
		const uint64_t head = f->frames.head;
		const uint32_t cap = f->frames.capacity;
		uint64_t i = cap > 0 && head > cap ? head - cap : 0;
		for (; i < head; i++) {
			const struct frame_slot *slot = &f->frames.slots[i % cap];
			if (slot->frames[0] != NULL && slot->euroc_ns >= now - lookback_ns) {
				break;
			}
		}
		c->frame_cursor = i;
		os_mutex_unlock(&f->frames.lock);
	}

	write_calibration(f);
	write_capture_txt(f, NULL);
	event_now(f, "capture", "start out=%s lookback_s=%.0f frames=%d", c->dir, (double)lookback_ns / U_TIME_1S_IN_NS,
	          c->with_frames ? 1 : 0);
	FL_NOTE(f, "Capture started: '%s' (%.0f s look-back, frames %s, stops after %" PRId64 " s)", c->dir,
	        (double)lookback_ns / U_TIME_1S_IN_NS,
	        !c->with_frames ? "off" : (c->codec == FRAMES_CODEC_ZSTD ? "zstd" : "raw"), max_s);
	(void)snprintf(f->last_out, sizeof(f->last_out), "%s", c->dir);
	return true;
}

static void
stop_capture(struct xreal_air_flight *f, const char *reason)
{
	struct capture *c = &f->cap;
	if (!c->active) {
		return;
	}

	event_now(f, "capture", "stop reason=%s", reason);
	// Everything up to now, the stop event included.
	while (true) {
		bool more = false;
		for (int s = 0; s < S_COUNT; s++) {
			more = drain_stream(f, (enum stream)s) || more;
		}
		more = drain_frames(f) || more;
		if (!more) {
			break;
		}
	}
	flush_capture(c);

	write_calibration(f);
	write_capture_txt(f, reason);

	uint64_t lost = 0;
	for (int s = 0; s < S_COUNT; s++) {
		lost += c->lost[s];
	}
	FL_NOTE(f,
	        "Capture stopped (%s): '%s', %.1f s live, %" PRIu64 " IMU samples, %" PRIu64
	        " pairs (%.0f MB), %" PRIu64 " records and %" PRIu64 " pairs lost",
	        reason, c->dir, (double)(now_ns() - c->start_ns) / U_TIME_1S_IN_NS, c->written[S_IMU], c->pairs_written,
	        (double)c->frame_bytes / (1024.0 * 1024.0), lost, c->pairs_lost);

	close_capture_files(c);
	(void)snprintf(f->last_stop_reason, sizeof(f->last_stop_reason), "%s", reason);
	c->active = false;
	frames_resize(f, frames_wanted(f));
}

static void
check_limits(struct xreal_air_flight *f, int64_t now)
{
	struct capture *c = &f->cap;
	if (!c->active || now < c->next_check_ns) {
		return;
	}
	c->next_check_ns = now + STATUS_INTERVAL_NS;
	c->mb = capture_mb(c);

	char reason[128];
	reason[0] = '\0';
	const int64_t free_b = free_bytes(c->dir);
	if (now - c->start_ns >= c->max_ns) {
		(void)snprintf(reason, sizeof(reason), "max_s reached");
	} else if (free_b >= 0 && free_b < (int64_t)f->cfg.min_free_mb * 1024 * 1024 / 2) {
		(void)snprintf(reason, sizeof(reason), "disk low (%" PRId64 " MB free)", free_b / (1024 * 1024));
	} else if (c->mb >= (double)f->cfg.max_mb) {
		(void)snprintf(reason, sizeof(reason), "XREAL_AIR_FLIGHT_MAX_MB reached");
	}
	if (reason[0] != '\0') {
		stop_capture(f, reason);
	}
}

static void
write_status(struct xreal_air_flight *f)
{
	char tmp[PATH_MAX], path[PATH_MAX];
	(void)snprintf(tmp, sizeof(tmp), "%s/.status.tmp", f->cfg.ctl_dir);
	(void)snprintf(path, sizeof(path), "%s/status", f->cfg.ctl_dir);

	FILE *file = fopen(tmp, "w");
	if (file == NULL) {
		return;
	}
	const struct capture *c = &f->cap;
	uint64_t lost = 0;
	for (int s = 0; s < S_COUNT; s++) {
		lost += c->lost[s];
	}
	(void)fprintf(file,
	              "state=%s\npid=%d\nversion=%s\nrings_s=%u\nframes_lookback_s=%u\nzstd=%d\n"
	              "out=%s\nelapsed_s=%.1f\nmax_s=%.0f\nframes=%d\npairs=%" PRIu64 "\nlost_pairs=%" PRIu64
	              "\nlost_records=%" PRIu64
	              "\nmb=%.1f\n"
	              "cmd=%s\ncmd_id=%s\ncmd_result=%s\nlast_out=%s\nlast_stop_reason=%s\n",
	              c->active ? "recording" : "idle", (int)getpid(), u_git_tag, f->cfg.lookback_s, f->armed_frames_s,
#ifdef XREAL_AIR_HAVE_ZSTD
	              1,
#else
	              0,
#endif
	              c->active ? c->dir : "", c->active ? (double)(now_ns() - c->start_ns) / U_TIME_1S_IN_NS : 0.0,
	              (double)c->max_ns / U_TIME_1S_IN_NS, c->with_frames ? 1 : 0, c->pairs_written, c->pairs_lost,
	              lost, c->mb, f->cmd_name, f->cmd_id, f->cmd_result, f->last_out, f->last_stop_reason);

	// What the head pose does right now, from the newest records.
	union any_record r;
	const int64_t now = now_ns();
	const uint64_t heads = xreal_air_flight_ring_head(f->rings[S_HEAD]);
	if (heads > 0 && xreal_air_flight_ring_read(f->rings[S_HEAD], heads - 1, &r) == XREAL_AIR_FLIGHT_RING_OK) {
		(void)fprintf(file, "head_age_ms=%.0f\nslam_tracking=%d\nslam_world=%u\n",
		              (double)(now - r.head.query_ns) / U_TIME_1MS_IN_NS, r.head.slam ? 1 : 0, r.head.world_id);
	}
	const uint64_t poses = xreal_air_flight_ring_head(f->rings[S_POSE]);
	if (poses > 0 && xreal_air_flight_ring_read(f->rings[S_POSE], poses - 1, &r) == XREAL_AIR_FLIGHT_RING_OK) {
		(void)fprintf(file, "slam_pose_age_ms=%.0f\nslam_pose_latency_ms=%.1f\n",
		              (double)(now - r.pose.ts_ns) / U_TIME_1MS_IN_NS,
		              (double)(r.pose.seen_ns - r.pose.ts_ns) / U_TIME_1MS_IN_NS);
	}
	(void)fprintf(file, "status_mono_ns=%" PRId64 "\n", now);

	(void)fclose(file);
	(void)rename(tmp, path);
}

//! Read and delete command file @p name. @return false if there is none.
static bool
take_command(struct xreal_air_flight *f, const char *name, struct params *p)
{
	char path[PATH_MAX];
	(void)snprintf(path, sizeof(path), "%s/%s", f->cfg.ctl_dir, name);
	const int fd = open(path, O_RDONLY | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK);
	if (fd < 0) {
		return false;
	}
	ssize_t n = read(fd, p->buf, sizeof(p->buf) - 1);
	(void)close(fd);
	(void)unlink(path);
	p->buf[n > 0 ? n : 0] = '\0';
	parse_params(p);
	return true;
}

static void
command_done(struct xreal_air_flight *f, const char *name, const struct params *p, bool ok, const char *err)
{
	(void)snprintf(f->cmd_name, sizeof(f->cmd_name), "%s", name);
	(void)snprintf(f->cmd_id, sizeof(f->cmd_id), "%s", param(p, "id", ""));
	(void)snprintf(f->cmd_result, sizeof(f->cmd_result), "%s%s", ok ? "ok" : "error: ", ok ? "" : err);
	csv_clean(f->cmd_id);
	if (!ok) {
		FL_ERROR(f, "%s refused: %s", name, err);
	}
	f->next_status_ns = 0;
}

static void
handle_commands(struct xreal_air_flight *f)
{
	struct params p;
	char err[512];

	if (take_command(f, "start", &p)) {
		err[0] = '\0';
		const bool ok = start_capture(f, &p, err, sizeof(err));
		command_done(f, "start", &p, ok, err);
	}

	if (take_command(f, "mark", &p)) {
		// At the caller's time if it gave one (the moment the key was pressed), else now.
		const int64_t read_ns = now_ns();
		int64_t ts = param_int(&p, "mono_ns", read_ns);
		ts = ts > 0 && ts <= read_ns ? ts : read_ns;
		char label[96];
		(void)snprintf(label, sizeof(label), "%s", param(&p, "label", ""));
		event_at(f, ts, "mark", "%s (read %.0f ms later)", label, (double)(read_ns - ts) / U_TIME_1MS_IN_NS);
		FL_NOTE(f, "Mark '%s'%s", label, f->cap.active ? "" : " (not recording, kept in the ring)");
		command_done(f, "mark", &p, true, NULL);
	}

	if (take_command(f, "arm", &p)) {
		int64_t s = param_int(&p, "frames_s", 0);
		s = s < 0 ? 0 : (s > 120 ? 120 : s);
		f->armed_frames_s = (uint32_t)s;
		frames_resize(f, frames_wanted(f));
		event_now(f, "capture", "arm frames_s=%u", f->armed_frames_s);
		FL_NOTE(f, "Frame look-back %u s (~%.0f MB of RAM)", f->armed_frames_s,
		        (double)f->armed_frames_s * PAIRS_PER_S * 2 * 480 * 640 / (1024 * 1024));
		command_done(f, "arm", &p, true, NULL);
	}

	if (take_command(f, "stop", &p)) {
		if (f->cap.active) {
			char reason[96];
			(void)snprintf(reason, sizeof(reason), "%s", param(&p, "reason", "stop command"));
			csv_clean(reason);
			stop_capture(f, reason);
			command_done(f, "stop", &p, true, NULL);
		} else {
			command_done(f, "stop", &p, false, "not recording");
		}
	}
}

static void
lower_priority(struct xreal_air_flight *f)
{
	// Not whatever the creating thread had (it may be real-time), then below the tracker.
	struct sched_param sp = {0};
	(void)pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
	if (setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), f->cfg.writer_nice) != 0) {
		FL_INFO(f, "Could not renice the writer: %s", strerror(errno));
	}
}

static void *
writer_thread(void *ptr)
{
	struct xreal_air_flight *f = (struct xreal_air_flight *)ptr;
	os_thread_helper_name(&f->oth, "XREAL Air Flight");
	lower_priority(f);

	while (os_thread_helper_is_running(&f->oth)) {
		struct capture *c = &f->cap;
		bool more = false;
		if (c->active) {
			for (int s = 0; s < S_COUNT; s++) {
				more = drain_stream(f, (enum stream)s) || more;
			}
			more = drain_frames(f) || more;
			flush_capture(c);
		}

		const int timeout_ms = more ? 0 : (c->active ? ACTIVE_POLL_MS : IDLE_POLL_MS);
		if (f->inotify_fd >= 0) {
			struct pollfd pfd = {.fd = f->inotify_fd, .events = POLLIN};
			if (poll(&pfd, 1, timeout_ms) > 0) {
				char buf[4096];
				while (read(f->inotify_fd, buf, sizeof(buf)) > 0) {
					// Only a wake-up, the command files are looked for below.
				}
			}
		} else if (timeout_ms > 0) {
			os_nanosleep((int64_t)timeout_ms * U_TIME_1MS_IN_NS);
		}

		handle_commands(f);

		const int64_t now = now_ns();
		check_limits(f, now);
		if (now >= f->next_status_ns) {
			f->next_status_ns = now + STATUS_INTERVAL_NS;
			write_status(f);
		}
	}

	stop_capture(f, "runtime shutting down");
	write_status(f);
	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

bool
xreal_air_flight_default_config(enum u_logging_level log_level, struct xreal_air_flight_config *out)
{
	U_ZERO(out);
	out->log_level = log_level;
	out->lookback_s = (uint32_t)debug_get_num_option_xreal_air_flight_lookback_s();
	out->lookback_s = out->lookback_s < 1 ? 1 : (out->lookback_s > 600 ? 600 : out->lookback_s);
	out->frames_lookback_s = (uint32_t)debug_get_num_option_xreal_air_flight_frames_s();
	out->min_free_mb = (uint32_t)debug_get_num_option_xreal_air_flight_min_free_mb();
	out->max_mb = (uint64_t)debug_get_num_option_xreal_air_flight_max_mb();
	out->default_max_s = DEFAULT_MAX_S;
	out->writer_nice = 10;

	const char *ctl = debug_get_option_xreal_air_flight_ctl();
	const char *rt = getenv("RUNTIME_DIRECTORY");
	const char *xdg = getenv("XDG_RUNTIME_DIR");
	if (ctl != NULL && ctl[0] != '\0') {
		(void)snprintf(out->ctl_dir, sizeof(out->ctl_dir), "%s", ctl);
	} else if (rt != NULL && rt[0] == '/' && strchr(rt, ':') == NULL) {
		// systemd's RuntimeDirectory= (one directory; several are ':' separated, then use the default).
		(void)snprintf(out->ctl_dir, sizeof(out->ctl_dir), "%s/capture", rt);
	} else if (xdg != NULL && xdg[0] == '/') {
		(void)snprintf(out->ctl_dir, sizeof(out->ctl_dir), "%s/monado-xreal/capture", xdg);
	} else {
		(void)snprintf(out->ctl_dir, sizeof(out->ctl_dir), "/tmp/monado-xreal-%d/capture", (int)getuid());
	}

	return debug_get_bool_option_xreal_air_flight();
}

struct xreal_air_flight *
xreal_air_flight_create(const struct xreal_air_flight_config *config)
{
	struct xreal_air_flight *f = U_TYPED_CALLOC(struct xreal_air_flight);
	if (f == NULL) {
		return NULL;
	}
	f->cfg = *config;
	f->inotify_fd = -1;
	f->owner_fd = -1;
	if (os_mutex_init(&f->frames.lock) != 0 || os_mutex_init(&f->meta_lock) != 0 ||
	    os_thread_helper_init(&f->oth) != 0) {
		free(f);
		return NULL;
	}
	f->armed_frames_s = config->frames_lookback_s > 120 ? 120 : config->frames_lookback_s;

	const uint64_t lb = config->lookback_s;
	const uint64_t counts[S_COUNT] = {
	    [S_IMU] = lb * IMU_PER_S,      [S_CAM] = lb * CAM_ROWS_PER_S, [S_POSE] = lb * POSES_PER_S,
	    [S_TIMING] = lb * POSES_PER_S, [S_HEAD] = lb * HEADS_PER_S,   [S_EVENT] = EVENTS,
	};
	const size_t sizes[S_COUNT] = {
	    [S_IMU] = sizeof(struct xreal_air_record_imu),        [S_CAM] = sizeof(struct xreal_air_record_cam_frame),
	    [S_POSE] = sizeof(struct xreal_air_flight_slam_pose), [S_TIMING] = sizeof(struct timing_record),
	    [S_HEAD] = sizeof(struct xreal_air_flight_head),      [S_EVENT] = sizeof(struct event_record),
	};
	size_t total = 0;
	for (int s = 0; s < S_COUNT; s++) {
		f->rings[s] = xreal_air_flight_ring_create(sizes[s], counts[s]);
		if (f->rings[s] == NULL) {
			FL_ERROR(f, "Could not allocate the %s ring", stream_names[s]);
			xreal_air_flight_destroy(&f);
			return NULL;
		}
		total += xreal_air_flight_ring_bytes(f->rings[s]);
	}

	frames_resize(f, frames_wanted(f));

	if (!xreal_air_record_mkdir_p(f->cfg.ctl_dir)) {
		FL_ERROR(f, "Could not create the control directory '%s': %s", f->cfg.ctl_dir, strerror(errno));
	} else {
		(void)chmod(f->cfg.ctl_dir, 0700);

		// Another runtime (or a monado-cli opening the glasses meanwhile) owns it: stay out.
		char owner[PATH_MAX + 16];
		(void)snprintf(owner, sizeof(owner), "%s/.owner", f->cfg.ctl_dir);
		f->owner_fd = open(owner, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
		if (f->owner_fd < 0 || flock(f->owner_fd, LOCK_EX | LOCK_NB) != 0) {
			FL_NOTE(f, "Off: another process owns the control directory '%s'", f->cfg.ctl_dir);
			xreal_air_flight_destroy(&f);
			return NULL;
		}

		f->inotify_fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
		if (f->inotify_fd >= 0 &&
		    inotify_add_watch(f->inotify_fd, f->cfg.ctl_dir, IN_CLOSE_WRITE | IN_MOVED_TO) < 0) {
			(void)close(f->inotify_fd);
			f->inotify_fd = -1;
		}
	}

	// Leftovers from an earlier run (a checkout's control directory outlives the runtime), before
	// anything can drop a fresh one.
	static const char *const commands[] = {"start", "stop", "mark", "arm"};
	for (size_t i = 0; i < ARRAY_SIZE(commands); i++) {
		char path[PATH_MAX + 16];
		(void)snprintf(path, sizeof(path), "%s/%s", f->cfg.ctl_dir, commands[i]);
		if (unlink(path) == 0) {
			FL_NOTE(f, "Ignored a stale '%s' command file", commands[i]);
		}
	}
	write_status(f);

	if (os_thread_helper_start(&f->oth, writer_thread, f) != 0) {
		FL_ERROR(f, "Could not start the writer thread");
		xreal_air_flight_destroy(&f);
		return NULL;
	}

	FL_NOTE(f, "On: %u s of look-back in %.1f MB of rings, frame look-back %u s, control directory '%s'",
	        config->lookback_s, (double)total / (1024.0 * 1024.0), f->armed_frames_s, f->cfg.ctl_dir);
	return f;
}

void
xreal_air_flight_set_calibration(struct xreal_air_flight *f,
                                 const char *document,
                                 size_t document_len,
                                 const struct xreal_air_parsed_calibration *calib)
{
	char *doc = NULL;
	if (document != NULL) {
		doc = malloc(document_len);
		if (doc != NULL) {
			memcpy(doc, document, document_len);
		}
	}
	os_mutex_lock(&f->meta_lock);
	free(f->calib_document);
	f->calib_document = doc;
	f->calib_document_len = doc != NULL ? document_len : 0;
	f->calib = *calib;
	f->calib_valid = true;
	os_mutex_unlock(&f->meta_lock);
}

void
xreal_air_flight_push_imu(struct xreal_air_flight *f, const struct xreal_air_record_imu *imu)
{
	xreal_air_flight_ring_push(f->rings[S_IMU], imu);
}

void
xreal_air_flight_push_cam_frame(struct xreal_air_flight *f, const struct xreal_air_record_cam_frame *frame)
{
	xreal_air_flight_ring_push(f->rings[S_CAM], frame);
}

void
xreal_air_flight_push_stereo(struct xreal_air_flight *f,
                             int64_t euroc_ns,
                             struct xrt_frame *left,
                             struct xrt_frame *right)
{
	struct frame_slot old = {0};

	os_mutex_lock(&f->frames.lock);
	if (f->frames.capacity == 0) {
		os_mutex_unlock(&f->frames.lock);
		return;
	}
	struct frame_slot *slot = &f->frames.slots[f->frames.head % f->frames.capacity];
	old = *slot;
	U_ZERO(slot);
	slot->euroc_ns = euroc_ns;
	xrt_frame_reference(&slot->frames[0], left);
	xrt_frame_reference(&slot->frames[1], right);
	f->frames.head++;
	os_mutex_unlock(&f->frames.lock);

	// The oldest pair may be the last reference to its frames: free them outside the lock.
	release_slot(&old);
}

void
xreal_air_flight_push_slam_pose(struct xreal_air_flight *f, const struct xreal_air_flight_slam_pose *pose)
{
	xreal_air_flight_ring_push(f->rings[S_POSE], pose);
}

void
xreal_air_flight_set_timing_columns(struct xreal_air_flight *f, const char *columns_csv)
{
	os_mutex_lock(&f->meta_lock);
	(void)snprintf(f->timing_columns, sizeof(f->timing_columns), "%s", columns_csv != NULL ? columns_csv : "");
	os_mutex_unlock(&f->meta_lock);
}

void
xreal_air_flight_push_slam_timing(struct xreal_air_flight *f, const int64_t *timestamps, uint32_t count)
{
	struct timing_record r;
	U_ZERO(&r);
	r.count = count < MAX_TIMING ? count : MAX_TIMING;
	memcpy(r.ts, timestamps, r.count * sizeof(int64_t));
	xreal_air_flight_ring_push(f->rings[S_TIMING], &r);
}

void
xreal_air_flight_push_head(struct xreal_air_flight *f, const struct xreal_air_flight_head *head)
{
	xreal_air_flight_ring_push(f->rings[S_HEAD], head);
}

void
xreal_air_flight_event(struct xreal_air_flight *f, const char *type, const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	push_event_at(f, now_ns(), type, fmt, args);
	va_end(args);
}

const char *
xreal_air_flight_ctl_dir(const struct xreal_air_flight *f)
{
	return f->cfg.ctl_dir;
}

void
xreal_air_flight_destroy(struct xreal_air_flight **f_ptr)
{
	struct xreal_air_flight *f = *f_ptr;
	if (f == NULL) {
		return;
	}

	// Finishes a running capture on the way out.
	os_thread_helper_destroy(&f->oth);
	if (f->inotify_fd >= 0) {
		(void)close(f->inotify_fd);
	}
	if (f->owner_fd >= 0) {
		(void)close(f->owner_fd);
	}

	f->armed_frames_s = 0;
	frames_resize(f, 0);
	os_mutex_destroy(&f->frames.lock);
	os_mutex_destroy(&f->meta_lock);

	for (int s = 0; s < S_COUNT; s++) {
		xreal_air_flight_ring_destroy(&f->rings[s]);
	}
	free(f->calib_document);
	free(f);
	*f_ptr = NULL;
}
