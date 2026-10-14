// Copyright 2026, Andrew Gaspar.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Persist the learned m_imu_3dof gyro bias of Xreal Air glasses across sessions.
 * @ingroup drv_xreal_air
 *
 * The estimator needs ~30 s of stillness before it learns anything, which normal worn use rarely
 * offers, so every session used to start with the full uncorrected yaw drift. The residual bias on
 * the yaw axis is stable across days, so it is saved per device and seeded at startup with an
 * uncertainty grown by the time and temperature change since, the still blocks then refine it.
 *
 * The sensor thread only ever copies a snapshot into a slot and signals; logging and the atomic
 * file write happen on a small writer thread.
 */

#include "os/os_threading.h"
#include "os/os_time.h"

#include "util/u_file.h"
#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_time.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"
#include "math/m_vec3.h"

#include "xreal_air_gyro_bias_store.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define STORE_I(s, ...) U_LOG_IFL_I((s)->log_level, "Xreal Air: " __VA_ARGS__)
#define STORE_W(s, ...) U_LOG_IFL_W((s)->log_level, "Xreal Air: " __VA_ARGS__)

//! At most this often a snapshot with newly learned blocks goes to the writer (log + maybe save).
#define POST_PERIOD_NS ((int64_t)30 * U_TIME_1S_IN_NS)
//! Without anything learned, a status line this often.
#define HEARTBEAT_PERIOD_NS ((int64_t)120 * U_TIME_1S_IN_NS)

//! A save is worth it when the bias moved this much (deg/s) ...
#define SAVE_MIN_BIAS_CHANGE_DEG_S (0.002f)
//! ... or the uncertainty shrank below this fraction of the saved one ...
#define SAVE_SD_SHRINK (0.75f)
//! ... or learning went on for this long since (keeps the save time, and so the seed sd, fresh).
#define SAVE_REFRESH_S (300)

struct snapshot
{
	struct m_imu_3dof_gyro_bias_record rec;
	uint32_t commits;
	uint32_t rejects_sample;
	uint32_t rejects_block;
	uint32_t rejects_yaw;
	double session_learned_s;
	bool seeded;
};

struct xreal_air_gyro_bias_store
{
	char path[1024];
	char device[64];
	enum u_logging_level log_level;

	bool loaded_valid;
	struct m_imu_3dof_gyro_bias_record loaded;
	struct m_imu_3dof_gyro_bias_params params;

	//! Sensor thread only.
	struct
	{
		int64_t posted_ns;
		uint32_t posted_commits;
	} tick;

	//! Protects @ref pending.
	struct os_thread_helper oth;
	bool pending_valid;
	struct snapshot pending;

	//! Writer thread only (and destroy once it stopped).
	struct
	{
		bool saved_valid;
		uint32_t saved_commits;
		struct m_imu_3dof_gyro_bias_record saved;
		uint32_t logged_commits;
	} writer;
};

static void
sanitize_id(const char *in, char *out, size_t size)
{
	size_t n = 0;
	for (const char *c = in; *c != '\0' && n + 1 < size; c++) {
		bool ok = (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') ||
		          *c == '-' || *c == '_';
		out[n++] = ok ? *c : '_';
	}
	out[n] = '\0';
	if (n == 0) {
		snprintf(out, size, "unknown");
	}
}

static int64_t
now_unix_s(void)
{
	return (int64_t)(os_realtime_get_ns() / U_TIME_1S_IN_NS);
}

static void
take_snapshot(const struct xreal_air_gyro_bias_store *s,
              const struct m_imu_3dof *fusion,
              float temp_c,
              struct snapshot *out)
{
	U_ZERO(out);
	m_imu_3dof_gyro_bias_get_record(fusion, &out->rec);
	snprintf(out->rec.device, sizeof(out->rec.device), "%s", s->device);
	out->rec.temp_c = temp_c;
	out->session_learned_s = out->rec.learned_s;
	if (s->loaded_valid) {
		out->rec.learned_s += s->loaded.learned_s;
	}
	out->commits = fusion->bias_est.commits;
	out->rejects_sample = fusion->bias_est.rejects_sample;
	out->rejects_block = fusion->bias_est.rejects_block;
	out->rejects_yaw = fusion->bias_est.rejects_yaw;
	out->seeded = fusion->bias_est.seeded;
}

static bool
save(struct xreal_air_gyro_bias_store *s, struct snapshot *snap)
{
	snap->rec.saved_unix_s = now_unix_s();

	char buf[1024];
	int n = m_imu_3dof_gyro_bias_record_format(&snap->rec, buf, sizeof(buf));
	if (n <= 0 || n >= (int)sizeof(buf)) {
		return false;
	}

	int ret = u_file_write_atomic(s->path, buf, (size_t)n);
	if (ret < 0) {
		STORE_W(s, "gyro bias: could not save to '%s': %s", s->path, strerror(-ret));
		return false;
	}

	s->writer.saved_valid = true;
	s->writer.saved_commits = snap->commits;
	s->writer.saved = snap->rec;

	const struct xrt_vec3 b = snap->rec.bias_deg_s;
	STORE_I(s, "gyro bias: saved [%.4f %.4f %.4f] deg/s, sd %.4f deg/s, %.1f C to '%s'", (double)b.x, (double)b.y,
	        (double)b.z, (double)snap->rec.sd_deg_s, (double)snap->rec.temp_c, s->path);
	return true;
}

static bool
worth_saving(const struct xreal_air_gyro_bias_store *s, const struct snapshot *snap)
{
	if (snap->commits == s->writer.saved_commits) {
		return false; // Nothing learned since.
	}
	if (!s->writer.saved_valid) {
		return true;
	}
	const struct m_imu_3dof_gyro_bias_record *old = &s->writer.saved;
	if (m_vec3_len(m_vec3_sub(snap->rec.bias_deg_s, old->bias_deg_s)) >= SAVE_MIN_BIAS_CHANGE_DEG_S) {
		return true;
	}
	if (snap->rec.sd_deg_s < old->sd_deg_s * SAVE_SD_SHRINK) {
		return true;
	}
	return now_unix_s() - old->saved_unix_s >= SAVE_REFRESH_S;
}

static void
process(struct xreal_air_gyro_bias_store *s, struct snapshot *snap)
{
	const struct xrt_vec3 b = snap->rec.bias_deg_s;

	if (snap->commits != s->writer.logged_commits) {
		STORE_I(s,
		        "gyro bias: %u blocks learned (%.0f s this session, %.0f s total), bias [%.4f %.4f %.4f] "
		        "deg/s, sd %.4f deg/s, %.1f C%s; rejects %u sample, %u block, %u yaw",
		        snap->commits, snap->session_learned_s, snap->rec.learned_s, (double)b.x, (double)b.y,
		        (double)b.z, (double)snap->rec.sd_deg_s, (double)snap->rec.temp_c,
		        snap->seeded ? ", seeded" : "", snap->rejects_sample, snap->rejects_block, snap->rejects_yaw);
		s->writer.logged_commits = snap->commits;
	} else {
		STORE_I(s,
		        "gyro bias: nothing new learned (%u blocks so far), bias [%.4f %.4f %.4f] deg/s, sd %.4f "
		        "deg/s, %.1f C%s; rejects %u sample, %u block, %u yaw",
		        snap->commits, (double)b.x, (double)b.y, (double)b.z, (double)snap->rec.sd_deg_s,
		        (double)snap->rec.temp_c, snap->seeded ? ", seeded" : "", snap->rejects_sample,
		        snap->rejects_block, snap->rejects_yaw);
	}

	if (worth_saving(s, snap)) {
		save(s, snap);
	}
}

static void *
writer_thread(void *ptr)
{
	struct xreal_air_gyro_bias_store *s = (struct xreal_air_gyro_bias_store *)ptr;

	os_thread_helper_name(&s->oth, "Xreal Air bias");

	os_thread_helper_lock(&s->oth);
	while (os_thread_helper_is_running_locked(&s->oth)) {
		if (!s->pending_valid) {
			os_thread_helper_wait_locked(&s->oth);
			continue;
		}

		struct snapshot snap = s->pending;
		s->pending_valid = false;

		// The disk only ever sees this thread, and without the lock held.
		os_thread_helper_unlock(&s->oth);
		process(s, &snap);
		os_thread_helper_lock(&s->oth);
	}
	os_thread_helper_unlock(&s->oth);

	return NULL;
}

static void
load(struct xreal_air_gyro_bias_store *s)
{
	size_t size = 0;
	char *content = u_file_read_content_from_path(s->path, &size);
	if (content == NULL) {
		STORE_I(s, "gyro bias: nothing saved at '%s' yet, learning from scratch", s->path);
		return;
	}

	char err[160] = "";
	struct m_imu_3dof_gyro_bias_record rec;
	bool ok = m_imu_3dof_gyro_bias_record_parse(content, &s->params, &rec, err, sizeof(err));
	free(content);

	if (!ok) {
		STORE_W(s, "gyro bias: ignoring '%s': %s", s->path, err);
		return;
	}
	if (strcmp(rec.device, s->device) != 0) {
		STORE_W(s, "gyro bias: ignoring '%s': saved for device '%s', not '%s'", s->path, rec.device, s->device);
		return;
	}

	s->loaded = rec;
	s->loaded_valid = true;
	// Matches what is on disk: nothing to save until something new is learned.
	s->writer.saved_valid = true;
	s->writer.saved = rec;

	STORE_I(s,
	        "gyro bias: loaded [%.4f %.4f %.4f] deg/s, sd %.4f deg/s, %.1f C, saved %" PRId64 " s ago, from '%s'",
	        (double)rec.bias_deg_s.x, (double)rec.bias_deg_s.y, (double)rec.bias_deg_s.z, (double)rec.sd_deg_s,
	        (double)rec.temp_c, now_unix_s() - rec.saved_unix_s, s->path);
}

struct xreal_air_gyro_bias_store *
xreal_air_gyro_bias_store_create(const char *device_id,
                                 const struct m_imu_3dof_gyro_bias_params *params,
                                 enum u_logging_level log_level)
{
	struct xreal_air_gyro_bias_store *s = U_TYPED_CALLOC(struct xreal_air_gyro_bias_store);
	s->log_level = log_level;
	s->params = *params;
	snprintf(s->device, sizeof(s->device), "%s", device_id != NULL ? device_id : "");

	char id[64];
	char name[128];
	sanitize_id(s->device, id, sizeof(id));
	snprintf(name, sizeof(name), "xreal_air_gyro_bias_%s.txt", id);

	int n = u_file_get_path_in_state_dir(name, s->path, sizeof(s->path));
	if (n <= 0 || n >= (int)sizeof(s->path)) {
		STORE_W(s, "gyro bias: no state directory (XDG_STATE_HOME/HOME), not persisting");
		free(s);
		return NULL;
	}

	load(s);

	if (os_thread_helper_init(&s->oth) != 0) {
		free(s);
		return NULL;
	}
	if (os_thread_helper_start(&s->oth, writer_thread, s) != 0) {
		STORE_W(s, "gyro bias: could not start the writer thread, not persisting");
		os_thread_helper_destroy(&s->oth);
		free(s);
		return NULL;
	}

	return s;
}

bool
xreal_air_gyro_bias_store_get_loaded(const struct xreal_air_gyro_bias_store *s, struct m_imu_3dof_gyro_bias_record *out)
{
	if (s == NULL || !s->loaded_valid) {
		return false;
	}
	*out = s->loaded;
	return true;
}

void
xreal_air_gyro_bias_store_seed(struct xreal_air_gyro_bias_store *s, struct m_imu_3dof *fusion, float temp_c)
{
	if (s == NULL || !s->loaded_valid) {
		return;
	}

	const struct m_imu_3dof_gyro_bias_record *rec = &s->loaded;
	float sd = m_imu_3dof_gyro_bias_record_seed_sd(rec, &fusion->bias_est.params, now_unix_s(), temp_c);
	struct xrt_vec3 bias = {
	    (float)DEG_TO_RAD((double)rec->bias_deg_s.x),
	    (float)DEG_TO_RAD((double)rec->bias_deg_s.y),
	    (float)DEG_TO_RAD((double)rec->bias_deg_s.z),
	};
	m_imu_3dof_gyro_bias_seed(fusion, &bias, sd);

	STORE_I(s, "gyro bias: seeded [%.4f %.4f %.4f] deg/s, sd %.4f deg/s (saved %.4f at %.1f C, now %.1f C)",
	        (double)rec->bias_deg_s.x, (double)rec->bias_deg_s.y, (double)rec->bias_deg_s.z, (double)sd,
	        (double)rec->sd_deg_s, (double)rec->temp_c, (double)temp_c);
}

void
xreal_air_gyro_bias_store_tick(struct xreal_air_gyro_bias_store *s,
                               const struct m_imu_3dof *fusion,
                               float temp_c,
                               int64_t now_ns)
{
	if (s == NULL) {
		return;
	}

	if (s->tick.posted_ns == 0) {
		s->tick.posted_ns = now_ns;
		return;
	}

	const uint32_t commits = fusion->bias_est.commits;
	const int64_t since = now_ns - s->tick.posted_ns;
	const bool learned = commits != s->tick.posted_commits;

	// The very first learned block right away, then rate-limited.
	bool post = learned ? (s->tick.posted_commits == 0 || since >= POST_PERIOD_NS) : since >= HEARTBEAT_PERIOD_NS;
	if (!post) {
		return;
	}

	s->tick.posted_ns = now_ns;
	s->tick.posted_commits = commits;

	struct snapshot snap;
	take_snapshot(s, fusion, temp_c, &snap);

	os_thread_helper_lock(&s->oth);
	s->pending = snap;
	s->pending_valid = true;
	os_thread_helper_signal_locked(&s->oth);
	os_thread_helper_unlock(&s->oth);
}

void
xreal_air_gyro_bias_store_destroy(struct xreal_air_gyro_bias_store **s_ptr,
                                  const struct m_imu_3dof *fusion,
                                  float temp_c)
{
	struct xreal_air_gyro_bias_store *s = *s_ptr;
	if (s == NULL) {
		return;
	}
	*s_ptr = NULL;

	os_thread_helper_destroy(&s->oth);

	if (fusion != NULL) {
		struct snapshot snap;
		take_snapshot(s, fusion, temp_c, &snap);
		if (snap.commits != s->writer.saved_commits) {
			save(s, &snap);
		}
	}

	free(s);
}

const char *
xreal_air_gyro_bias_store_path(const struct xreal_air_gyro_bias_store *s)
{
	return s->path;
}
