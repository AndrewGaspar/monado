// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Bounded real-time thread priority on Linux, rtkit first.
 * @author Andrew Gaspar
 *
 * @ingroup aux_util
 */

#include "xrt/xrt_config_have.h"

#include "util/u_linux_realtime.h"

#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <linux/capability.h>

#ifdef XRT_HAVE_SYSTEMD
#include <systemd/sd-bus.h>
#endif

#define LOG_D(...) U_LOG_IFL_D(log_level, __VA_ARGS__)
#define LOG_W(...) U_LOG_IFL_W(log_level, __VA_ARGS__)

#define NAME_LENGTH 32

//! Length of the human readable reasons built up below.
#define REASON_LENGTH 256

//! Length of a D-Bus error, embedded in a reason.
#define BUS_ERROR_LENGTH 160

#define RTKIT_SERVICE_NAME "org.freedesktop.RealtimeKit1"
#define RTKIT_OBJECT_PATH "/org/freedesktop/RealtimeKit1"
#define RTKIT_INTERFACE "org.freedesktop.RealtimeKit1"

//! How long to wait for rtkit (including D-Bus activation of rtkit-daemon).
#define RTKIT_CALL_TIMEOUT_USEC (3 * 1000 * 1000)


/*
 *
 * Helper functions.
 *
 */

static const char *
policy_to_string(int policy)
{
	switch (policy) {
	case SCHED_FIFO: return "SCHED_FIFO";
	case SCHED_RR: return "SCHED_RR";
	case SCHED_OTHER: return "SCHED_OTHER(normal)";
	case SCHED_IDLE: return "SCHED_IDLE";
	case SCHED_BATCH: return "SCHED_BATCH";
	default: return "SCHED_<UNKNOWN>";
	}
}

static void
get_name(char *str, size_t count)
{
	assert(str != NULL);
	assert(count > 0);

	// First init.
	str[0] = '\0';

	// Get name of thread.
	pthread_getname_np(pthread_self(), str, count);

	if (str[0] == '\0') {
		snprintf(str, count, "tid(%i)", gettid());
	}
}

/*!
 * Read this thread's policy and priority straight from the kernel. glibc's
 * pthread_getschedparam() caches them, so it misses changes made by rtkit.
 */
static void
get_this_thread_sched(int *out_policy, int *out_priority)
{
	struct sched_param params = {0};

	// With pid 0 these act on the calling thread on Linux.
	int policy = sched_getscheduler(0);
	if (policy < 0 || sched_getparam(0, &params) != 0) {
		policy = SCHED_OTHER;
		params.sched_priority = 0;
	}

#ifdef SCHED_RESET_ON_FORK
	policy &= ~SCHED_RESET_ON_FORK;
#endif

	*out_policy = policy;
	*out_priority = params.sched_priority;
}

static bool
is_realtime_policy(int policy)
{
	return policy == SCHED_FIFO || policy == SCHED_RR;
}

static int
clamp_priority(int priority, int min, int max)
{
	if (priority < min) {
		return min;
	}
	if (priority > max) {
		return max;
	}
	return priority;
}

static bool
has_effective_cap_sys_nice(void)
{
	struct __user_cap_header_struct header = {
	    .version = _LINUX_CAPABILITY_VERSION_3,
	    .pid = 0,
	};
	struct __user_cap_data_struct data[_LINUX_CAPABILITY_U32S_3] = {0};

	if (syscall(SYS_capget, &header, data) != 0) {
		return false;
	}

	return (data[CAP_TO_INDEX(CAP_SYS_NICE)].effective & CAP_TO_MASK(CAP_SYS_NICE)) != 0;
}

static void
rlimit_to_string(rlim_t value, char *str, size_t count)
{
	if (value == RLIM_INFINITY) {
		snprintf(str, count, "unlimited");
	} else {
		snprintf(str, count, "%llu", (unsigned long long)value);
	}
}

#ifdef XRT_HAVE_SYSTEMD
/*!
 * Make sure the hard RLIMIT_RTTIME is set and at most @p max_usec, rtkit
 * refuses the request otherwise. A real-time thread of this process that runs
 * this long without blocking gets SIGXCPU/SIGKILL instead of starving the
 * machine. Lowering a hard limit can not be undone, which is fine: it only
 * ever matters for real-time threads.
 */
static bool
limit_rttime(enum u_logging_level log_level, rlim_t max_usec, rlim_t *out_usec, char *why, size_t why_count)
{
	struct rlimit rl = {0};
	if (getrlimit(RLIMIT_RTTIME, &rl) != 0) {
		snprintf(why, why_count, "getrlimit(RLIMIT_RTTIME): %s", strerror(errno));
		return false;
	}

	struct rlimit wanted = rl;
	if (wanted.rlim_max == RLIM_INFINITY || wanted.rlim_max > max_usec) {
		wanted.rlim_max = max_usec;
	}
	if (wanted.rlim_cur == RLIM_INFINITY || wanted.rlim_cur > wanted.rlim_max) {
		wanted.rlim_cur = wanted.rlim_max;
	}

	if (wanted.rlim_cur != rl.rlim_cur || wanted.rlim_max != rl.rlim_max) {
		if (setrlimit(RLIMIT_RTTIME, &wanted) != 0) {
			snprintf(why, why_count, "setrlimit(RLIMIT_RTTIME, %llu): %s",
			         (unsigned long long)wanted.rlim_max, strerror(errno));
			return false;
		}
		LOG_D("Set RLIMIT_RTTIME to %llu/%llu us (soft/hard) for rtkit.", (unsigned long long)wanted.rlim_cur,
		      (unsigned long long)wanted.rlim_max);
	}

	*out_usec = wanted.rlim_max;
	return true;
}

static void
describe_bus_error(const sd_bus_error *error, int ret, char *str, size_t count)
{
	if (sd_bus_error_is_set(error)) {
		snprintf(str, count, "%s: %s", error->name, error->message != NULL ? error->message : "");
	} else {
		snprintf(str, count, "%s", strerror(-ret));
	}
}

static bool
rtkit_make_thread_realtime_on_bus(enum u_logging_level log_level,
                                  sd_bus *bus,
                                  int priority,
                                  char *info,
                                  size_t info_count,
                                  char *why,
                                  size_t why_count)
{
	sd_bus_error error = SD_BUS_ERROR_NULL;
	char err_str[BUS_ERROR_LENGTH];
	int32_t max_priority = 0;
	int64_t rttime_usec_max = 0;
	int ret;

	ret = sd_bus_get_property_trivial(bus, RTKIT_SERVICE_NAME, RTKIT_OBJECT_PATH, RTKIT_INTERFACE,
	                                  "MaxRealtimePriority", &error, 'i', &max_priority);
	if (ret < 0) {
		describe_bus_error(&error, ret, err_str, sizeof(err_str));
		snprintf(why, why_count, "not available (%s)", err_str);
		sd_bus_error_free(&error);
		return false;
	}

	ret = sd_bus_get_property_trivial(bus, RTKIT_SERVICE_NAME, RTKIT_OBJECT_PATH, RTKIT_INTERFACE, "RTTimeUSecMax",
	                                  &error, 'x', &rttime_usec_max);
	if (ret < 0) {
		describe_bus_error(&error, ret, err_str, sizeof(err_str));
		snprintf(why, why_count, "could not read RTTimeUSecMax (%s)", err_str);
		sd_bus_error_free(&error);
		return false;
	}

	LOG_D("rtkit: MaxRealtimePriority %i, RTTimeUSecMax %" PRId64 " us.", max_priority, rttime_usec_max);

	if (max_priority < 1 || rttime_usec_max <= 0) {
		snprintf(why, why_count, "MaxRealtimePriority is %i, RTTimeUSecMax is %" PRId64 " us", max_priority,
		         rttime_usec_max);
		return false;
	}

	rlim_t rttime_usec = 0;
	if (!limit_rttime(log_level, (rlim_t)rttime_usec_max, &rttime_usec, why, why_count)) {
		return false;
	}

	uint32_t rtkit_priority = (uint32_t)clamp_priority(priority, 1, max_priority);

	ret =
	    sd_bus_call_method(bus, RTKIT_SERVICE_NAME, RTKIT_OBJECT_PATH, RTKIT_INTERFACE, "MakeThreadRealtimeWithPID",
	                       &error, NULL, "ttu", (uint64_t)getpid(), (uint64_t)gettid(), rtkit_priority);
	if (ret < 0) {
		describe_bus_error(&error, ret, err_str, sizeof(err_str));
		snprintf(why, why_count, "refused priority %u (%s; RLIMIT_RTTIME %llu us)", rtkit_priority, err_str,
		         (unsigned long long)rttime_usec);
		sd_bus_error_free(&error);
		return false;
	}

	snprintf(info, info_count, "rtkit, MaxRealtimePriority %i, RLIMIT_RTTIME %llu us", max_priority,
	         (unsigned long long)rttime_usec);
	return true;
}
#endif

/*!
 * Ask rtkit to make this thread real-time at @p priority (clamped to rtkit's
 * maximum). On success @p info says how, on failure @p why says why not.
 */
static bool
rtkit_make_thread_realtime(
    enum u_logging_level log_level, int priority, char *info, size_t info_count, char *why, size_t why_count)
{
#ifdef XRT_HAVE_SYSTEMD
	sd_bus *bus = NULL;

	int ret = sd_bus_open_system(&bus);
	if (ret < 0) {
		snprintf(why, why_count, "no system bus (%s)", strerror(-ret));
		return false;
	}

	sd_bus_set_method_call_timeout(bus, RTKIT_CALL_TIMEOUT_USEC);

	bool ok = rtkit_make_thread_realtime_on_bus(log_level, bus, priority, info, info_count, why, why_count);

	sd_bus_flush_close_unref(bus);

	return ok;
#else
	(void)log_level;
	(void)priority;
	(void)info;
	(void)info_count;
	snprintf(why, why_count, "not built in (needs libsystemd, XRT_HAVE_SYSTEMD)");
	return false;
#endif
}

/*!
 * Make this thread SCHED_FIFO at @p priority with pthread_setschedparam(),
 * on failure @p why says why not, including the likely cause of EPERM.
 */
static bool
sched_make_thread_realtime(int priority, char *why, size_t why_count)
{
	struct sched_param params = {
	    .sched_priority = priority,
	};

	int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
	if (ret == 0) {
		return true;
	}

	if (ret != EPERM) {
		snprintf(why, why_count, "%s", strerror(ret));
		return false;
	}

	struct rlimit rl = {0};
	char soft[32] = "?";
	if (getrlimit(RLIMIT_RTPRIO, &rl) == 0) {
		rlimit_to_string(rl.rlim_cur, soft, sizeof(soft));
	}

	snprintf(why, why_count, "%s (RLIMIT_RTPRIO=%s, CAP_SYS_NICE %s)", strerror(ret), soft,
	         has_effective_cap_sys_nice() ? "present" : "absent");

	return false;
}



/*
 *
 * 'Exported' functions.
 *
 */

bool
u_linux_try_to_set_bounded_realtime_priority_on_thread(enum u_logging_level log_level, const char *name, int priority)
{
	char rtkit_info[REASON_LENGTH] = "";
	char rtkit_why[REASON_LENGTH] = "";
	char sched_why[REASON_LENGTH] = "";
	char str[NAME_LENGTH];
	int policy = SCHED_OTHER;
	int obtained = 0;

	// Always have some name.
	if (name == NULL) {
		get_name(str, ARRAY_SIZE(str));
		name = str;
	}

	get_this_thread_sched(&policy, &obtained);
	LOG_D("Thread '%s': asking for real-time priority %i, currently '%s' priority %i.", name, priority,
	      policy_to_string(policy), obtained);

	// rtkit first: works for any desktop session user, no capabilities needed.
	if (rtkit_make_thread_realtime(log_level, priority, rtkit_info, sizeof(rtkit_info), rtkit_why,
	                               sizeof(rtkit_why))) {
		get_this_thread_sched(&policy, &obtained);
		LOG_W("Thread '%s': real-time priority obtained: '%s' priority %i (asked %i; via %s).", name,
		      policy_to_string(policy), obtained, priority, rtkit_info);
		return is_realtime_policy(policy);
	}

	LOG_D("Thread '%s': rtkit: %s, trying pthread_setschedparam.", name, rtkit_why);

	// Then directly, needs CAP_SYS_NICE or a high enough RLIMIT_RTPRIO.
	int sched_priority =
	    clamp_priority(priority, sched_get_priority_min(SCHED_FIFO), sched_get_priority_max(SCHED_FIFO));
	if (sched_make_thread_realtime(sched_priority, sched_why, sizeof(sched_why))) {
		get_this_thread_sched(&policy, &obtained);
		LOG_W(
		    "Thread '%s': real-time priority obtained: '%s' priority %i (asked %i; via pthread_setschedparam, "
		    "rtkit: %s).",
		    name, policy_to_string(policy), obtained, priority, rtkit_why);
		return is_realtime_policy(policy);
	}

	get_this_thread_sched(&policy, &obtained);
	LOG_W(
	    "Thread '%s': real-time priority %i NOT obtained, staying '%s' priority %i. rtkit: %s; "
	    "pthread_setschedparam(SCHED_FIFO, %i): %s.",
	    name, priority, policy_to_string(policy), obtained, rtkit_why, sched_priority, sched_why);

	return false;
}
