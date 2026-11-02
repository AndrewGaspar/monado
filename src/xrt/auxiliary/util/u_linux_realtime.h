// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Bounded real-time thread priority on Linux, rtkit first.
 * @author Andrew Gaspar
 *
 * Separate from @ref u_linux.h because asking rtkit needs sd-bus (libsystemd),
 * which should not be pulled into everything that links aux_util (such as the
 * OpenXR runtime library loaded into every application).
 *
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "util/u_logging.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Try to make this thread real-time at a bounded priority, without needing
 * CAP_SYS_NICE.
 *
 * Asks rtkit (org.freedesktop.RealtimeKit1 on the system bus) first. Before
 * the request the process's RLIMIT_RTTIME is lowered to at most rtkit's
 * RTTimeUSecMax, as rtkit requires, and @p priority is clamped to rtkit's
 * MaxRealtimePriority. rtkit makes the thread SCHED_RR | SCHED_RESET_ON_FORK.
 * When rtkit is not built in (needs XRT_HAVE_SYSTEMD), not running or refuses,
 * falls back to pthread_setschedparam() with SCHED_FIFO at @p priority, which
 * needs CAP_SYS_NICE or a high enough RLIMIT_RTPRIO.
 *
 * The outcome is logged exactly once at warn level (if @p log_level allows):
 * the policy and priority obtained and how, or why the thread stays at its
 * normal policy. Details go to the debug level.
 *
 * Unlike u_linux_try_to_set_realtime_priority_on_thread() this never asks for
 * the platform's maximum priority.
 *
 * @param log_level Logging level to control chattiness.
 * @param name      Thread name to be used in logging, can be NULL.
 * @param priority  Wanted real-time priority, clamped to the policy's range.
 *
 * @return true if the thread is now SCHED_FIFO or SCHED_RR.
 *
 * @ingroup aux_util
 */
bool
u_linux_try_to_set_bounded_realtime_priority_on_thread(enum u_logging_level log_level, const char *name, int priority);


#ifdef __cplusplus
}
#endif
