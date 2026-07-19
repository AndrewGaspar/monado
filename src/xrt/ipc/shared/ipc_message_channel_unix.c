// Copyright 2020-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IPC message channel functions for UNIX platforms.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup ipc_shared
 */

#include "xrt/xrt_config_os.h"

#ifdef XRT_OS_WINDOWS
#error "This file shouldn't be compiled on Windows!"
#endif

#include "util/u_logging.h"
#include "util/u_pretty_print.h"

#include "os/os_time.h"

#include "shared/ipc_protocol.h"
#include "shared/ipc_message_channel.h"

#include <errno.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <poll.h>


/*
 *
 * Logging
 *
 */

#define IPC_TRACE(d, ...) U_LOG_IFL_T(d->log_level, __VA_ARGS__)
#define IPC_DEBUG(d, ...) U_LOG_IFL_D(d->log_level, __VA_ARGS__)
#define IPC_INFO(d, ...) U_LOG_IFL_I(d->log_level, __VA_ARGS__)
#define IPC_WARN(d, ...) U_LOG_IFL_W(d->log_level, __VA_ARGS__)
#define IPC_ERROR(d, ...) U_LOG_IFL_E(d->log_level, __VA_ARGS__)


/*
 *
 * Structs and defines.
 *
 */

union imcontrol_buf {
	uint8_t buf[512];
	struct cmsghdr align;
};


/*
 *
 * Bounded receive helper (client-side deadlock guard, HypXRland task #89).
 *
 */

/*!
 * Slice length (ms) used when polling an unbounded (wait-class) receive, so we
 * loop back often enough to notice a dead socket even while legitimately
 * blocking forever.
 */
#define IPC_UNBOUNDED_POLL_SLICE_MS 1000

/*!
 * Wait for @p imc->ipc_handle to become readable before we call recvmsg().
 *
 * Behavior is driven entirely by fields on the channel, all zero on the server
 * (so the server keeps its historical infinite-block semantics):
 *
 * - If the channel is already marked failed, fail immediately.
 * - If timeout_ms <= 0 (disabled) or waiting_unbounded is set, poll forever in
 *   slices, returning success as soon as the fd is readable/hung-up (recvmsg
 *   then surfaces the data or the EOF). This detects a dead service even for
 *   the wait-class calls that may legitimately block for a long time.
 * - Otherwise wait at most timeout_ms across the whole call; on expiry, log
 *   loudly to stderr, mark the connection dead, and fail so the caller (and
 *   every subsequent call) fails fast instead of freezing forever.
 */
static xrt_result_t
ipc_channel_wait_readable(struct ipc_message_channel *imc)
{
	if (imc->failed) {
		return XRT_ERROR_IPC_FAILURE;
	}

	const bool unbounded = imc->waiting_unbounded || imc->timeout_ms <= 0;

	// Absolute deadline for the bounded case.
	const uint64_t start_ns = unbounded ? 0 : os_monotonic_get_ns();
	const uint64_t budget_ns = unbounded ? 0 : (uint64_t)imc->timeout_ms * (uint64_t)U_TIME_1MS_IN_NS;

	for (;;) {
		int wait_ms;
		if (unbounded) {
			wait_ms = IPC_UNBOUNDED_POLL_SLICE_MS;
		} else {
			uint64_t now_ns = os_monotonic_get_ns();
			uint64_t elapsed_ns = now_ns - start_ns;
			if (elapsed_ns >= budget_ns) {
				wait_ms = 0;
			} else {
				uint64_t remain_ms = (budget_ns - elapsed_ns) / U_TIME_1MS_IN_NS;
				wait_ms = remain_ms > INT32_MAX ? INT32_MAX : (int)remain_ms;
			}
		}

		struct pollfd pfd = {
		    .fd = imc->ipc_handle,
		    .events = POLLIN,
		    .revents = 0,
		};

		int pret = poll(&pfd, 1, wait_ms);
		if (pret > 0) {
			// Readable, or POLLHUP/POLLERR — let recvmsg surface it.
			return XRT_SUCCESS;
		}
		if (pret < 0) {
			if (errno == EINTR) {
				// Interrupted; re-evaluate remaining budget.
				continue;
			}
			IPC_ERROR(imc, "poll(%i) failed: '%s'! Marking connection dead.", (int)imc->ipc_handle,
			          strerror(errno));
			imc->failed = true;
			return XRT_ERROR_IPC_FAILURE;
		}

		// pret == 0: this poll slice expired.
		if (unbounded) {
			// A live-but-idle socket just times out with no revents; a
			// dead one becomes readable (POLLHUP) and takes the pret>0
			// path above. So keep waiting.
			continue;
		}

		// Bounded receive whose whole budget is spent: the service never
		// replied. Fail loudly and poison the connection so a late reply
		// can never be mis-delivered to the next call.
		imc->failed = true;

		const char *name = imc->cmd_name != NULL ? imc->cmd_name : "<unknown>";
		IPC_ERROR(imc,
		          "IPC TIMEOUT: monado-service did not reply to command '%s' within %i ms; "
		          "marking connection dead. (task #89 deadlock guard; tune/disable with "
		          "XRT_IPC_CLIENT_TIMEOUT_MS)",
		          name, imc->timeout_ms);
		// Also emit directly to stderr, unconditionally: this is the sink
		// that survives a compositor freeze and lands in the journal.
		fprintf(stderr,
		        "[monado-ipc] TIMEOUT: no reply from monado-service for IPC command '%s' after %d ms; "
		        "connection marked dead (XRT_IPC_CLIENT_TIMEOUT_MS)\n",
		        name, imc->timeout_ms);
		fflush(stderr);

		return XRT_ERROR_IPC_FAILURE;
	}
}


/*
 *
 * 'Exported' functions.
 *
 */

void
ipc_message_channel_close(struct ipc_message_channel *imc)
{
	if (imc->ipc_handle < 0) {
		return;
	}
	close(imc->ipc_handle);
	imc->ipc_handle = -1;
}

xrt_result_t
ipc_send(struct ipc_message_channel *imc, const void *data, size_t size)
{
	// Connection was poisoned by an earlier timeout: fail fast so we never
	// push a new command onto a stream that may still hold a stale reply.
	if (imc->failed) {
		return XRT_ERROR_IPC_FAILURE;
	}

	struct msghdr msg = {0};
	struct iovec iov = {0};

	iov.iov_base = (void *)data;
	iov.iov_len = size;

	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_flags = 0;

	ssize_t ret = sendmsg(imc->ipc_handle, &msg, MSG_NOSIGNAL);
	if (ret < 0) {
		int code = errno;
		IPC_ERROR(imc, "sendmsg(%i) failed: '%i' '%s'!", imc->ipc_handle, code, strerror(code));
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ipc_receive(struct ipc_message_channel *imc, void *out_data, size_t size)
{
	// Bounded wait: fail (and poison the connection) if the service does not
	// answer in time, instead of blocking recvmsg() forever.
	xrt_result_t wret = ipc_channel_wait_readable(imc);
	if (wret != XRT_SUCCESS) {
		return wret;
	}

	// wait for the response
	struct iovec iov = {0};
	struct msghdr msg = {0};

	iov.iov_base = out_data;
	iov.iov_len = size;

	msg.msg_name = 0;
	msg.msg_namelen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_flags = 0;

	ssize_t len = recvmsg(imc->ipc_handle, &msg, MSG_NOSIGNAL);

	if (len < 0) {
		int code = errno;
		IPC_ERROR(imc, "recvmsg(%i) failed: '%i' '%s'!", (int)imc->ipc_handle, code, strerror(code));
		return XRT_ERROR_IPC_FAILURE;
	}

	if ((size_t)len != size) {
		IPC_ERROR(imc, "recvmsg(%i) failed: wrong size '%i', expected '%i'!", (int)imc->ipc_handle, (int)len,
		          (int)size);
		return XRT_ERROR_IPC_FAILURE;
	}

	return XRT_SUCCESS;
}

xrt_result_t
ipc_receive_fds(struct ipc_message_channel *imc, void *out_data, size_t size, int *out_handles, uint32_t handle_count)
{
	assert(imc != NULL);
	assert(out_data != NULL);
	assert(size != 0);
	assert(out_handles != NULL);
	assert(handle_count != 0);

	// Bounded wait (same guard as ipc_receive): the fd-passing reply path is
	// exactly what the OpenXR session bring-up funnels through.
	xrt_result_t wret = ipc_channel_wait_readable(imc);
	if (wret != XRT_SUCCESS) {
		return wret;
	}

	union imcontrol_buf u;
	const size_t fds_size = sizeof(int) * handle_count;
	const size_t cmsg_size = CMSG_SPACE(fds_size);
	memset(u.buf, 0, cmsg_size);

	struct iovec iov = {0};
	iov.iov_base = out_data;
	iov.iov_len = size;

	struct msghdr msg = {0};
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = u.buf;
	msg.msg_controllen = cmsg_size;

	ssize_t len = recvmsg(imc->ipc_handle, &msg, MSG_NOSIGNAL);
	if (len < 0) {
		IPC_ERROR(imc, "recvmsg(%i) failed: '%s'!", imc->ipc_handle, strerror(errno));
		return XRT_ERROR_IPC_FAILURE;
	}

	if (len == 0) {
		IPC_ERROR(imc, "recvmsg(%i) failed: no data!", imc->ipc_handle);
		return XRT_ERROR_IPC_FAILURE;
	}

	// Did the other side actually send file descriptors.
	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	if (cmsg == NULL) {
		return XRT_SUCCESS;
	}

	memcpy(out_handles, (int *)CMSG_DATA(cmsg), fds_size);

	return XRT_SUCCESS;
}

xrt_result_t
ipc_send_fds(struct ipc_message_channel *imc, const void *data, size_t size, const int *handles, uint32_t handle_count)
{
	assert(imc != NULL);
	assert(data != NULL);
	assert(size != 0);
	assert(handles != NULL);

	const size_t fds_size = sizeof(int) * handle_count;

	union imcontrol_buf u = {0};
	size_t cmsg_size = CMSG_SPACE(fds_size);

	struct iovec iov = {0};
	iov.iov_base = (void *)data;
	iov.iov_len = size;

	struct msghdr msg = {0};
	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_flags = 0;
	msg.msg_control = u.buf;
	msg.msg_controllen = cmsg_size;

	struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
	cmsg->cmsg_level = SOL_SOCKET;
	cmsg->cmsg_type = SCM_RIGHTS;
	cmsg->cmsg_len = CMSG_LEN(fds_size);

	memcpy(CMSG_DATA(cmsg), handles, fds_size);

	ssize_t ret = sendmsg(imc->ipc_handle, &msg, MSG_NOSIGNAL);
	if (ret >= 0) {
		return XRT_SUCCESS;
	}

	/*
	 * Error path.
	 */

	struct u_pp_sink_stack_only sink;
	u_pp_delegate_t dg = u_pp_sink_stack_only_init(&sink);

	u_pp(dg, "sendmsg(%i) failed: count: %u, error: '%i' '%s'!", imc->ipc_handle, handle_count, errno,
	     strerror(errno));

	for (uint32_t i = 0; i < handle_count; i++) {
		u_pp(dg, "\n\tfd #%i: %i", i, handles[i]);
	}

	IPC_ERROR(imc, "%s", sink.buffer);

	return XRT_ERROR_IPC_FAILURE;
}

xrt_result_t
ipc_receive_handles_shmem(struct ipc_message_channel *imc,
                          void *out_data,
                          size_t size,
                          xrt_shmem_handle_t *out_handles,
                          uint32_t handle_count)
{
	return ipc_receive_fds(imc, out_data, size, out_handles, handle_count);
}

xrt_result_t
ipc_send_handles_shmem(struct ipc_message_channel *imc,
                       const void *data,
                       size_t size,
                       const xrt_shmem_handle_t *handles,
                       uint32_t handle_count)
{
	return ipc_send_fds(imc, data, size, handles, handle_count);
}


/*
 *
 * AHardwareBuffer graphics buffer functions.
 *
 */

#if defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER)

#include <android/hardware_buffer.h>


xrt_result_t
ipc_receive_handles_graphics_buffer(struct ipc_message_channel *imc,
                                    void *out_data,
                                    size_t size,
                                    xrt_graphics_buffer_handle_t *out_handles,
                                    uint32_t handle_count)
{
	xrt_result_t result = ipc_receive(imc, out_data, size);
	if (result != XRT_SUCCESS) {
		return result;
	}
	bool failed = false;
	for (uint32_t i = 0; i < handle_count; ++i) {
		int err = AHardwareBuffer_recvHandleFromUnixSocket(imc->ipc_handle, &(out_handles[i]));
		if (err != 0) {
			failed = true;
		}
	}
	return failed ? XRT_ERROR_IPC_FAILURE : XRT_SUCCESS;
}


xrt_result_t
ipc_send_handles_graphics_buffer(struct ipc_message_channel *imc,
                                 const void *data,
                                 size_t size,
                                 const xrt_graphics_buffer_handle_t *handles,
                                 uint32_t handle_count)
{
	xrt_result_t result = ipc_send(imc, data, size);
	if (result != XRT_SUCCESS) {
		return result;
	}
	bool failed = false;
	for (uint32_t i = 0; i < handle_count; ++i) {
		int err = AHardwareBuffer_sendHandleToUnixSocket(handles[i], imc->ipc_handle);
		if (err != 0) {
			failed = true;
		}
	}
	return failed ? XRT_ERROR_IPC_FAILURE : XRT_SUCCESS;
}


/*
 *
 * FD graphics buffer functions.
 *
 */

#elif defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_FD)

xrt_result_t
ipc_receive_handles_graphics_buffer(struct ipc_message_channel *imc,
                                    void *out_data,
                                    size_t size,
                                    xrt_graphics_buffer_handle_t *out_handles,
                                    uint32_t handle_count)
{
	return ipc_receive_fds(imc, out_data, size, out_handles, handle_count);
}

xrt_result_t
ipc_send_handles_graphics_buffer(struct ipc_message_channel *imc,
                                 const void *data,
                                 size_t size,
                                 const xrt_graphics_buffer_handle_t *handles,
                                 uint32_t handle_count)
{
	return ipc_send_fds(imc, data, size, handles, handle_count);
}

#else
#error "Need port to transport these graphics buffers"
#endif


/*
 *
 * FD graphics sync functions.
 *
 */

#if defined(XRT_GRAPHICS_SYNC_HANDLE_IS_FD)

xrt_result_t
ipc_receive_handles_graphics_sync(struct ipc_message_channel *imc,
                                  void *out_data,
                                  size_t size,
                                  xrt_graphics_sync_handle_t *out_handles,
                                  uint32_t handle_count)
{
	//! @todo Temporary hack to send no handles.
	if (handle_count == 0) {
		return ipc_receive(imc, out_data, size);
	}
	return ipc_receive_fds(imc, out_data, size, out_handles, handle_count);
}

xrt_result_t
ipc_send_handles_graphics_sync(struct ipc_message_channel *imc,
                               const void *data,
                               size_t size,
                               const xrt_graphics_sync_handle_t *handles,
                               uint32_t handle_count)
{
	//! @todo Temporary hack to send no handles.
	if (handle_count == 0) {
		return ipc_send(imc, data, size);
	}
	return ipc_send_fds(imc, data, size, handles, handle_count);
}

#else
#error "Need port to transport these graphics buffers"
#endif
