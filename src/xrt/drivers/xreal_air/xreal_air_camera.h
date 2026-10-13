// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra SLAM camera UVC capture.
 *
 * A minimal V4L2 mmap capture of the glasses' single UVC stream, handing out the raw
 * 640x482 L8 frames (see @ref xreal_air_camera_decode.h). Deliberately not the generic v4l2
 * frameserver: the stream needs no mode selection or control tweaking, and owning the loop
 * gives a clean poll()-based shutdown and the kernel timestamps verbatim.
 *
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "util/u_logging.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * One raw frame as dequeued, only valid for the duration of the callback.
 */
struct xreal_air_camera_raw_frame
{
	const uint8_t *data;
	size_t size;

	uint32_t v4l2_sequence; //!< V4L2 buffer sequence number (gaps = dropped frames).
	int64_t kernel_ns;      //!< V4L2 buffer timestamp, CLOCK_MONOTONIC (or 0 if not monotonic).
	int64_t dequeue_ns;     //!< os_monotonic_get_ns() right after VIDIOC_DQBUF.
};

typedef void (*xreal_air_camera_frame_func_t)(void *user, const struct xreal_air_camera_raw_frame *frame);

struct xreal_air_camera;

/*!
 * Find the Air 2 Ultra's camera capture node (/dev/videoN) by USB VID/PID in sysfs.
 *
 * @return true and the path in @p out if found.
 */
bool
xreal_air_camera_find_device(char *out, size_t out_size);

/*!
 * Open @p path and start streaming on a thread of its own, calling @p func for every frame.
 */
struct xreal_air_camera *
xreal_air_camera_create(const char *path,
                        xreal_air_camera_frame_func_t func,
                        void *user,
                        enum u_logging_level log_level);

/*!
 * Stop streaming, join the thread and close the device. Sets @p cam_ptr to NULL.
 */
void
xreal_air_camera_destroy(struct xreal_air_camera **cam_ptr);

#ifdef __cplusplus
}
#endif
