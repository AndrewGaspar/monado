// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra SLAM camera UVC capture.
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_camera.h"
#include "xreal_air_camera_decode.h"
#include "xreal_air_interface.h"

#include "os/os_threading.h"
#include "os/os_time.h"
#include "util/u_misc.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#define CAM_LOG(cam, level, ...) U_LOG_IFL(level, (cam)->log_level, __VA_ARGS__)
#define CAM_DEBUG(cam, ...) CAM_LOG(cam, U_LOGGING_DEBUG, "Xreal Air camera: " __VA_ARGS__)
#define CAM_INFO(cam, ...) CAM_LOG(cam, U_LOGGING_INFO, "Xreal Air camera: " __VA_ARGS__)
#define CAM_ERROR(cam, ...) CAM_LOG(cam, U_LOGGING_ERROR, "Xreal Air camera: " __VA_ARGS__)

#define NUM_BUFFERS 4
#define POLL_TIMEOUT_MS 100

struct xreal_air_camera_buffer
{
	void *mem;
	size_t length;
};

struct xreal_air_camera
{
	enum u_logging_level log_level;

	int fd;
	char path[64];

	struct xreal_air_camera_buffer buffers[NUM_BUFFERS];
	uint32_t buffer_count;
	bool streaming;

	xreal_air_camera_frame_func_t func;
	void *user;

	struct os_thread_helper oth;
};


/*
 *
 * Device discovery.
 *
 */

static bool
read_sysfs_line(const char *path, char *out, size_t out_size)
{
	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return false;
	}

	bool ok = fgets(out, (int)out_size, f) != NULL;
	(void)fclose(f);

	if (ok) {
		out[strcspn(out, "\r\n")] = '\0';
	}
	return ok;
}

bool
xreal_air_camera_find_device(char *out, size_t out_size)
{
	DIR *dir = opendir("/sys/class/video4linux");
	if (dir == NULL) {
		return false;
	}

	bool found = false;
	struct dirent *ent;
	while (!found && (ent = readdir(dir)) != NULL) {
		if (strncmp(ent->d_name, "video", 5) != 0) {
			continue;
		}

		char path[PATH_MAX];
		char value[32];

		// UVC exposes a capture node (index 0) and a metadata node (index 1) per function.
		(void)snprintf(path, sizeof(path), "/sys/class/video4linux/%s/index", ent->d_name);
		if (!read_sysfs_line(path, value, sizeof(value)) || strcmp(value, "0") != 0) {
			continue;
		}

		// device -> the USB interface, its parent is the USB device with the ids.
		(void)snprintf(path, sizeof(path), "/sys/class/video4linux/%s/device/../idVendor", ent->d_name);
		if (!read_sysfs_line(path, value, sizeof(value)) || strtol(value, NULL, 16) != XREAL_AIR_VID) {
			continue;
		}

		(void)snprintf(path, sizeof(path), "/sys/class/video4linux/%s/device/../idProduct", ent->d_name);
		if (!read_sysfs_line(path, value, sizeof(value)) || strtol(value, NULL, 16) != XREAL_AIR_2_ULTRA_PID) {
			continue;
		}

		(void)snprintf(out, out_size, "/dev/%s", ent->d_name);
		found = true;
	}

	(void)closedir(dir);
	return found;
}


/*
 *
 * Streaming.
 *
 */

static int
xioctl(int fd, unsigned long request, void *arg)
{
	int ret;
	do {
		ret = ioctl(fd, request, arg);
	} while (ret < 0 && errno == EINTR);
	return ret;
}

static bool
setup_stream(struct xreal_air_camera *cam)
{
	struct v4l2_capability cap;
	U_ZERO(&cap);
	if (xioctl(cam->fd, VIDIOC_QUERYCAP, &cap) < 0) {
		CAM_ERROR(cam, "VIDIOC_QUERYCAP failed on %s: %s", cam->path, strerror(errno));
		return false;
	}
	if ((cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) == 0 || (cap.device_caps & V4L2_CAP_STREAMING) == 0) {
		CAM_ERROR(cam, "%s is not a streaming capture device", cam->path);
		return false;
	}

	// The one and only mode: "YUYV" 640x241, which really carries 640x482 L8.
	struct v4l2_format fmt;
	U_ZERO(&fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = XREAL_AIR_CAM_RAW_WIDTH;
	fmt.fmt.pix.height = XREAL_AIR_CAM_RAW_HEIGHT / 2;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0) {
		CAM_ERROR(cam, "VIDIOC_S_FMT failed: %s", strerror(errno));
		return false;
	}
	if (fmt.fmt.pix.width != XREAL_AIR_CAM_RAW_WIDTH || fmt.fmt.pix.height != XREAL_AIR_CAM_RAW_HEIGHT / 2 ||
	    fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV || fmt.fmt.pix.sizeimage < XREAL_AIR_CAM_RAW_SIZE) {
		CAM_ERROR(cam, "Unexpected format %ux%u fourcc 0x%08x size %u", fmt.fmt.pix.width, fmt.fmt.pix.height,
		          fmt.fmt.pix.pixelformat, fmt.fmt.pix.sizeimage);
		return false;
	}

	struct v4l2_requestbuffers req;
	U_ZERO(&req);
	req.count = NUM_BUFFERS;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) {
		CAM_ERROR(cam, "VIDIOC_REQBUFS failed: %s", strerror(errno));
		return false;
	}
	if (req.count > NUM_BUFFERS) {
		req.count = NUM_BUFFERS;
	}

	for (uint32_t i = 0; i < req.count; i++) {
		struct v4l2_buffer buf;
		U_ZERO(&buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (xioctl(cam->fd, VIDIOC_QUERYBUF, &buf) < 0) {
			CAM_ERROR(cam, "VIDIOC_QUERYBUF failed: %s", strerror(errno));
			return false;
		}

		void *mem = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam->fd, buf.m.offset);
		if (mem == MAP_FAILED) {
			CAM_ERROR(cam, "mmap failed: %s", strerror(errno));
			return false;
		}
		cam->buffers[i].mem = mem;
		cam->buffers[i].length = buf.length;
		cam->buffer_count = i + 1;

		if (xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
			CAM_ERROR(cam, "VIDIOC_QBUF failed: %s", strerror(errno));
			return false;
		}
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
		CAM_ERROR(cam, "VIDIOC_STREAMON failed: %s", strerror(errno));
		return false;
	}
	cam->streaming = true;

	return true;
}

static void *
camera_thread(void *ptr)
{
	struct xreal_air_camera *cam = (struct xreal_air_camera *)ptr;
	os_thread_helper_name(&cam->oth, "XREAL Air Camera");

	uint64_t frames = 0;
	uint64_t short_frames = 0;

	while (os_thread_helper_is_running(&cam->oth)) {
		struct pollfd pfd = {.fd = cam->fd, .events = POLLIN};
		int ret = poll(&pfd, 1, POLL_TIMEOUT_MS);
		if (ret < 0) {
			if (errno == EINTR) {
				continue;
			}
			CAM_ERROR(cam, "poll failed: %s", strerror(errno));
			break;
		}
		if (ret == 0) {
			continue;
		}
		if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
			CAM_ERROR(cam, "Device error/hangup (unplugged?), stopping capture");
			break;
		}

		struct v4l2_buffer buf;
		U_ZERO(&buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (xioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
			if (errno == EAGAIN) {
				continue;
			}
			CAM_ERROR(cam, "VIDIOC_DQBUF failed: %s", strerror(errno));
			break;
		}

		const int64_t dequeue_ns = (int64_t)os_monotonic_get_ns();

		if (buf.index < cam->buffer_count && (buf.flags & V4L2_BUF_FLAG_ERROR) == 0 &&
		    buf.bytesused >= XREAL_AIR_CAM_TELEMETRY_OFFSET + XREAL_AIR_CAM_TELEMETRY_RAW_LEN) {
			struct xreal_air_camera_raw_frame frame = {
			    .data = (const uint8_t *)cam->buffers[buf.index].mem,
			    .size = buf.bytesused,
			    .v4l2_sequence = buf.sequence,
			    .kernel_ns = 0,
			    .dequeue_ns = dequeue_ns,
			};
			if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
				frame.kernel_ns = (int64_t)buf.timestamp.tv_sec * U_TIME_1S_IN_NS +
				                  (int64_t)buf.timestamp.tv_usec * U_TIME_1US_IN_NS;
			}

			cam->func(cam->user, &frame);
			frames++;
		} else {
			// Typically the first frames after STREAMON, flagged as errored by uvcvideo.
			short_frames++;
			if (short_frames <= 16) {
				CAM_INFO(cam, "Dropping bad frame: seq %u flags 0x%x (%s) bytesused %u", buf.sequence,
				         buf.flags, (buf.flags & V4L2_BUF_FLAG_ERROR) != 0 ? "error" : "short",
				         buf.bytesused);
			}
		}

		if (xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
			CAM_ERROR(cam, "VIDIOC_QBUF failed: %s", strerror(errno));
			break;
		}
	}

	CAM_INFO(cam, "Capture thread exiting after %" PRIu64 " frames (%" PRIu64 " bad)", frames, short_frames);
	return NULL;
}

static void
teardown(struct xreal_air_camera *cam)
{
	if (cam->streaming) {
		enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		(void)xioctl(cam->fd, VIDIOC_STREAMOFF, &type);
		cam->streaming = false;
	}

	for (uint32_t i = 0; i < cam->buffer_count; i++) {
		(void)munmap(cam->buffers[i].mem, cam->buffers[i].length);
	}
	cam->buffer_count = 0;

	if (cam->fd >= 0) {
		// Release the buffers before closing.
		struct v4l2_requestbuffers req;
		U_ZERO(&req);
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		req.memory = V4L2_MEMORY_MMAP;
		(void)xioctl(cam->fd, VIDIOC_REQBUFS, &req);

		(void)close(cam->fd);
		cam->fd = -1;
	}
}


/*
 *
 * Exported functions.
 *
 */

struct xreal_air_camera *
xreal_air_camera_create(const char *path,
                        xreal_air_camera_frame_func_t func,
                        void *user,
                        enum u_logging_level log_level)
{
	struct xreal_air_camera *cam = U_TYPED_CALLOC(struct xreal_air_camera);
	cam->log_level = log_level;
	cam->fd = -1;
	cam->func = func;
	cam->user = user;
	(void)snprintf(cam->path, sizeof(cam->path), "%s", path);

	if (os_thread_helper_init(&cam->oth) != 0) {
		free(cam);
		return NULL;
	}

	cam->fd = open(cam->path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (cam->fd < 0) {
		CAM_ERROR(cam, "Could not open %s: %s", cam->path, strerror(errno));
		goto error;
	}

	if (!setup_stream(cam)) {
		goto error;
	}

	if (os_thread_helper_start(&cam->oth, camera_thread, cam) != 0) {
		CAM_ERROR(cam, "Could not start capture thread");
		goto error;
	}

	CAM_INFO(cam, "Streaming from %s", cam->path);
	return cam;

error:
	teardown(cam);
	os_thread_helper_destroy(&cam->oth);
	free(cam);
	return NULL;
}

void
xreal_air_camera_destroy(struct xreal_air_camera **cam_ptr)
{
	struct xreal_air_camera *cam = *cam_ptr;
	if (cam == NULL) {
		return;
	}

	// Stops and joins the thread (it polls with a timeout, so this is bounded).
	os_thread_helper_destroy(&cam->oth);
	teardown(cam);

	free(cam);
	*cam_ptr = NULL;
}
