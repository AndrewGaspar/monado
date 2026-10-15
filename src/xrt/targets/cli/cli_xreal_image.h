// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Grayscale image loading for cli_cmd_xreal_slam_replay.c (OpenCV, C linkage).
 * @author Andrew Gaspar
 */

#pragma once

#include "util/u_frame.h"
#include "xrt/xrt_frame.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Load an 8-bit image file as a new L8 frame.
bool
cli_xreal_load_gray(const char *path, struct xrt_frame **out_frame);

#ifdef __cplusplus
}
#endif
