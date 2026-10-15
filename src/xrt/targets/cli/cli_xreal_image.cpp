// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Grayscale image loading for cli_cmd_xreal_slam_replay.c (OpenCV, C linkage).
 * @author Andrew Gaspar
 */

#include "cli_xreal_image.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cstring>

extern "C" bool
cli_xreal_load_gray(const char *path, struct xrt_frame **out_frame)
{
	cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (img.empty() || img.type() != CV_8UC1) {
		return false;
	}

	struct xrt_frame *xf = nullptr;
	u_frame_create_one_off(XRT_FORMAT_L8, (uint32_t)img.cols, (uint32_t)img.rows, &xf);
	if (xf == nullptr) {
		return false;
	}
	for (int y = 0; y < img.rows; y++) {
		std::memcpy(xf->data + (size_t)y * xf->stride, img.ptr<uint8_t>(y), (size_t)img.cols);
	}
	*out_frame = xf;
	return true;
}
