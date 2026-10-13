// Copyright 2025, Luka Panio
// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra SLAM camera frame decoding (descramble, telemetry, orientation).
 *
 * Pure functions, no device access, so they can be unit tested on synthetic frames.
 *
 * The single UVC stream advertises YUYV 640x241@60 but every byte is one 8-bit grayscale
 * pixel: the frame is really a 640x482 L8 raster. Rows 0-479 are the image, scrambled as
 * 128 chunks of 2400 bytes under a fixed permutation whose phase rotates every frame;
 * row 480 is a telemetry row and row 481 is padding. Consecutive frames alternate between
 * the two cameras, the pair sharing a counter, but the order within a pair is NOT fixed,
 * the camera is identified from the telemetry row.
 *
 * The descrambled bytes are the sensor readout: 480 lines of 640 pixels, where each
 * readout line is a *column* of the upright image (the sensors are mounted rotated 90
 * degrees, and the two cameras 180 degrees opposed to each other). Orienting gives the
 * 480x640 (w x h) portrait image the factory "SLAM_camera" calibration describes.
 *
 * Sources: chunk map from mazeasdamien/myXreal, descramble from Luka Panio's /
 * Benjamin Berg's WIP Monado branch (gitlab.freedesktop.org/benzea/monado), which also gets
 * the orientation and camera id right (verified live, see @ref xreal_air_cam_orient and
 * @ref xreal_air_cam_identify), telemetry layout from sean9061/xreal-air2ultra-camera and
 * DeskUnreal/xreal-vio-vr.
 *
 * @author Luka Panio <lukapanio@gmail.com>
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XREAL_AIR_CAM_RAW_WIDTH 640                                                 //!< Raw raster width.
#define XREAL_AIR_CAM_RAW_HEIGHT 482                                                //!< 480 image + 2 rows.
#define XREAL_AIR_CAM_RAW_SIZE (XREAL_AIR_CAM_RAW_WIDTH * XREAL_AIR_CAM_RAW_HEIGHT) //!< 308480 bytes.
#define XREAL_AIR_CAM_READOUT_LINE 640                                              //!< Pixels per readout line.
#define XREAL_AIR_CAM_READOUT_LINES 480                                             //!< Readout lines.
#define XREAL_AIR_CAM_IMAGE_SIZE (XREAL_AIR_CAM_READOUT_LINE * XREAL_AIR_CAM_READOUT_LINES) //!< 307200 bytes.
#define XREAL_AIR_CAM_TELEMETRY_OFFSET XREAL_AIR_CAM_IMAGE_SIZE                             //!< Row 480.
#define XREAL_AIR_CAM_CHUNK_SIZE 2400
#define XREAL_AIR_CAM_CHUNK_COUNT 128

//! Upright image size, matches the factory calibration's resolution [w, h] = [480, 640].
#define XREAL_AIR_CAM_WIDTH 480
#define XREAL_AIR_CAM_HEIGHT 640

//! How many leading telemetry bytes are kept verbatim for offline reverse engineering.
#define XREAL_AIR_CAM_TELEMETRY_RAW_LEN 80

//! Camera index, matches the factory calibration's SLAM_camera device_1 / device_2.
enum xreal_air_cam_id
{
	XREAL_AIR_CAM_UNKNOWN = -1,
	XREAL_AIR_CAM_LEFT = 0,  //!< SLAM_camera device_1, EuRoC cam0.
	XREAL_AIR_CAM_RIGHT = 1, //!< SLAM_camera device_2, EuRoC cam1.
};

/*!
 * The decoded telemetry row (row 480). Little endian. Field confidence varies, the raw bytes
 * are kept so the dataset can be re-decoded later.
 */
struct xreal_air_cam_telemetry
{
	uint64_t device_ns; //!< Offset 0: device clock, ns since power-on, per frame (HIGH confidence).
	uint16_t counter;   //!< Offset 18: stereo pair counter, shared by both frames of a pair.
	uint32_t exposure;  //!< Offset 22: u24 exposure, ~ns (MED confidence).
	uint8_t gain;       //!< Offset 26: gain / exposure index (MED-LOW confidence).
	uint8_t id_byte58;  //!< Offset 58: always 0 on our Air 2 Ultra (sean9061 saw 0x20/0x21), kept raw.
	uint8_t id_byte59;  //!< Offset 59 (0x3b): camera id, 0 = left, 1 = right (verified live).
	uint64_t pair_ts;   //!< Offset 62 (0x3e): second timestamp, same for both frames of a pair.
	uint8_t raw[XREAL_AIR_CAM_TELEMETRY_RAW_LEN]; //!< Leading telemetry bytes, verbatim.
};

//! The fixed chunk permutation, from mazeasdamien/myXreal.
extern const uint8_t xreal_air_cam_chunk_map[XREAL_AIR_CAM_CHUNK_COUNT];

/*!
 * Parse the telemetry row of a raw frame, @p raw must be at least @ref XREAL_AIR_CAM_RAW_SIZE
 * minus the padding row long.
 */
bool
xreal_air_cam_parse_telemetry(const uint8_t *raw, size_t size, struct xreal_air_cam_telemetry *out);

/*!
 * Which camera a frame came from.
 *
 * Byte 59 (0x3b) is non-zero for the right camera (Berg, DeskUnreal). Verified live on the
 * user's unit (firmware of 2026-10): it alternates 0/1 within every pair while byte 58, which
 * sean9061 documents as 0x20/0x21, is always 0, so byte 58 is not used. Stereo check: with
 * this id and @ref xreal_air_cam_orient, near objects sit further right in the left image.
 */
enum xreal_air_cam_id
xreal_air_cam_identify(const struct xreal_air_cam_telemetry *t);

/*!
 * Find the phase of the chunk permutation: the index into @ref xreal_air_cam_chunk_map of the
 * raw chunk holding the first readout line, which lies in the black fisheye vignette and so
 * is the raw chunk whose first 128 bytes have the smallest sum.
 */
uint32_t
xreal_air_cam_find_phase(const uint8_t *raw);

/*!
 * Undo the chunk permutation: write the 480x640 readout (480 lines of 640 pixels) to @p out.
 *
 * @return The phase used, see @ref xreal_air_cam_find_phase.
 */
uint32_t
xreal_air_cam_descramble(const uint8_t *raw, uint8_t *out_readout);

/*!
 * Orient a descrambled readout into the upright 480x640 (w x h) image of camera @p cam.
 *
 * Readout line r, pixel c lands at (x, y) = (479 - r, c) for the right camera and at
 * (r, 639 - c) for the left camera, i.e. readout lines are image columns and the two cameras
 * are 180 degrees opposed. This is Berg's mapping (left 90 degrees counter clockwise, right
 * clockwise), verified live: it gives the true, non-mirrored view (sean9061 and DeskUnreal
 * produce mirror images) and is the frame the factory SLAM_camera intrinsics are in.
 */
void
xreal_air_cam_orient(const uint8_t *readout, enum xreal_air_cam_id cam, uint8_t *out, size_t out_stride);

/*!
 * Descramble and orient in one pass, straight from the raw frame.
 *
 * @return The phase used, see @ref xreal_air_cam_find_phase.
 */
uint32_t
xreal_air_cam_decode(const uint8_t *raw, enum xreal_air_cam_id cam, uint8_t *out, size_t out_stride);

#ifdef __cplusplus
}
#endif
