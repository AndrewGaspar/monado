// Copyright 2020-2023, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Simulated HMD device.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_simulated
 */

#include "xrt/xrt_device.h"

#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_pretty_print.h"
#include "util/u_time.h"
#include "util/u_var.h"

#include "os/os_threading.h"
#include "tracking/t_plane_detections.h"

#include "simulated_interface.h"

#include <stdio.h>
#include <string.h>


/*
 *
 * Structs and defines.
 *
 */

//! Most static planes from SIMULATED_PLANES.
#define SIMULATED_MAX_PLANES 8

/*!
 * A example HMD device.
 *
 * @implements xrt_device
 */
struct simulated_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;
	struct xrt_pose center;

	uint64_t created_ns;
	float diameter_m;

	enum u_logging_level log_level;
	enum simulated_movement movement;

	//! Static planes from SIMULATED_PLANES, for testing plane detection consumers without hardware.
	struct os_mutex planes_mutex;
	struct t_plane_requests plane_requests;
	uint32_t plane_count;
	struct t_plane_location planes[SIMULATED_MAX_PLANES];
	struct xrt_vec2 plane_vertices[SIMULATED_MAX_PLANES][4];
};


/*
 *
 * Functions
 *
 */

static inline struct simulated_hmd *
simulated_hmd(struct xrt_device *xdev)
{
	return (struct simulated_hmd *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(simulated_log, "SIMULATED_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_NUM_OPTION(view_count, "SIMULATED_VIEW_COUNT", 2)
DEBUG_GET_ONCE_OPTION(simulated_planes, "SIMULATED_PLANES", NULL)

#define HMD_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

static void
simulated_hmd_destroy(struct xrt_device *xdev)
{
	struct simulated_hmd *dh = simulated_hmd(xdev);

	// Remove the variable tracking.
	u_var_remove_root(dh);

	os_mutex_destroy(&dh->planes_mutex);
	u_device_free(&dh->base);
}

static xrt_result_t
simulated_hmd_get_tracked_pose(struct xrt_device *xdev,
                               enum xrt_input_name name,
                               int64_t at_timestamp_ns,
                               struct xrt_space_relation *out_relation)
{
	struct simulated_hmd *hmd = simulated_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	const double time_s = time_ns_to_s(at_timestamp_ns - hmd->created_ns);
	const double d = hmd->diameter_m;
	const double d2 = d * 2;
	const double t = 2.0;
	const double t2 = t * 2;
	const double t3 = t * 3;
	const double t4 = t * 4;
	const struct xrt_vec3 up = {0, 1, 0};

	switch (hmd->movement) {
	default:
	case SIMULATED_MOVEMENT_WOBBLE: {
		struct xrt_pose tmp = XRT_POSE_IDENTITY;

		// Wobble time.
		tmp.position.x = sin((time_s / t2) * M_PI) * d2 - d;
		tmp.position.y = sin((time_s / t) * M_PI) * d;
		tmp.orientation.x = sin((time_s / t3) * M_PI) / 64.0f;
		tmp.orientation.y = sin((time_s / t4) * M_PI) / 16.0f;
		tmp.orientation.z = sin((time_s / t4) * M_PI) / 64.0f;
		math_quat_normalize(&tmp.orientation);

		// Transform with center to set it.
		math_pose_transform(&hmd->center, &tmp, &hmd->pose);
	} break;
	case SIMULATED_MOVEMENT_ROTATE: {
		struct xrt_pose tmp = XRT_POSE_IDENTITY;

		// Rotate around the up vector.
		math_quat_from_angle_vector(time_s / 4, &up, &hmd->pose.orientation);

		// Transform with center to set it.
		math_pose_transform(&hmd->center, &tmp, &hmd->pose);
	} break;
	case SIMULATED_MOVEMENT_STATIONARY:
		// Reset pose.
		hmd->pose = hmd->center;
		break;
	}

	out_relation->pose = hmd->pose;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	return XRT_SUCCESS;
}

static xrt_result_t
simulated_ref_space_usage(struct xrt_device *xdev,
                          enum xrt_reference_space_type type,
                          enum xrt_input_name name,
                          bool used)
{
	struct simulated_hmd *hmd = simulated_hmd(xdev);

	struct u_pp_sink_stack_only sink;
	u_pp_delegate_t dg = u_pp_sink_stack_only_init(&sink);

	u_pp(dg, "Ref space ");
	u_pp_xrt_reference_space_type(dg, type);
	u_pp(dg, " is %sused", used ? "" : "not ");

	if (name != 0) {
		u_pp(dg, ", driven by ");
		u_pp_xrt_input_name(dg, name);
		u_pp(dg, ".");
	} else {
		u_pp(dg, ", not controlled by us.");
	}

	HMD_INFO(hmd, "%s", sink.buffer);

	return XRT_SUCCESS;
}

/*
 *
 * Static planes (SIMULATED_PLANES).
 *
 */

static xrt_result_t
simulated_hmd_begin_plane_detection_ext(struct xrt_device *xdev,
                                        const struct xrt_plane_detector_begin_info_ext *begin_info,
                                        uint64_t plane_detection_id,
                                        uint64_t *out_plane_detection_id)
{
	struct simulated_hmd *hmd = simulated_hmd(xdev);
	(void)plane_detection_id;
	os_mutex_lock(&hmd->planes_mutex);
	xrt_result_t xret = t_plane_requests_begin(&hmd->plane_requests, begin_info, out_plane_detection_id);
	os_mutex_unlock(&hmd->planes_mutex);
	return xret;
}

static xrt_result_t
simulated_hmd_destroy_plane_detection_ext(struct xrt_device *xdev, uint64_t plane_detection_id)
{
	struct simulated_hmd *hmd = simulated_hmd(xdev);
	os_mutex_lock(&hmd->planes_mutex);
	t_plane_requests_destroy(&hmd->plane_requests, plane_detection_id);
	os_mutex_unlock(&hmd->planes_mutex);
	return XRT_SUCCESS;
}

static xrt_result_t
simulated_hmd_get_plane_detection_state_ext(struct xrt_device *xdev,
                                            uint64_t plane_detection_id,
                                            enum xrt_plane_detector_state_ext *out_state)
{
	struct simulated_hmd *hmd = simulated_hmd(xdev);
	struct xrt_plane_detector_begin_info_ext query;
	os_mutex_lock(&hmd->planes_mutex);
	const bool known = t_plane_requests_get(&hmd->plane_requests, plane_detection_id, &query);
	os_mutex_unlock(&hmd->planes_mutex);
	*out_state = known ? XRT_PLANE_DETECTOR_STATE_DONE_EXT : XRT_PLANE_DETECTOR_STATE_NONE_EXT;
	return XRT_SUCCESS;
}

static xrt_result_t
simulated_hmd_get_plane_detections_ext(struct xrt_device *xdev,
                                       uint64_t plane_detection_id,
                                       struct xrt_plane_detections_ext *out_detections)
{
	struct simulated_hmd *hmd = simulated_hmd(xdev);
	struct xrt_plane_detector_begin_info_ext query;
	os_mutex_lock(&hmd->planes_mutex);
	const bool known = t_plane_requests_get(&hmd->plane_requests, plane_detection_id, &query);
	os_mutex_unlock(&hmd->planes_mutex);
	if (!known) {
		out_detections->location_count = 0;
		return XRT_ERROR_INVALID_ARGUMENT;
	}
	return t_plane_detections_fill(&query, hmd->planes, hmd->plane_count, out_detections);
}

/*!
 * SIMULATED_PLANES="<label>:<x>,<y>,<z>,<width>,<depth>;..." horizontal planes facing up, centered at
 * (x, y, z) in the tracking origin (Y up), width along X, depth along Z; label floor, table (PLATFORM)
 * or anything else (undefined). E.g. "table:0,-0.55,-0.8,1.2,0.6;floor:0,-1.1,-1.5,3,3".
 */
static void
simulated_hmd_parse_planes(struct simulated_hmd *hmd, const char *spec)
{
	char buf[1024];
	snprintf(buf, sizeof(buf), "%s", spec);
	char *save = NULL;
	for (char *item = strtok_r(buf, ";", &save); item != NULL && hmd->plane_count < SIMULATED_MAX_PLANES;
	     item = strtok_r(NULL, ";", &save)) {
		char label[32] = "";
		float x, y, z, w, d;
		if (sscanf(item, " %31[^:]:%f,%f,%f,%f,%f", label, &x, &y, &z, &w, &d) != 6 || w <= 0 || d <= 0) {
			HMD_ERROR(hmd, "SIMULATED_PLANES: could not parse '%s'", item);
			continue;
		}
		const uint32_t i = hmd->plane_count++;
		struct t_plane_location *l = &hmd->planes[i];
		memset(l, 0, sizeof(*l));
		l->id = i + 1;
		// The plane's +Z (its normal) is up: -90 degrees about X; its +Y is then -Z (away).
		l->pose.orientation = (struct xrt_quat){-0.70710678f, 0, 0, 0.70710678f};
		l->pose.position = (struct xrt_vec3){x, y, z};
		l->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		l->extents = (struct xrt_vec2){w, d};
		l->orientation = XRT_PLANE_DETECTOR_ORIENTATION_HORIZONTAL_UPWARD_EXT;
		l->semantic_type = strcmp(label, "floor") == 0   ? XRT_PLANE_DETECTOR_SEMANTIC_TYPE_FLOOR_EXT
		                   : strcmp(label, "table") == 0 ? XRT_PLANE_DETECTOR_SEMANTIC_TYPE_PLATFORM_EXT
		                                                 : XRT_PLANE_DETECTOR_SEMANTIC_TYPE_UNDEFINED_EXT;
		hmd->plane_vertices[i][0] = (struct xrt_vec2){-w / 2, -d / 2};
		hmd->plane_vertices[i][1] = (struct xrt_vec2){w / 2, -d / 2};
		hmd->plane_vertices[i][2] = (struct xrt_vec2){w / 2, d / 2};
		hmd->plane_vertices[i][3] = (struct xrt_vec2){-w / 2, d / 2};
		l->vertex_count = 4;
		l->vertices = hmd->plane_vertices[i];
		HMD_INFO(hmd, "Static plane #%u '%s' at (%.2f, %.2f, %.2f), %.2f x %.2f m", l->id, label, x, y, z, w,
		         d);
	}
}

/*
 *
 * 'Exported' functions.
 *
 */

enum u_logging_level
simulated_log_level(void)
{
	return debug_get_log_option_simulated_log();
}

struct xrt_device *
simulated_hmd_create(enum simulated_movement movement, const struct xrt_pose *center)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct simulated_hmd *hmd = U_DEVICE_ALLOCATE(struct simulated_hmd, flags, 1, 0);
	hmd->base.update_inputs = u_device_noop_update_inputs;
	hmd->base.get_tracked_pose = simulated_hmd_get_tracked_pose;
	hmd->base.get_view_poses = u_device_get_view_poses;
	hmd->base.get_visibility_mask = u_device_get_visibility_mask;
	hmd->base.ref_space_usage = simulated_ref_space_usage;
	hmd->base.destroy = simulated_hmd_destroy;
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.supported.ref_space_usage = true;
	hmd->pose.orientation.w = 1.0f; // All other values set to zero.
	hmd->center = *center;
	hmd->created_ns = os_monotonic_get_ns();
	hmd->diameter_m = 0.05f;
	hmd->log_level = simulated_log_level();
	hmd->movement = movement;

	hmd->base.hmd->view_count = debug_get_num_option_view_count();

	os_mutex_init(&hmd->planes_mutex);
	t_plane_requests_init(&hmd->plane_requests);
	const char *planes = debug_get_option_simulated_planes();
	if (planes != NULL && planes[0] != '\0') {
		simulated_hmd_parse_planes(hmd, planes);
	}
	if (hmd->plane_count > 0) {
		hmd->base.supported.planes = true;
		hmd->base.supported.plane_capability_flags = (enum xrt_plane_detection_capability_flags_ext)(
		    XRT_PLANE_DETECTION_CAPABILITY_PLANE_DETECTION_BIT_EXT |
		    XRT_PLANE_DETECTION_CAPABILITY_SEMANTIC_FLOOR_BIT_EXT |
		    XRT_PLANE_DETECTION_CAPABILITY_SEMANTIC_PLATFORM_BIT_EXT |
		    XRT_PLANE_DETECTION_CAPABILITY_ORIENTATION_BIT_EXT);
		hmd->base.begin_plane_detection_ext = simulated_hmd_begin_plane_detection_ext;
		hmd->base.destroy_plane_detection_ext = simulated_hmd_destroy_plane_detection_ext;
		hmd->base.get_plane_detection_state_ext = simulated_hmd_get_plane_detection_state_ext;
		hmd->base.get_plane_detections_ext = simulated_hmd_get_plane_detections_ext;
	}
	// Print name.
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Simulated HMD");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "Simulated HMD");

	// Setup input.
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;

	// Setup info.
	bool ret = true;
	struct u_device_simple_info info;
	info.display.w_pixels = 1280;
	info.display.h_pixels = 720;
	info.display.w_meters = 0.13f;
	info.display.h_meters = 0.07f;
	info.lens_horizontal_separation_meters = 0.13f / 2.0f;
	info.lens_vertical_position_meters = 0.07f / 2.0f;

	if (hmd->base.hmd->view_count == 1) {
		info.fov[0] = 120.0f * (M_PI / 180.0f);
		ret = u_device_setup_one_eye(&hmd->base, &info);
	} else if (hmd->base.hmd->view_count == 2) {
		info.fov[0] = 85.0f * (M_PI / 180.0f);
		info.fov[1] = 85.0f * (M_PI / 180.0f);
		ret = u_device_setup_split_side_by_side(&hmd->base, &info);
	} else {
		U_LOG_E("Invalid view count");
		ret = false;
	}
	if (!ret) {
		HMD_ERROR(hmd, "Failed to setup basic device info");
		simulated_hmd_destroy(&hmd->base);
		return NULL;
	}

	// Setup variable tracker.
	u_var_add_root(hmd, "Simulated HMD", true);
	u_var_add_pose(hmd, &hmd->pose, "pose");
	u_var_add_pose(hmd, &hmd->center, "center");
	u_var_add_f32(hmd, &hmd->diameter_m, "diameter_m");
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");

	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&hmd->base);

	return &hmd->base;
}
