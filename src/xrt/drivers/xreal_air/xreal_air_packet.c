// Copyright 2023-2025, Tobias Frisch
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air packet parsing implementation.
 * @author Tobias Frisch <jacki@thejackimonster.de>
 * @ingroup drv_xreal_air
 */

#include "xrt/xrt_compiler.h"

#include "xreal_air_hmd.h"

#include <cjson/cJSON.h>
#include <string.h>


/*
 *
 * Buffer reading helpers.
 *
 */

static inline void
skip(const uint8_t **buffer, size_t num)
{
	*buffer += num;
}

static inline void
read_i16(const uint8_t **buffer, int16_t *out_value)
{
	*out_value = (*(*buffer + 0) << 0u) | // Byte 0
	             (*(*buffer + 1) << 8u);  // Byte 1
	*buffer += 2;
}

static inline void
read_i24_to_i32(const uint8_t **buffer, int32_t *out_value)
{
	*out_value = (*(*buffer + 0) << 0u) | // Byte 0
	             (*(*buffer + 1) << 8u) | // Byte 1
	             (*(*buffer + 2) << 16u); // Byte 2
	if (*(*buffer + 2) & 0x80u)
		*out_value |= (0xFFu << 24u); // Properly sign extend.
	*buffer += 3;
}

static inline void
read_i32(const uint8_t **buffer, int32_t *out_value)
{
	*out_value = (*(*buffer + 0) << 0u) |  // Byte 0
	             (*(*buffer + 1) << 8u) |  // Byte 1
	             (*(*buffer + 2) << 16u) | // Byte 2
	             (*(*buffer + 3) << 24u);  // Byte 3
	*buffer += 4;
}

static inline void
read_i16_rev(const uint8_t **buffer, int16_t *out_value)
{
	*out_value = (*(*buffer + 1) << 0u) | // Byte 1
	             (*(*buffer + 0) << 8u);  // Byte 2
	*buffer += 2;
}

static inline void
read_i15_to_i32(const uint8_t **buffer, int32_t *out_value)
{
	int16_t v = (*(*buffer + 0) << 0u) | // Byte 0
	            (*(*buffer + 1) << 8u);  // Byte 1
	v = (v ^ 0x8000);                    // Flip sign bit
	*out_value = v;                      // Properly sign extend.
	*buffer += 2;
}

static inline void
read_i32_rev(const uint8_t **buffer, int32_t *out_value)
{
	*out_value = (*(*buffer + 3) << 0u) |  // Byte 3
	             (*(*buffer + 2) << 8u) |  // Byte 2
	             (*(*buffer + 1) << 16u) | // Byte 1
	             (*(*buffer + 0) << 24u);  // Byte 0
	*buffer += 4;
}

static inline void
read_u8(const uint8_t **buffer, uint8_t *out_value)
{
	*out_value = **buffer;
	*buffer += 1;
}

static inline void
read_u16(const uint8_t **buffer, uint16_t *out_value)
{
	*out_value = (*(*buffer + 0) << 0u) | // Byte 0
	             (*(*buffer + 1) << 8u);  // Byte 1
	*buffer += 2;
}

XRT_MAYBE_UNUSED static inline void
read_u32(const uint8_t **buffer, uint32_t *out_value)
{
	*out_value = (*(*buffer + 0) << 0u) |  // Byte 0
	             (*(*buffer + 1) << 8u) |  // Byte 1
	             (*(*buffer + 2) << 16u) | // Byte 2
	             (*(*buffer + 3) << 24u);  // Byte 3
	*buffer += 4;
}

static inline void
read_u64(const uint8_t **buffer, uint64_t *out_value)
{
	*out_value = ((uint64_t) * (*buffer + 0) << 0u) |  // Byte 0
	             ((uint64_t) * (*buffer + 1) << 8u) |  // Byte 1
	             ((uint64_t) * (*buffer + 2) << 16u) | // Byte 2
	             ((uint64_t) * (*buffer + 3) << 24u) | // Byte 3
	             ((uint64_t) * (*buffer + 4) << 32u) | // Byte 4
	             ((uint64_t) * (*buffer + 5) << 40u) | // Byte 5
	             ((uint64_t) * (*buffer + 6) << 48u) | // Byte 6
	             ((uint64_t) * (*buffer + 7) << 56u);  // Byte 7
	*buffer += 8;
}

static inline void
read_u8_array(const uint8_t **buffer, uint8_t *out_value, size_t num)
{
	memcpy(out_value, (*buffer), num);
	*buffer += num;
}


/*
 *
 * JSON helpers.
 *
 */

static void
read_json_vec3(cJSON *object, const char *const string, struct xrt_vec3 *out_vec3)
{
	cJSON *obj_vec3 = cJSON_GetObjectItem(object, string);

	if ((!obj_vec3) || (!cJSON_IsArray(obj_vec3)) || (cJSON_GetArraySize(obj_vec3) != 3)) {
		return;
	}

	cJSON *obj_x = cJSON_GetArrayItem(obj_vec3, 0);
	cJSON *obj_y = cJSON_GetArrayItem(obj_vec3, 1);
	cJSON *obj_z = cJSON_GetArrayItem(obj_vec3, 2);

	out_vec3->x = (float)cJSON_GetNumberValue(obj_x);
	out_vec3->y = (float)cJSON_GetNumberValue(obj_y);
	out_vec3->z = (float)cJSON_GetNumberValue(obj_z);
}

static void
read_json_quat(cJSON *object, const char *const string, struct xrt_quat *out_quat)
{
	cJSON *obj_quat = cJSON_GetObjectItem(object, string);

	if ((!obj_quat) || (!cJSON_IsArray(obj_quat)) || (cJSON_GetArraySize(obj_quat) != 4)) {
		return;
	}

	cJSON *obj_x = cJSON_GetArrayItem(obj_quat, 0);
	cJSON *obj_y = cJSON_GetArrayItem(obj_quat, 1);
	cJSON *obj_z = cJSON_GetArrayItem(obj_quat, 2);
	cJSON *obj_w = cJSON_GetArrayItem(obj_quat, 3);

	out_quat->x = (float)cJSON_GetNumberValue(obj_x);
	out_quat->y = (float)cJSON_GetNumberValue(obj_y);
	out_quat->z = (float)cJSON_GetNumberValue(obj_z);
	out_quat->w = (float)cJSON_GetNumberValue(obj_w);
}

static void
read_json_array(cJSON *object, const char *const string, int size, float *out_array)
{
	cJSON *obj_array = cJSON_GetObjectItem(object, string);

	if ((!obj_array) || (!cJSON_IsArray(obj_array)) || (cJSON_GetArraySize(obj_array) != size)) {
		return;
	}

	for (int i = 0; i < size; i++) {
		cJSON *obj_i = cJSON_GetArrayItem(obj_array, i);

		if (!obj_i) {
			break;
		}

		out_array[i] = (float)cJSON_GetNumberValue(obj_i);
	}
}


/*
 *
 * Helpers.
 *
 */

static void
read_sample(const uint8_t **buffer, struct xreal_air_parsed_sample *sample)
{
	read_i16(buffer, &sample->gyro_multiplier);
	read_i32(buffer, &sample->gyro_divisor);

	read_i24_to_i32(buffer, &sample->gyro.x);
	read_i24_to_i32(buffer, &sample->gyro.y);
	read_i24_to_i32(buffer, &sample->gyro.z);

	read_i16(buffer, &sample->accel_multiplier);
	read_i32(buffer, &sample->accel_divisor);

	read_i24_to_i32(buffer, &sample->accel.x);
	read_i24_to_i32(buffer, &sample->accel.y);
	read_i24_to_i32(buffer, &sample->accel.z);

	read_i16_rev(buffer, &sample->mag_multiplier);
	read_i32_rev(buffer, &sample->mag_divisor);

	read_i15_to_i32(buffer, &sample->mag.x);
	read_i15_to_i32(buffer, &sample->mag.y);
	read_i15_to_i32(buffer, &sample->mag.z);
}

static void
parse_calibration_json(struct xreal_air_parsed_calibration *calibration, cJSON *dev1)
{
	read_json_vec3(dev1, "accel_bias", &calibration->accel_bias);
	read_json_quat(dev1, "accel_q_gyro", &calibration->accel_q_gyro);
	read_json_vec3(dev1, "gyro_bias", &calibration->gyro_bias);
	read_json_quat(dev1, "gyro_q_mag", &calibration->gyro_q_mag);
	read_json_vec3(dev1, "mag_bias", &calibration->mag_bias);

	read_json_vec3(dev1, "scale_accel", &calibration->scale_accel);
	read_json_vec3(dev1, "scale_gyro", &calibration->scale_gyro);
	read_json_vec3(dev1, "scale_mag", &calibration->scale_mag);

	read_json_array(dev1, "imu_noises", 4, calibration->imu_noises);
}


/*
 *
 * Exported functions.
 *
 */
#include <stdio.h>

void
xreal_air_calibration_set_defaults(struct xreal_air_parsed_calibration *calibration)
{
	// Sane, non-degenerate IMU calibration used as a starting point (and as a fallback if the
	// factory calibration can't be parsed). Identity misalignment quaternions, unit scale and zero
	// bias mean read_sample_and_apply_calibration() passes the raw (already unit-converted) samples
	// through unchanged instead of multiplying them by an all-zero (calloc'd) calibration, which
	// would zero out the gyro/accel and freeze the 3DoF fusion.
	memset(calibration, 0, sizeof(*calibration));

	calibration->accel_q_gyro = (struct xrt_quat){0.0f, 0.0f, 0.0f, 1.0f};
	calibration->gyro_q_mag = (struct xrt_quat){0.0f, 0.0f, 0.0f, 1.0f};

	calibration->scale_accel = (struct xrt_vec3){1.0f, 1.0f, 1.0f};
	calibration->scale_gyro = (struct xrt_vec3){1.0f, 1.0f, 1.0f};
	calibration->scale_mag = (struct xrt_vec3){1.0f, 1.0f, 1.0f};
}

/*!
 * Locate the self-contained `"IMU": { ... }` object inside a (possibly larger, or partially
 * mis-assembled) calibration blob and return the byte range of the object VALUE (the `{ ... }`).
 *
 * The Xreal Air 2 Ultra returns a ~55 KB factory blob that bundles the per-eye display distortion
 * meshes AND the IMU calibration into one JSON document. In practice the segmented HID transfer of
 * that blob does not reassemble into a byte-0 valid JSON document on this device (the leading
 * `{"left_display":{"data":[` is missing and there is a zero-filled gap before the `{"FSN":...}`
 * root), so cJSON_ParseWithLength() over the whole buffer fails. The embedded `"IMU"` object itself
 * is intact, however, and its schema is exactly what parse_calibration_json() expects, so we scan
 * for it and parse just that sub-object. Returns false if no complete IMU object is present.
 */
static bool
find_imu_object(const char *buffer, size_t size, size_t *out_start, size_t *out_len)
{
	static const char needle[] = "\"IMU\"";
	const size_t needle_len = sizeof(needle) - 1;

	for (size_t i = 0; i + needle_len <= size; i++) {
		if (memcmp(buffer + i, needle, needle_len) != 0) {
			continue;
		}

		// Find the opening brace of the object value after "IMU":
		size_t j = i + needle_len;
		while (j < size && buffer[j] != '{') {
			// Bail if we run into another key before an object value.
			if (buffer[j] == '"') {
				break;
			}
			j++;
		}
		if (j >= size || buffer[j] != '{') {
			continue;
		}

		// Brace-match to the end of the object, respecting JSON strings and escapes.
		int depth = 0;
		bool in_string = false;
		bool escaped = false;
		for (size_t k = j; k < size; k++) {
			char c = buffer[k];
			if (in_string) {
				if (escaped) {
					escaped = false;
				} else if (c == '\\') {
					escaped = true;
				} else if (c == '"') {
					in_string = false;
				}
				continue;
			}
			if (c == '"') {
				in_string = true;
			} else if (c == '{') {
				depth++;
			} else if (c == '}') {
				depth--;
				if (depth == 0) {
					*out_start = j;
					*out_len = (k - j) + 1;
					return true;
				}
			}
		}
	}

	return false;
}

bool
xreal_air_parse_calibration_buffer(struct xreal_air_parsed_calibration *calibration, const char *buffer, size_t size)
{
	// Start from sane defaults so any field the factory blob omits keeps a non-degenerate value.
	xreal_air_calibration_set_defaults(calibration);

	// Fast path: the whole buffer is one valid JSON document (original Xreal Air).
	cJSON *root = cJSON_ParseWithLength(buffer, size);
	if (root) {
		cJSON *imu = cJSON_GetObjectItem(root, "IMU");
		if (imu) {
			cJSON *dev1 = cJSON_GetObjectItem(imu, "device_1");
			if (dev1) {
				parse_calibration_json(calibration, dev1);
				cJSON_Delete(root);
				return true;
			}
		}
		cJSON_Delete(root);
	}

	// Fallback (Xreal Air 2 Ultra): the blob isn't byte-0 valid JSON, but the embedded "IMU"
	// object is intact — extract and parse just that sub-object.
	size_t imu_start = 0, imu_len = 0;
	if (find_imu_object(buffer, size, &imu_start, &imu_len)) {
		cJSON *imu = cJSON_ParseWithLength(buffer + imu_start, imu_len);
		if (imu) {
			cJSON *dev1 = cJSON_GetObjectItem(imu, "device_1");
			if (dev1) {
				parse_calibration_json(calibration, dev1);
				cJSON_Delete(imu);
				return true;
			}
			cJSON_Delete(imu);
		}
	}

	return false;
}

#include <stdio.h>

bool
xreal_air_parse_sensor_packet(struct xreal_air_parsed_sensor *sensor,
                              const uint8_t *buffer,
                              size_t size,
                              size_t max_size)
{
	const uint8_t *start = buffer;

	if ((size != max_size) || (size < 64)) {
		return false;
	}

	if (buffer[0] != 1) {
		return false;
	}

	// Header
	skip(&buffer, 2);

	// Temperature
	read_i16(&buffer, &sensor->temperature);

	// Timestamp
	read_u64(&buffer, &sensor->timestamp);

	// Sample
	read_sample(&buffer, &sensor->sample);

	// Checksum
	skip(&buffer, 4);

	// Unknown, skip 6 bytes.
	skip(&buffer, 6);

	return (size_t)buffer - (size_t)start == 64;
}

bool
xreal_air_parse_sensor_control_data_packet(struct xreal_air_parsed_sensor_control_data *data,
                                           const uint8_t *buffer,
                                           size_t size,
                                           size_t max_size)
{
	const uint8_t *start = buffer;

	if ((size != max_size) || (size < 8)) {
		return false;
	}

	// Header
	skip(&buffer, 1);

	// Checksum
	skip(&buffer, 4);

	// Length
	read_u16(&buffer, &data->length);

	// MSGID
	read_u8(&buffer, &data->msgid);

	// Sensor control data depending on action
	read_u8_array(&buffer, data->data, size - 8);

	return (size_t)buffer - (size_t)start == max_size;
}

bool
xreal_air_parse_control_packet(struct xreal_air_parsed_control *control, const uint8_t *buffer, int size)
{
	const uint8_t *start = buffer;

	if (size != 64) {
		return false;
	}

	// Header
	skip(&buffer, 1);

	// Checksum
	skip(&buffer, 4);

	// Length
	read_u16(&buffer, &control->length);

	// Timestamp
	read_u64(&buffer, &control->timestamp);

	// Action
	read_u16(&buffer, &control->action);

	// Reserved, skip 5 bytes.
	skip(&buffer, 5);

	// Control data depending on action
	read_u8_array(&buffer, control->data, 42);

	return (size_t)buffer - (size_t)start == 64;
}
