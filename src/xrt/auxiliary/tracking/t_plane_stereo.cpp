// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Dense stereo for plane detection: a fisheye stereo pair in, 3D points in the IMU frame out.
 * @author Andrew Gaspar
 * @ingroup aux_tracking
 */

#include "t_plane_stereo.h"
#include "t_plane_depth.h"

#include "math/m_api.h"
#include "os/os_time.h"

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/stereo.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

#include <cmath>
#include <cstring>
#include <vector>

namespace {

struct Mat3
{
	double m[3][3];
};

Mat3
mat_from_quat(const xrt_quat &q)
{
	xrt_matrix_3x3 r;
	math_matrix_3x3_from_quat(&q, &r);
	Mat3 out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.m[i][j] = r.v[i * 3 + j];
		}
	}
	return out;
}

Mat3
mul(const Mat3 &a, const Mat3 &b)
{
	Mat3 out = {};
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				out.m[i][j] += a.m[i][k] * b.m[k][j];
			}
		}
	}
	return out;
}

Mat3
transpose(const Mat3 &a)
{
	Mat3 out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.m[i][j] = a.m[j][i];
		}
	}
	return out;
}

void
apply(const Mat3 &a, const double v[3], double out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = a.m[i][0] * v[0] + a.m[i][1] * v[1] + a.m[i][2] * v[2];
	}
}

void
normalize(double v[3])
{
	const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	for (int i = 0; i < 3; i++) {
		v[i] /= l;
	}
}

void
cross3(const double a[3], const double b[3], double out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

//! Bilinear sample of a source pixel: top-left offset (or -1) and 8-bit fractional weights.
struct Tap
{
	int32_t offset;
	uint8_t fx, fy;
};

float
ms_since(timepoint_ns t0)
{
	return (float)((double)(os_monotonic_get_ns() - t0) / 1e6);
}

} // namespace

struct t_plane_stereo
{
	t_plane_depth_source base;
	t_plane_stereo_config cfg;
	int src_w = 0, src_h = 0;
	int W = 0, H = 0; //!< Full-resolution rectified.
	int w = 0, h = 0; //!< SGBM resolution.
	int scale = 1;
	double f = 0, cx = 0, cy = 0;
	double baseline = 0;
	Mat3 R_imu_rect;
	double t_imu_rect[3];
	std::vector<Tap> taps[2];
	cv::Ptr<cv::StereoSGBM> sgbm;
	cv::Mat full[2], small[2], padded[2], disp;
	std::vector<float> depth, rel_sigma;
	xrt_pose T_imu_rect_pose = XRT_POSE_IDENTITY;
};

extern "C" void
t_plane_stereo_default_config(struct t_plane_stereo_config *cfg)
{
	std::memset(cfg, 0, sizeof(*cfg));
	cfg->rect_width = 400;
	cfg->rect_height = 560;
	cfg->focal_px = 170.0f;
	cfg->half_res = true;
	cfg->num_disparities = 48;
	cfg->block_size = 7;
	cfg->pad_left = true;
	cfg->min_disparity_px = 2.0f;
}

static bool
stereo_compute(t_plane_depth_source *src,
               const xrt_frame *left,
               const xrt_frame *right,
               t_plane_depth_image *out,
               t_plane_depth_timing *out_timing);

static void
stereo_destroy(t_plane_depth_source *src)
{
	delete (t_plane_stereo *)src;
}

extern "C" int
t_plane_stereo_create_depth_source(const struct t_plane_stereo_config *cfg,
                                   const struct t_plane_stereo_camera cams[2],
                                   struct t_plane_depth_source **out_src)
{
	if (cfg->rect_width <= 0 || cfg->rect_height <= 0 || cfg->num_disparities <= 0 ||
	    cfg->num_disparities % 16 != 0 || cams[0].width != cams[1].width || cams[0].height != cams[1].height) {
		return -1;
	}

	t_plane_stereo *st = new t_plane_stereo();
	st->cfg = *cfg;
	st->src_w = cams[0].width;
	st->src_h = cams[0].height;
	st->scale = cfg->half_res ? 2 : 1;
	st->W = cfg->rect_width / st->scale * st->scale;
	st->H = cfg->rect_height / st->scale * st->scale;
	st->w = st->W / st->scale;
	st->h = st->H / st->scale;
	st->f = cfg->focal_px;
	st->cx = st->W / 2.0;
	st->cy = st->H / 2.0;

	// Extrinsics: R_c1_c0 and the right camera's center in the left camera's frame.
	const Mat3 R_imu_c0 = mat_from_quat(cams[0].T_imu_cam.orientation);
	const Mat3 R_imu_c1 = mat_from_quat(cams[1].T_imu_cam.orientation);
	const Mat3 R_c1_c0 = mul(transpose(R_imu_c1), R_imu_c0);
	const double d_imu[3] = {
	    (double)cams[1].T_imu_cam.position.x - cams[0].T_imu_cam.position.x,
	    (double)cams[1].T_imu_cam.position.y - cams[0].T_imu_cam.position.y,
	    (double)cams[1].T_imu_cam.position.z - cams[0].T_imu_cam.position.z,
	};
	double c1_in_c0[3];
	apply(transpose(R_imu_c0), d_imu, c1_in_c0);
	st->baseline = std::sqrt(c1_in_c0[0] * c1_in_c0[0] + c1_in_c0[1] * c1_in_c0[1] + c1_in_c0[2] * c1_in_c0[2]);

	// Rectified frame (in c0): x along the baseline, z the mean optical axis.
	double ex[3] = {c1_in_c0[0], c1_in_c0[1], c1_in_c0[2]};
	normalize(ex);
	const double z0[3] = {0, 0, 1};
	double z1[3];
	apply(transpose(R_c1_c0), z0, z1);
	double zf[3] = {z0[0] + z1[0], z0[1] + z1[1], z0[2] + z1[2]};
	normalize(zf);
	double ey[3], ez[3];
	cross3(zf, ex, ey);
	normalize(ey);
	cross3(ex, ey, ez);
	Mat3 R_rect_c0;
	for (int j = 0; j < 3; j++) {
		R_rect_c0.m[0][j] = ex[j];
		R_rect_c0.m[1][j] = ey[j];
		R_rect_c0.m[2][j] = ez[j];
	}
	const Mat3 R_c0_rect = transpose(R_rect_c0);
	st->R_imu_rect = mul(R_imu_c0, R_c0_rect);
	st->t_imu_rect[0] = cams[0].T_imu_cam.position.x;
	st->t_imu_rect[1] = cams[0].T_imu_cam.position.y;
	st->t_imu_rect[2] = cams[0].T_imu_cam.position.z;

	// Remap taps: rectified ray -> each camera -> its pixel.
	for (int c = 0; c < 2; c++) {
		st->taps[c].resize((size_t)st->W * (size_t)st->H);
		for (int v = 0; v < st->H; v++) {
			for (int u = 0; u < st->W; u++) {
				const double ray[3] = {(u - st->cx) / st->f, (v - st->cy) / st->f, 1.0};
				double p0[3], p[3];
				apply(R_c0_rect, ray, p0);
				if (c == 0) {
					std::memcpy(p, p0, sizeof(p));
				} else {
					apply(R_c1_c0, p0, p);
				}
				Tap tap = {-1, 0, 0};
				double uv[2];
				if (cams[c].project(cams[c].user, p, uv)) {
					const double x0 = std::floor(uv[0]), y0 = std::floor(uv[1]);
					if (x0 >= 0 && y0 >= 0 && x0 + 1 < st->src_w && y0 + 1 < st->src_h) {
						tap.offset = (int32_t)y0 * st->src_w + (int32_t)x0;
						tap.fx = (uint8_t)std::lround(std::min(255.0, (uv[0] - x0) * 256.0));
						tap.fy = (uint8_t)std::lround(std::min(255.0, (uv[1] - y0) * 256.0));
					}
				}
				st->taps[c][(size_t)v * st->W + u] = tap;
			}
		}
		st->full[c].create(st->H, st->W, CV_8UC1);
		st->small[c].create(st->h, st->w, CV_8UC1);
	}

	const int b = cfg->block_size;
	st->sgbm = cv::StereoSGBM::create(0, cfg->num_disparities, b, 8 * b * b, 32 * b * b, 1, 31, 10, 100, 2,
	                                  cv::StereoSGBM::MODE_SGBM);

	xrt_matrix_3x3 r;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			r.v[i * 3 + j] = (float)st->R_imu_rect.m[i][j];
		}
	}
	math_quat_from_matrix_3x3(&r, &st->T_imu_rect_pose.orientation);
	math_quat_normalize(&st->T_imu_rect_pose.orientation);
	st->T_imu_rect_pose.position = {(float)st->t_imu_rect[0], (float)st->t_imu_rect[1], (float)st->t_imu_rect[2]};
	st->depth.resize((size_t)st->w * st->h);
	st->rel_sigma.resize((size_t)st->w * st->h);

	st->base.compute = stereo_compute;
	st->base.destroy = stereo_destroy;
	*out_src = &st->base;
	return 0;
}

static void
rectify(t_plane_stereo &st, int c, const xrt_frame *xf)
{
	const uint8_t *src = xf->data;
	const size_t stride = xf->stride;
	const int sw = st.src_w;
	cv::Mat &dst = st.full[c];
	const Tap *taps = st.taps[c].data();
	for (int v = 0; v < st.H; v++) {
		uint8_t *row = dst.ptr<uint8_t>(v);
		for (int u = 0; u < st.W; u++) {
			const Tap &t = taps[(size_t)v * st.W + u];
			if (t.offset < 0) {
				row[u] = 0;
				continue;
			}
			const int y = t.offset / sw, x = t.offset % sw;
			const uint8_t *p = src + (size_t)y * stride + (size_t)x;
			const uint32_t a = p[0], b = p[1], cc = p[stride], d = p[stride + 1];
			const uint32_t fx = t.fx, fy = t.fy;
			const uint32_t top = a * (256 - fx) + b * fx;
			const uint32_t bot = cc * (256 - fx) + d * fx;
			row[u] = (uint8_t)((top * (256 - fy) + bot * fy + (1u << 15)) >> 16);
		}
	}

	if (st.scale == 1) {
		dst.copyTo(st.small[c]);
		return;
	}
	for (int v = 0; v < st.h; v++) {
		const uint8_t *r0 = dst.ptr<uint8_t>(2 * v);
		const uint8_t *r1 = dst.ptr<uint8_t>(2 * v + 1);
		uint8_t *o = st.small[c].ptr<uint8_t>(v);
		for (int u = 0; u < st.w; u++) {
			o[u] = (uint8_t)((r0[2 * u] + r0[2 * u + 1] + r1[2 * u] + r1[2 * u + 1] + 2) >> 2);
		}
	}
}

static bool
stereo_compute(t_plane_depth_source *src,
               const xrt_frame *left,
               const xrt_frame *right,
               t_plane_depth_image *out,
               t_plane_depth_timing *out_timing)
{
	t_plane_stereo *st = (t_plane_stereo *)src;
	t_plane_depth_timing timing = {};
	if (out_timing != nullptr) {
		*out_timing = timing;
	}
	if (left->format != XRT_FORMAT_L8 || right->format != XRT_FORMAT_L8 || (int)left->width != st->src_w ||
	    (int)left->height != st->src_h || (int)right->width != st->src_w || (int)right->height != st->src_h) {
		return false;
	}

	timepoint_ns t0 = os_monotonic_get_ns();
	rectify(*st, 0, left);
	rectify(*st, 1, right);
	timing.prepare_ms = ms_since(t0);

	t0 = os_monotonic_get_ns();
	const int pad = st->cfg.pad_left ? st->cfg.num_disparities : 0;
	cv::Mat d16;
	if (pad > 0) {
		cv::copyMakeBorder(st->small[0], st->padded[0], 0, 0, pad, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
		cv::copyMakeBorder(st->small[1], st->padded[1], 0, 0, pad, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
		st->sgbm->compute(st->padded[0], st->padded[1], st->disp);
		d16 = st->disp(cv::Rect(pad, 0, st->w, st->h));
	} else {
		st->sgbm->compute(st->small[0], st->small[1], st->disp);
		d16 = st->disp;
	}

	// Disparity (at the SGBM resolution, 1/16 px) -> depth; the confidence in full-resolution pixels.
	const double fb = st->f * st->baseline;
	const double s = st->scale;
	for (int v = 0; v < st->h; v++) {
		const int16_t *row = d16.ptr<int16_t>(v);
		float *depth = &st->depth[(size_t)v * st->w];
		float *sigma = &st->rel_sigma[(size_t)v * st->w];
		for (int u = 0; u < st->w; u++) {
			const double d = s * row[u] / 16.0;
			if (row[u] <= 0 || d <= st->cfg.min_disparity_px) {
				depth[u] = 0.0f;
				sigma[u] = 0.0f;
				continue;
			}
			depth[u] = (float)(fb / d);
			sigma[u] = (float)(0.5 / d);
		}
	}
	timing.match_ms = ms_since(t0);

	// Pixel (u, v) of the SGBM grid is full-resolution pixel (s u, s v).
	out->width = (uint32_t)st->w;
	out->height = (uint32_t)st->h;
	out->fx = out->fy = (float)(st->f / s);
	out->cx = (float)(st->cx / s);
	out->cy = (float)(st->cy / s);
	out->T_imu_cam = st->T_imu_rect_pose;
	out->depth = st->depth.data();
	out->rel_sigma = st->rel_sigma.data();
	if (out_timing != nullptr) {
		*out_timing = timing;
	}
	return true;
}
