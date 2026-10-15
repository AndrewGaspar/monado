// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 2 Ultra SLAM calibration conversion.
 * @author Andrew Gaspar
 * @ingroup drv_xreal_air
 */

#include "xreal_air_slam_calib.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define KB4_PARAMS 8
#define NEWTON_ITERATIONS 50
#define LM_MAX_ITERATIONS 100


/*
 *
 * Extrinsics and timestamps.
 *
 */

void
xreal_air_slam_T_imu_cam(const struct xreal_air_slam_camera_calibration *cam, double out_q_xyzw[4], double out_p[3])
{
	const double *q = cam->imu_q_cam;
	double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n < 1e-9) {
		n = 1.0;
	}

	// JPL -> Hamilton: same components, conjugated.
	out_q_xyzw[0] = -q[0] / n;
	out_q_xyzw[1] = -q[1] / n;
	out_q_xyzw[2] = -q[2] / n;
	out_q_xyzw[3] = q[3] / n;

	for (int i = 0; i < 3; i++) {
		out_p[i] = cam->imu_p_cam[i];
	}
}

int64_t
xreal_air_slam_frame_timestamp(int64_t left_start_ns, uint32_t exposure_ns, int64_t extra_offset_ns)
{
	int64_t half = XREAL_AIR_SLAM_DEFAULT_HALF_EXPOSURE_NS;
	if (exposure_ns > 0 && exposure_ns <= XREAL_AIR_SLAM_MAX_EXPOSURE_NS) {
		half = (int64_t)exposure_ns / 2;
	}
	return left_start_ns + half + extra_offset_ns;
}


/*
 *
 * fisheye624.
 *
 */

//! The radial polynomial theta * (1 + k0 theta^2 + ... + k5 theta^12), and its derivative.
static double
f624_radial(const double *k, double th, double *out_d)
{
	const double t2 = th * th;
	const double poly = 1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * (k[3] + t2 * (k[4] + t2 * k[5])))));
	if (out_d != NULL) {
		*out_d =
		    1 + t2 * (3 * k[0] +
		              t2 * (5 * k[1] + t2 * (7 * k[2] + t2 * (9 * k[3] + t2 * (11 * k[4] + t2 * 13 * k[5])))));
	}
	return th * poly;
}

//! Tangential + thin prism distortion of the distorted radial coordinate (xr, yr), and its Jacobian.
static void
f624_tangential(const double p16[16], double xr, double yr, double out[2], double J[2][2])
{
	const double p0 = p16[10], p1 = p16[11];
	const double s0 = p16[12], s1 = p16[13], s2 = p16[14], s3 = p16[15];
	const double rd2 = xr * xr + yr * yr;

	out[0] = xr + (2 * xr * xr + rd2) * p0 + 2 * xr * yr * p1 + s0 * rd2 + s1 * rd2 * rd2;
	out[1] = yr + (2 * yr * yr + rd2) * p1 + 2 * xr * yr * p0 + s2 * rd2 + s3 * rd2 * rd2;

	if (J != NULL) {
		J[0][0] = 1 + 6 * xr * p0 + 2 * yr * p1 + 2 * xr * s0 + 4 * xr * rd2 * s1;
		J[0][1] = 2 * yr * p0 + 2 * xr * p1 + 2 * yr * s0 + 4 * yr * rd2 * s1;
		J[1][0] = 2 * xr * p1 + 2 * yr * p0 + 2 * xr * s2 + 4 * xr * rd2 * s3;
		J[1][1] = 1 + 6 * yr * p1 + 2 * xr * p0 + 2 * yr * s2 + 4 * yr * rd2 * s3;
	}
}

void
xreal_air_fisheye624_project(const double p16[16], const double point[3], double out_uv[2])
{
	const double x = point[0], y = point[1], z = point[2];
	const double r = sqrt(x * x + y * y);
	const double th = atan2(r, z);
	const double cos_phi = r > 1e-12 ? x / r : 1.0;
	const double sin_phi = r > 1e-12 ? y / r : 0.0;

	const double td = f624_radial(&p16[4], th, NULL);
	double d[2];
	f624_tangential(p16, td * cos_phi, td * sin_phi, d, NULL);

	out_uv[0] = p16[0] * d[0] + p16[2];
	out_uv[1] = p16[1] * d[1] + p16[3];
}

bool
xreal_air_fisheye624_unproject(const double p16[16], const double uv[2], double *out_theta, double *out_phi)
{
	const double target[2] = {(uv[0] - p16[2]) / p16[0], (uv[1] - p16[3]) / p16[1]};

	// Undo the tangential + thin prism part: Newton on (xr, yr).
	double q[2] = {target[0], target[1]};
	double err = 1.0;
	for (int i = 0; i < NEWTON_ITERATIONS; i++) {
		double d[2], J[2][2];
		f624_tangential(p16, q[0], q[1], d, J);
		const double e0 = d[0] - target[0];
		const double e1 = d[1] - target[1];
		err = fabs(e0) + fabs(e1);
		if (err < 1e-14) {
			break;
		}
		const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
		if (fabs(det) < 1e-12) {
			return false;
		}
		q[0] -= (J[1][1] * e0 - J[0][1] * e1) / det;
		q[1] -= (-J[1][0] * e0 + J[0][0] * e1) / det;
	}
	if (err > 1e-9) {
		return false;
	}

	// Undo the radial polynomial: Newton on theta.
	const double td = sqrt(q[0] * q[0] + q[1] * q[1]);
	double th = td;
	double f = 1.0;
	for (int i = 0; i < NEWTON_ITERATIONS; i++) {
		double df = 0;
		f = f624_radial(&p16[4], th, &df) - td;
		if (fabs(f) < 1e-14) {
			break;
		}
		if (fabs(df) < 1e-12) {
			return false;
		}
		th -= f / df;
	}
	if (fabs(f) > 1e-9 || !(th >= 0.0) || th > M_PI) {
		return false;
	}

	*out_theta = th;
	*out_phi = atan2(q[1], q[0]);
	return true;
}


/*
 *
 * KB4 fit.
 *
 */

//! x = fx fy cx cy k1 k2 k3 k4
static void
kb4_eval(const double x[KB4_PARAMS], double th, double phi, double uv[2], double J[2][KB4_PARAMS])
{
	const double t2 = th * th;
	const double t3 = t2 * th, t5 = t3 * t2, t7 = t5 * t2, t9 = t7 * t2;
	const double d = th + x[4] * t3 + x[5] * t5 + x[6] * t7 + x[7] * t9;
	const double c = cos(phi), s = sin(phi);

	uv[0] = x[0] * d * c + x[2];
	uv[1] = x[1] * d * s + x[3];

	if (J != NULL) {
		memset(J, 0, sizeof(double) * 2 * KB4_PARAMS);
		J[0][0] = d * c;
		J[0][2] = 1;
		J[1][1] = d * s;
		J[1][3] = 1;
		const double pw[4] = {t3, t5, t7, t9};
		for (int i = 0; i < 4; i++) {
			J[0][4 + i] = x[0] * c * pw[i];
			J[1][4 + i] = x[1] * s * pw[i];
		}
	}
}

void
xreal_air_kb4_project_angles(const struct xreal_air_kb4 *kb4, double theta, double phi, double out_uv[2])
{
	const double x[KB4_PARAMS] = {kb4->fx, kb4->fy, kb4->cx, kb4->cy, kb4->k[0], kb4->k[1], kb4->k[2], kb4->k[3]};
	kb4_eval(x, theta, phi, out_uv, NULL);
}

//! Solve A x = b for an n x n system (n <= KB4_PARAMS) by Gaussian elimination with partial pivoting.
static bool
solve_linear(int n, double A[KB4_PARAMS][KB4_PARAMS], double b[KB4_PARAMS], double out[KB4_PARAMS])
{
	for (int col = 0; col < n; col++) {
		int pivot = col;
		for (int r = col + 1; r < n; r++) {
			if (fabs(A[r][col]) > fabs(A[pivot][col])) {
				pivot = r;
			}
		}
		if (fabs(A[pivot][col]) < 1e-300) {
			return false;
		}
		if (pivot != col) {
			for (int c = 0; c < n; c++) {
				const double tmp = A[col][c];
				A[col][c] = A[pivot][c];
				A[pivot][c] = tmp;
			}
			const double tmp = b[col];
			b[col] = b[pivot];
			b[pivot] = tmp;
		}
		for (int r = col + 1; r < n; r++) {
			const double f = A[r][col] / A[col][col];
			for (int c = col; c < n; c++) {
				A[r][c] -= f * A[col][c];
			}
			b[r] -= f * b[col];
		}
	}
	for (int r = n - 1; r >= 0; r--) {
		double sum = b[r];
		for (int c = r + 1; c < n; c++) {
			sum -= A[r][c] * out[c];
		}
		out[r] = sum / A[r][r];
	}
	return true;
}

struct fit_point
{
	double u, v, th, phi;
};

static double
fit_cost(const double x[KB4_PARAMS], const struct fit_point *pts, int count)
{
	double cost = 0;
	for (int i = 0; i < count; i++) {
		double uv[2];
		kb4_eval(x, pts[i].th, pts[i].phi, uv, NULL);
		const double du = uv[0] - pts[i].u, dv = uv[1] - pts[i].v;
		cost += du * du + dv * dv;
	}
	return cost;
}

bool
xreal_air_kb4_fit(const struct xreal_air_slam_camera_calibration *cam, double step_px, struct xreal_air_kb4 *out)
{
	if (!cam->valid || cam->kc_count != XREAL_AIR_SLAM_CAMERA_MAX_KC || cam->fc[0] <= 0 || cam->fc[1] <= 0 ||
	    cam->resolution[0] <= 0 || cam->resolution[1] <= 0) {
		return false;
	}
	if (!(step_px > 0)) {
		step_px = XREAL_AIR_KB4_FIT_DEFAULT_STEP_PX;
	}

	double p16[16] = {cam->fc[0], cam->fc[1], cam->cc[0], cam->cc[1]};
	for (int i = 0; i < XREAL_AIR_SLAM_CAMERA_MAX_KC; i++) {
		p16[4 + i] = cam->kc[i];
	}

	// The whole image, corners included (no hard fisheye circle on these cameras).
	const double w = cam->resolution[0], h = cam->resolution[1];
	const int nu = (int)floor(w / step_px + 1e-9) + 1;
	const int nv = (int)floor(h / step_px + 1e-9) + 1;
	struct fit_point *pts = calloc((size_t)nu * (size_t)nv, sizeof(*pts));
	if (pts == NULL) {
		return false;
	}

	int count = 0;
	for (int j = 0; j < nv; j++) {
		for (int i = 0; i < nu; i++) {
			const double uv[2] = {i * step_px, j * step_px};
			double th = 0, phi = 0;
			if (!xreal_air_fisheye624_unproject(p16, uv, &th, &phi)) {
				continue;
			}
			pts[count++] = (struct fit_point){uv[0], uv[1], th, phi};
		}
	}
	if (count < 100) {
		free(pts);
		return false;
	}

	double x[KB4_PARAMS] = {cam->fc[0], cam->fc[1], cam->cc[0], cam->cc[1], 0, 0, 0, 0};
	double cost = fit_cost(x, pts, count);
	double lambda = 1e-3;
	int it = 0;
	for (; it < LM_MAX_ITERATIONS; it++) {
		double A[KB4_PARAMS][KB4_PARAMS] = {{0}};
		double g[KB4_PARAMS] = {0};
		for (int n = 0; n < count; n++) {
			double uv[2], J[2][KB4_PARAMS];
			kb4_eval(x, pts[n].th, pts[n].phi, uv, J);
			const double r[2] = {uv[0] - pts[n].u, uv[1] - pts[n].v};
			for (int row = 0; row < 2; row++) {
				for (int a = 0; a < KB4_PARAMS; a++) {
					g[a] += J[row][a] * r[row];
					for (int b = 0; b < KB4_PARAMS; b++) {
						A[a][b] += J[row][a] * J[row][b];
					}
				}
			}
		}

		bool improved = false;
		double step_norm = 0, x_norm = 0;
		while (lambda < 1e12) {
			double M[KB4_PARAMS][KB4_PARAMS];
			double rhs[KB4_PARAMS];
			for (int a = 0; a < KB4_PARAMS; a++) {
				for (int b = 0; b < KB4_PARAMS; b++) {
					M[a][b] = A[a][b];
				}
				M[a][a] += lambda * A[a][a];
				rhs[a] = -g[a];
			}
			double delta[KB4_PARAMS];
			if (!solve_linear(KB4_PARAMS, M, rhs, delta)) {
				lambda *= 10;
				continue;
			}
			double xn[KB4_PARAMS];
			step_norm = 0;
			x_norm = 0;
			for (int a = 0; a < KB4_PARAMS; a++) {
				xn[a] = x[a] + delta[a];
				step_norm += delta[a] * delta[a];
				x_norm += x[a] * x[a];
			}
			const double cn = fit_cost(xn, pts, count);
			if (cn < cost) {
				memcpy(x, xn, sizeof(x));
				cost = cn;
				lambda *= 0.3;
				improved = true;
				break;
			}
			lambda *= 10;
		}
		if (!improved || sqrt(step_norm) < 1e-12 * (1 + sqrt(x_norm))) {
			break;
		}
	}

	double sum2 = 0, max = 0;
	for (int n = 0; n < count; n++) {
		double uv[2];
		kb4_eval(x, pts[n].th, pts[n].phi, uv, NULL);
		const double e = hypot(uv[0] - pts[n].u, uv[1] - pts[n].v);
		sum2 += e * e;
		if (e > max) {
			max = e;
		}
	}
	free(pts);

	out->fx = x[0];
	out->fy = x[1];
	out->cx = x[2];
	out->cy = x[3];
	for (int i = 0; i < 4; i++) {
		out->k[i] = x[4 + i];
	}
	out->rms_px = sqrt(sum2 / count);
	out->max_px = max;
	out->points = count;
	out->iterations = it;

	return isfinite(out->rms_px) && out->rms_px < 10.0;
}
