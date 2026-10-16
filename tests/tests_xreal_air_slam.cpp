// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Xreal Air 6DoF: calibration conversion, timestamps, frame conventions and the 3DoF/SLAM
 *         hand-over.
 * @author Andrew Gaspar
 */

#include "xreal_air/xreal_air_slam_calib.h"
#include "xreal_air/xreal_air_pose_blend.h"
#include "xreal_air/xreal_air_slam_guard.h"

#include "math/m_api.h"
#include "math/m_space.h"
#include "math/m_vec3.h"

#include "catch_amalgamated.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

using Catch::Approx;

/*
 * The factory "SLAM_camera" of the user's Air 2 Ultra (serial ZBBM5DZFMP), verbatim, and what
 * the offline Basalt evaluation found to work (basalt-results/calib/kb4.json).
 */
static xreal_air_slam_camera_calibration
factory_camera(int i)
{
	xreal_air_slam_camera_calibration c;
	memset(&c, 0, sizeof(c));
	c.valid = true;
	snprintf(c.model, sizeof(c.model), "fisheye624");
	c.resolution[0] = 480;
	c.resolution[1] = 640;
	c.kc_count = 12;
	if (i == 0) {
		const double kc[12] = {0.005751675480784109,  0.0818962032992643,    -0.11146098365461432,
		                       0.06541732249597328,   -0.019857187578065906, 0.002484011292683035,
		                       -0.003304443228442813, 0.003939829503233638,  0.008407548576717877,
		                       0.0006605963655307348, -0.008203401546229887, -0.0007676734747295421};
		c.fc[0] = 240.42715086110638;
		c.fc[1] = 240.60562174255358;
		c.cc[0] = 231.19472025357308;
		c.cc[1] = 327.8882181804491;
		memcpy(c.kc, kc, sizeof(kc));
		const double p[3] = {-0.049491031609048906, 0.004457664969310794, 0.011466859006973978};
		const double q[4] = {0.07217791068230216, 0.055651249371278785, 0.0022432290073991982,
		                     0.9958354560752364};
		memcpy(c.imu_p_cam, p, sizeof(p));
		memcpy(c.imu_q_cam, q, sizeof(q));
	} else {
		const double kc[12] = {0.005507877519534051,   0.08576968615495836,    -0.11960663004987365,
		                       0.07314188987866171,    -0.023200075182788105,  0.0030297236115102346,
		                       -0.0022945518463509565, 0.0005620395680658569,  0.002801120564059989,
		                       0.00017153718461278917, -0.0009539523501527359, -7.145263146096477e-05};
		c.fc[0] = 239.70803683464794;
		c.fc[1] = 239.93453484632474;
		c.cc[0] = 240.65070845224014;
		c.cc[1] = 317.10695787136973;
		memcpy(c.kc, kc, sizeof(kc));
		const double p[3] = {0.08780929217766974, 0.004737136465894672, 0.012057744031620837};
		const double q[4] = {0.06520106272542181, -0.0397844854655278, -0.001214762624222925,
		                     0.9970780012052648};
		memcpy(c.imu_p_cam, p, sizeof(p));
		memcpy(c.imu_q_cam, q, sizeof(q));
	}
	return c;
}

static void
p16_of(const xreal_air_slam_camera_calibration &c, double p16[16])
{
	p16[0] = c.fc[0];
	p16[1] = c.fc[1];
	p16[2] = c.cc[0];
	p16[3] = c.cc[1];
	for (int i = 0; i < 12; i++) {
		p16[4 + i] = c.kc[i];
	}
}

TEST_CASE("xreal_air_slam_T_imu_cam")
{
	SECTION("JPL -> Hamilton is the conjugate, matches the offline-validated calibration")
	{
		// basalt-results/calib/kb4.json T_imu_cam, the only reading that tracked.
		const double expected_q[2][4] = {
		    {-0.072177910682, -0.055651249371, -0.002243229007, 0.995835456075},
		    {-0.065201062725, 0.039784485466, 0.001214762624, 0.997078001205},
		};
		const double expected_p[2][3] = {
		    {-0.049491031609, 0.004457664969, 0.011466859007},
		    {0.087809292178, 0.004737136466, 0.012057744032},
		};
		for (int i = 0; i < 2; i++) {
			const xreal_air_slam_camera_calibration c = factory_camera(i);
			double q[4], p[3];
			xreal_air_slam_T_imu_cam(&c, q, p);
			for (int k = 0; k < 4; k++) {
				CHECK(q[k] == Approx(expected_q[i][k]).margin(1e-9));
			}
			for (int k = 0; k < 3; k++) {
				CHECK(p[k] == Approx(expected_p[i][k]).margin(1e-9));
			}
		}
	}

	SECTION("Cameras look forward, pitched down and toed out")
	{
		// Rotate the camera's optical axis (+z) into the IMU frame (x right, y down, z forward).
		for (int i = 0; i < 2; i++) {
			const xreal_air_slam_camera_calibration c = factory_camera(i);
			double q[4], p[3];
			xreal_air_slam_T_imu_cam(&c, q, p);
			xrt_quat qf = {(float)q[0], (float)q[1], (float)q[2], (float)q[3]};
			xrt_vec3 z = {0, 0, 1}, axis;
			math_quat_rotate_vec3(&qf, &z, &axis);
			CHECK(axis.z > 0.98f); // forward
			CHECK(axis.y > 0.1f);  // down (y down), ~8 degrees
			// Left camera (device_1, -x side) toes out to the left, right one to the right.
			CHECK((i == 0 ? axis.x < -0.05f : axis.x > 0.05f));
			CHECK((i == 0 ? p[0] < 0 : p[0] > 0));
		}
	}
}

TEST_CASE("xreal_air_fisheye624")
{
	const xreal_air_slam_camera_calibration c = factory_camera(0);
	double p16[16];
	p16_of(c, p16);

	// Centre, edges and the far corners (which are beyond 90 degrees on these cameras).
	const double pixels[][2] = {{240, 320}, {0, 0}, {480, 640}, {0, 640}, {480, 0}, {100, 500}, {470, 10}};
	for (const auto &uv : pixels) {
		double th = 0, phi = 0;
		REQUIRE(xreal_air_fisheye624_unproject(p16, uv, &th, &phi));
		const double point[3] = {std::sin(th) * std::cos(phi), std::sin(th) * std::sin(phi), std::cos(th)};
		double back[2];
		xreal_air_fisheye624_project(p16, point, back);
		CHECK(back[0] == Approx(uv[0]).margin(1e-6));
		CHECK(back[1] == Approx(uv[1]).margin(1e-6));
	}

	double th = 0, phi = 0;
	const double corner[2] = {0, 0};
	REQUIRE(xreal_air_fisheye624_unproject(p16, corner, &th, &phi));
	CHECK(th * 180.0 / M_PI > 90.0); // The report saw ~95-97 degrees in the corners.
	CHECK(th * 180.0 / M_PI < 100.0);
}

TEST_CASE("xreal_air_kb4_fit")
{
	SECTION("Matches the offline fit (fit_kb4.py, 4 px grid) on both cameras")
	{
		// basalt-results/calib/kb4.json intrinsics, and the reported residuals.
		const double expected[2][8] = {
		    {240.03304005241142, 240.22760981195185, 232.0087198094196, 327.82202024652764,
		     0.024412750858634265, 0.02002509093412959, -0.02356441787538953, 0.004870802405877735},
		    {239.38820442121806, 239.61024454325607, 240.4433389611325, 317.1594782710674, 0.022528873308165342,
		     0.02472851624599719, -0.026852562266876615, 0.005693017938220507},
		};
		const double expected_rms[2] = {1.48, 0.60};
		for (int i = 0; i < 2; i++) {
			const xreal_air_slam_camera_calibration c = factory_camera(i);
			xreal_air_kb4 kb4;
			REQUIRE(xreal_air_kb4_fit(&c, 4.0, &kb4));
			CHECK(kb4.fx == Approx(expected[i][0]).margin(0.01));
			CHECK(kb4.fy == Approx(expected[i][1]).margin(0.01));
			CHECK(kb4.cx == Approx(expected[i][2]).margin(0.01));
			CHECK(kb4.cy == Approx(expected[i][3]).margin(0.01));
			for (int k = 0; k < 4; k++) {
				CHECK(kb4.k[k] == Approx(expected[i][4 + k]).margin(2e-4));
			}
			CHECK(kb4.rms_px == Approx(expected_rms[i]).margin(0.03));
		}
	}

	SECTION("The default grid is close to the dense one")
	{
		const xreal_air_slam_camera_calibration c = factory_camera(0);
		xreal_air_kb4 dense, fast;
		REQUIRE(xreal_air_kb4_fit(&c, 4.0, &dense));
		REQUIRE(xreal_air_kb4_fit(&c, 0.0, &fast));
		CHECK(fast.points < dense.points);
		CHECK(fast.rms_px < 2.0);
		CHECK(fast.max_px < 7.0);

		// Both models agree within a pixel inside the well-covered centre region.
		const xreal_air_slam_camera_calibration cam = factory_camera(0);
		double p16[16];
		p16_of(cam, p16);
		for (double u = 80; u <= 400; u += 40) {
			for (double v = 120; v <= 520; v += 40) {
				const double uv[2] = {u, v};
				double th, phi, a[2], b[2];
				REQUIRE(xreal_air_fisheye624_unproject(p16, uv, &th, &phi));
				xreal_air_kb4_project_angles(&dense, th, phi, a);
				xreal_air_kb4_project_angles(&fast, th, phi, b);
				CHECK(std::hypot(a[0] - b[0], a[1] - b[1]) < 0.2);
				CHECK(std::hypot(a[0] - u, a[1] - v) < 1.5);
			}
		}
	}

	SECTION("Refuses what is not a fisheye624 calibration")
	{
		xreal_air_slam_camera_calibration c = factory_camera(0);
		xreal_air_kb4 kb4;
		c.kc_count = 8;
		CHECK_FALSE(xreal_air_kb4_fit(&c, 0, &kb4));
		c = factory_camera(1);
		c.valid = false;
		CHECK_FALSE(xreal_air_kb4_fit(&c, 0, &kb4));
	}
}

TEST_CASE("xreal_air_slam_frame_timestamp")
{
	const int64_t t = 1077984014540;
	// The recorded exposure: 15.9275 ms, so the middle is 7.96375 ms after the device time.
	CHECK(xreal_air_slam_frame_timestamp(t, 15927500, 0) == t + 7963750);
	// No (warm-up) or unbelievable exposure: the offline optimum, +8 ms.
	CHECK(xreal_air_slam_frame_timestamp(t, 0, 0) == t + 8000000);
	CHECK(xreal_air_slam_frame_timestamp(t, 50000000, 0) == t + 8000000);
	// The user tweak adds on top.
	CHECK(xreal_air_slam_frame_timestamp(t, 15927500, -1000000) == t + 6963750);
}

static float
quat_angle_deg(const xrt_quat &a, const xrt_quat &b)
{
	float d = std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
	d = d > 1.0f ? 1.0f : d;
	return 2.0f * std::acos(d) * 180.0f / (float)M_PI;
}

static xrt_quat
axis_angle(float x, float y, float z, float deg)
{
	const xrt_vec3 axis = {x, y, z};
	xrt_quat q;
	math_quat_from_angle_vector(deg * (float)M_PI / 180.0f, &axis, &q);
	return q;
}

static xrt_space_relation
orientation_relation(const xrt_quat &q)
{
	xrt_space_relation r = XRT_SPACE_RELATION_ZERO;
	r.pose.orientation = q;
	r.relation_flags = (xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                              XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	return r;
}

static xrt_space_relation
pose_relation(const xrt_quat &q, const xrt_vec3 &p)
{
	xrt_space_relation r = orientation_relation(q);
	r.pose.position = p;
	r.relation_flags = XRT_SPACE_RELATION_BITMASK_ALL;
	return r;
}

TEST_CASE("xreal_air_basalt_to_head")
{
	SECTION("Basalt's initial pose (IMU forward along world +y) is looking down -Z, Y up")
	{
		// IMU x -> world x, IMU y (down) -> world -z, IMU z (forward) -> world +y: -90 deg about x
		// (what Basalt printed as T_w_i for the living-room-02 replay).
		const xrt_space_relation world_imu = pose_relation(axis_angle(1, 0, 0, -90), {0, 0, 0});
		xrt_space_relation head;
		xreal_air_basalt_to_head(&world_imu, &head);

		const xrt_quat identity = {0, 0, 0, 1};
		CHECK(quat_angle_deg(head.pose.orientation, identity) < 0.01f);
	}

	SECTION("Positions: Basalt up (+z) is OpenXR up (+y), Basalt forward (+y) is OpenXR -z")
	{
		const xrt_space_relation up = pose_relation(axis_angle(1, 0, 0, -90), {0.1f, 0, 0.5f});
		xrt_space_relation head;
		xreal_air_basalt_to_head(&up, &head);
		CHECK(head.pose.position.x == Approx(0.1f).margin(1e-6));
		CHECK(head.pose.position.y == Approx(0.5f).margin(1e-6));
		CHECK(head.pose.position.z == Approx(0.0f).margin(1e-6));

		const xrt_space_relation fwd = pose_relation(axis_angle(1, 0, 0, -90), {0, 0.3f, 0});
		xreal_air_basalt_to_head(&fwd, &head);
		CHECK(head.pose.position.z == Approx(-0.3f).margin(1e-6));
	}

	SECTION("Turning the head left (IMU yaw about -y, its up) is a positive rotation about OpenXR +Y")
	{
		// In the IMU frame up is -y; a left turn is +30 degrees about the up axis.
		xrt_quat init = axis_angle(1, 0, 0, -90), turn = axis_angle(0, -1, 0, 30), q;
		math_quat_rotate(&init, &turn, &q);
		const xrt_space_relation world_imu = pose_relation(q, {0, 0, 0});
		xrt_space_relation head;
		xreal_air_basalt_to_head(&world_imu, &head);
		CHECK(quat_angle_deg(head.pose.orientation, axis_angle(0, 1, 0, 30)) < 0.01f);
	}
}

TEST_CASE("xreal_air_pose_blend")
{
	const int64_t s = 1000000000;
	xreal_air_pose_blend b;
	xreal_air_pose_blend_init(&b);
	xrt_space_relation out;

	// 3DoF only: passes the fusion through, position at the origin.
	const xrt_quat fusion_q = axis_angle(0, 1, 0, 20);
	xrt_space_relation fusion = orientation_relation(fusion_q);
	xreal_air_pose_blend_update(&b, 10 * s, NULL, 0, &fusion, &out);
	CHECK(quat_angle_deg(out.pose.orientation, fusion_q) < 0.01f);
	CHECK(out.pose.position.x == 0.0f);
	CHECK_FALSE(b.slam_active);
	CHECK((out.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) == 0);

	// SLAM comes in with a different heading, a slightly different down and its own origin.
	xrt_quat slam_q;
	{
		xrt_quat yaw = axis_angle(0, 1, 0, -70), tilt = axis_angle(1, 0, 0, 2);
		math_quat_rotate(&tilt, &yaw, &slam_q);
	}
	const xrt_vec3 slam_p = {0.3f, 1.2f, -0.4f};
	xrt_space_relation slam = pose_relation(slam_q, slam_p);

	SECTION("Acquiring SLAM does not move the output")
	{
		xreal_air_pose_blend_update(&b, 10 * s, &slam, 0, &fusion, &out);
		CHECK(b.slam_active);
		CHECK(b.slam_acquired == 1);
		CHECK(quat_angle_deg(out.pose.orientation, fusion_q) < 0.05f);
		CHECK(out.pose.position.x == Approx(0).margin(1e-5));
		CHECK(out.pose.position.y == Approx(0).margin(1e-5));
		CHECK(out.pose.position.z == Approx(0).margin(1e-5));
		CHECK((out.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0);

		SECTION("SLAM motion comes through, rotated onto the output heading")
		{
			// Move 1 m along SLAM +x after the tilt fade; the output heading is SLAM's turned by
			// +90 degrees about Y, so that is -z in the output.
			xrt_space_relation moved = pose_relation(slam_q, {slam_p.x + 1.0f, slam_p.y, slam_p.z});
			xreal_air_pose_blend_update(&b, 12 * s, &moved, 0, &fusion, &out);
			CHECK(out.pose.position.x == Approx(0).margin(0.05));
			CHECK(out.pose.position.y == Approx(0).margin(0.05));
			CHECK(out.pose.position.z == Approx(-1.0).margin(0.05));

			// After the fade only a yaw correction is left: the output's down is SLAM's down.
			xrt_vec3 up = {0, 1, 0}, out_up, slam_up;
			xrt_quat inv_out, inv_slam;
			math_quat_invert(&out.pose.orientation, &inv_out);
			math_quat_invert(&slam_q, &inv_slam);
			math_quat_rotate_vec3(&inv_out, &up, &out_up);
			math_quat_rotate_vec3(&inv_slam, &up, &slam_up);
			CHECK(out_up.x == Approx(slam_up.x).margin(1e-4));
			CHECK(out_up.y == Approx(slam_up.y).margin(1e-4));
			CHECK(out_up.z == Approx(slam_up.z).margin(1e-4));
		}

		SECTION("A re-initialisation while tracking does not move the output")
		{
			xreal_air_pose_blend_update(&b, 12 * s, &slam, 0, &fusion, &out);
			const xrt_pose before = out.pose;
			xrt_space_relation reinit = pose_relation(axis_angle(0, 1, 0, 0), {0, 0, 0});
			xreal_air_pose_blend_update(&b, 12 * s + 1000, &reinit, 7, &fusion, &out);
			CHECK(b.slam_acquired == 2);
			CHECK(quat_angle_deg(out.pose.orientation, before.orientation) < 0.05f);
			CHECK(out.pose.position.x == Approx(before.position.x).margin(1e-4));
			CHECK(out.pose.position.y == Approx(before.position.y).margin(1e-4));
			CHECK(out.pose.position.z == Approx(before.position.z).margin(1e-4));
		}

		SECTION("The tilt correction fades, it does not snap")
		{
			xreal_air_pose_blend_update(&b, 10 * s + s / 2, &slam, 0, &fusion, &out);
			const float half = quat_angle_deg(out.pose.orientation, fusion_q);
			CHECK(half > 0.5f);
			CHECK(half < 1.5f);
		}

		SECTION("Losing SLAM holds the position and keeps the orientation")
		{
			xrt_space_relation moved = pose_relation(slam_q, {slam_p.x, slam_p.y + 0.5f, slam_p.z});
			xreal_air_pose_blend_update(&b, 13 * s, &moved, 0, &fusion, &out);
			const xrt_pose before = out.pose;

			// The fusion has its own idea of yaw by now.
			const xrt_quat drifted = axis_angle(0, 1, 0, 45);
			xrt_space_relation f2 = orientation_relation(drifted);
			xreal_air_pose_blend_update(&b, 13 * s + 1000, NULL, 0, &f2, &out);
			CHECK_FALSE(b.slam_active);
			CHECK(b.slam_lost == 1);
			CHECK(quat_angle_deg(out.pose.orientation, before.orientation) < 0.05f);
			CHECK(out.pose.position.y == Approx(before.position.y).margin(1e-5));
			CHECK(out.pose.position.y == Approx(0.5).margin(1e-3));
			CHECK((out.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) == 0);

			// Fusion rotation still comes through relative to the hand-over.
			xrt_quat turned_q, delta = axis_angle(0, 1, 0, 10);
			math_quat_rotate(&delta, &drifted, &turned_q);
			xrt_space_relation f3 = orientation_relation(turned_q);
			xreal_air_pose_blend_update(&b, 20 * s, NULL, 0, &f3, &out);
			const float turned = quat_angle_deg(out.pose.orientation, before.orientation);
			CHECK(turned == Approx(10.0f).margin(2.5f)); // 10 deg yaw, plus the faded-out tilt

			SECTION("Re-acquiring the same world starts in place, then glides back to the old alignment")
			{
				const xrt_pose last = out.pose;
				// The head moved 0.3 m along SLAM +y during the stall, SLAM saw it, the held
				// position did not.
				xrt_space_relation back =
				    pose_relation(slam_q, {slam_p.x, slam_p.y + 0.5f + 0.3f, slam_p.z});
				xreal_air_pose_blend_update(&b, 21 * s, &back, 0, &f3, &out);
				CHECK(b.slam_acquired == 2);
				CHECK(quat_angle_deg(out.pose.orientation, last.orientation) < 0.05f);
				CHECK(out.pose.position.y == Approx(last.position.y).margin(1e-4));

				// After the glide the room is where it was before the stall: SLAM +y is
				// output +y (the alignment's yaw keeps it vertical), 0.8 m above the start.
				xreal_air_pose_blend_update(&b, 21 * s + XREAL_AIR_POSE_BLEND_GLIDE_NS, &back, 0, &f3,
				                            &out);
				CHECK(out.pose.position.x == Approx(0).margin(0.05));
				CHECK(out.pose.position.y == Approx(0.8).margin(0.01));
				CHECK(out.pose.position.z == Approx(0).margin(0.05));
			}

			SECTION("Re-acquiring a new world (re-initialised SLAM) re-anchors for good")
			{
				const xrt_pose last = out.pose;
				xrt_space_relation again = pose_relation(axis_angle(0, 1, 0, 150), {5, 5, 5});
				xreal_air_pose_blend_update(&b, 21 * s, &again, 1, &f3, &out);
				CHECK(b.slam_acquired == 2);
				CHECK(quat_angle_deg(out.pose.orientation, last.orientation) < 0.05f);
				CHECK(out.pose.position.x == Approx(last.position.x).margin(1e-4));
				CHECK(out.pose.position.y == Approx(last.position.y).margin(1e-4));
				CHECK(out.pose.position.z == Approx(last.position.z).margin(1e-4));

				xreal_air_pose_blend_update(&b, 30 * s, &again, 1, &f3, &out);
				CHECK(out.pose.position.x == Approx(last.position.x).margin(1e-4));
				CHECK(out.pose.position.y == Approx(last.position.y).margin(1e-4));
				CHECK(out.pose.position.z == Approx(last.position.z).margin(1e-4));
			}
		}
	}
}

/*
 *
 * Trusting SLAM: IMU stillness, wear state, the pose guard, the position history.
 *
 */

static const int64_t MS = 1000000;

TEST_CASE("xreal_air_still")
{
	xreal_air_still st;
	xreal_air_still_init(&st, 0.03f, 0.15f);
	unsigned int seed = 7;
	auto noise = [&](float std) { return std * ((float)rand_r(&seed) / (float)RAND_MAX - 0.5f) * 3.46f; };

	SECTION("Lying on a desk: still after half a second")
	{
		bool still = false;
		int64_t t = 1000 * MS;
		for (int i = 0; i < 400; i++, t += MS) {
			const xrt_vec3 g = {noise(1e-3f), noise(1e-3f), noise(1e-3f)};
			const xrt_vec3 a = {0.3f + noise(0.014f), -7.5f + noise(0.014f), 6.3f + noise(0.014f)};
			still = xreal_air_still_push(&st, t, &g, &a);
		}
		CHECK_FALSE(still); // Not a full window yet.
		for (int i = 0; i < 200; i++, t += MS) {
			const xrt_vec3 g = {noise(1e-3f), noise(1e-3f), noise(1e-3f)};
			const xrt_vec3 a = {0.3f + noise(0.014f), -7.5f + noise(0.014f), 6.3f + noise(0.014f)};
			still = xreal_air_still_push(&st, t, &g, &a);
		}
		CHECK(still);

		SECTION("A head turning slowly is not still")
		{
			for (int i = 0; i < 100; i++, t += MS) {
				const xrt_vec3 g = {0, 0.1f, 0};
				const xrt_vec3 a = {0.3f, -7.5f, 6.3f};
				still = xreal_air_still_push(&st, t, &g, &a);
			}
			CHECK_FALSE(still);
		}

		SECTION("A gap in the samples starts over")
		{
			const xrt_vec3 g = {0, 0, 0};
			const xrt_vec3 a = {0.3f, -7.5f, 6.3f};
			for (int i = 0; i < 20; i++) {
				still = xreal_air_still_push(&st, t + 200 * MS + i * MS, &g, &a);
			}
			CHECK_FALSE(still);
		}
	}

	SECTION("Worn, nodding a little (accel swings): not still")
	{
		bool still = false;
		int64_t t = 1000 * MS;
		for (int i = 0; i < 1000; i++, t += MS) {
			const xrt_vec3 g = {0.01f, 0.01f, 0};
			const float swing = 0.3f * std::sin((float)i * 2.0f * (float)M_PI / 500.0f);
			const xrt_vec3 a = {0, -9.8f + swing, 0};
			still = xreal_air_still_push(&st, t, &g, &a);
		}
		CHECK_FALSE(still);
	}
}

TEST_CASE("xreal_air_wear")
{
	xreal_air_wear w;
	xreal_air_wear_init(&w);
	const int64_t debounce = 400 * MS;

	// Nothing known: worn.
	CHECK_FALSE(xreal_air_wear_update(&w, 1000 * MS, debounce));
	CHECK(w.worn);

	// The display coming on is no change.
	xreal_air_wear_raw(&w, true, 1000 * MS);
	CHECK_FALSE(xreal_air_wear_update(&w, 2000 * MS, debounce));
	CHECK(w.worn);

	// A flicker shorter than the debounce does nothing.
	xreal_air_wear_raw(&w, false, 3000 * MS);
	CHECK_FALSE(xreal_air_wear_update(&w, 3100 * MS, debounce));
	xreal_air_wear_raw(&w, true, 3200 * MS);
	CHECK_FALSE(xreal_air_wear_update(&w, 4000 * MS, debounce));
	CHECK(w.worn);

	// Taken off for good: reported once, after the debounce, with the time of the edge.
	xreal_air_wear_raw(&w, false, 5000 * MS);
	CHECK_FALSE(xreal_air_wear_update(&w, 5399 * MS, debounce));
	CHECK(xreal_air_wear_update(&w, 5400 * MS, debounce));
	CHECK_FALSE(w.worn);
	CHECK(w.worn_change_ns == 5000 * MS);
	CHECK_FALSE(xreal_air_wear_update(&w, 6000 * MS, debounce));

	// Repeated raw edges with the same state do not restart the debounce.
	xreal_air_wear_raw(&w, true, 7000 * MS);
	xreal_air_wear_raw(&w, true, 7300 * MS);
	CHECK(xreal_air_wear_update(&w, 7400 * MS, debounce));
	CHECK(w.worn);
}

//! A 30 Hz pose stream fed to a guard, with the pose query at a fixed latency after each pose.
struct guard_feed
{
	xreal_air_slam_guard g;
	int64_t t = 10000 * MS;
	xrt_pose pose = {{0, 0, 0, 1}, {0, 0, 0}};
	xrt_vec3 vel = {0, 0, 0};
	int64_t lag = 60 * MS;

	guard_feed()
	{
		xreal_air_slam_guard_config cfg;
		xreal_air_slam_guard_default_config(&cfg);
		xreal_air_slam_guard_init(&g, &cfg);
	}

	//! One frame later, moved by the velocity; returns the verdict, @p usable what the output may do.
	xreal_air_slam_pose_verdict
	step(bool imu_still, bool *usable = nullptr, int64_t dt = 33333333)
	{
		t += dt;
		const float s = (float)dt * 1e-9f;
		pose.position = {pose.position.x + vel.x * s, pose.position.y + vel.y * s, pose.position.z + vel.z * s};
		const xreal_air_slam_pose_verdict v = xreal_air_slam_guard_push(&g, t, &pose, &vel, imu_still);
		const bool u = xreal_air_slam_guard_usable(&g, t + lag, t);
		if (usable != nullptr) {
			*usable = u;
		}
		return v;
	}

	//! Steps until usable (at most 5 s), true if it got there.
	bool
	settle()
	{
		for (int i = 0; i < 150; i++) {
			bool u = false;
			step(false, &u);
			if (u) {
				return true;
			}
		}
		return false;
	}
};

TEST_CASE("xreal_air_slam_guard")
{
	guard_feed f;

	SECTION("A clean stream: warm-up, then usable, everything ok")
	{
		bool usable = true;
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_FIRST);
		CHECK_FALSE(usable);
		f.vel = {0.5f, 0, 0}; // Walking pace.
		int first_usable = -1;
		for (int i = 0; i < 90; i++) {
			CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_OK);
			if (usable && first_usable < 0) {
				first_usable = i;
			}
		}
		// 1.5 s warm-up at 30 Hz.
		CHECK(first_usable >= 44);
		CHECK(first_usable <= 46);
		CHECK(usable);
	}

	REQUIRE(f.settle());
	const uint32_t world = f.g.world_id;

	SECTION("The same pose again is not news")
	{
		CHECK(xreal_air_slam_guard_push(&f.g, f.t, &f.pose, &f.vel, false) == XREAL_AIR_SLAM_POSE_SAME);
	}

	SECTION("A stall: same world, warm-up again")
	{
		bool usable = true;
		CHECK(f.step(false, &usable, 2000 * MS) == XREAL_AIR_SLAM_POSE_RESUMED);
		CHECK_FALSE(usable);
		CHECK(f.g.world_id == world);
		CHECK(f.settle());
	}

	SECTION("One discontinuity is a re-initialisation: a new world, warm-up, then usable")
	{
		f.pose.position.x += 3.0f;
		bool usable = true;
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_REINIT);
		CHECK_FALSE(usable);
		CHECK(f.g.world_id == world + 1);
		CHECK(f.settle());

		SECTION("Another one much later is another re-initialisation")
		{
			for (int i = 0; i < 90; i++) {
				f.step(false);
			}
			f.pose.position.y += 1.0f;
			CHECK(f.step(false) == XREAL_AIR_SLAM_POSE_REINIT);
			CHECK(f.g.world_id == world + 2);
		}
	}

	SECTION("The live runaway: a jump every frame is a divergence, not a re-initialisation per frame")
	{
		// 2026-10-15 20:49: "pose jumped 0.25 m" (and more) on every 30 Hz pose for 7 minutes.
		f.vel = {9.0f, 0, 0};
		bool usable = true;
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_REINIT);
		CHECK_FALSE(usable);
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_DIVERGED);
		CHECK_FALSE(usable);
		for (int i = 0; i < 100; i++) {
			CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_DIVERGED); // Latched.
			CHECK_FALSE(usable);
		}
		CHECK(f.g.world_id == world + 1); // Not one world per frame.

		SECTION("A restart clears it, the next pose starts a new world")
		{
			xreal_air_slam_guard_restart(&f.g);
			f.vel = {0, 0, 0};
			f.pose.position = {0, 0, 0};
			CHECK(f.step(false) == XREAL_AIR_SLAM_POSE_FIRST);
			CHECK(f.g.world_id == world + 2);
			CHECK(f.settle());
		}
	}

	SECTION("Drifting away while the IMU is at rest (the glasses on the desk): never followed")
	{
		// Constant acceleration from rest, 0.56 m/s^2 as the live log's step sizes grew.
		const xrt_vec3 start = f.pose.position;
		bool usable = true;
		bool followed_far = false;
		xreal_air_slam_pose_verdict v = XREAL_AIR_SLAM_POSE_OK;
		float drift_when_held = -1;
		int frames = 0;
		while (v != XREAL_AIR_SLAM_POSE_DIVERGED && frames < 300) {
			f.vel.y += 0.56f / 30.0f;
			v = f.step(true, &usable);
			const float drift = m_vec3_len(m_vec3_sub(f.pose.position, start));
			if (!usable && drift_when_held < 0) {
				drift_when_held = drift;
			}
			if (usable && drift > 0.1f) {
				followed_far = true;
			}
			frames++;
		}
		CHECK(v == XREAL_AIR_SLAM_POSE_DIVERGED);
		CHECK_FALSE(followed_far);
		// The output let go at 0.1 m/s, ~1 cm of drift in; diverged 0.3 s later.
		CHECK(drift_when_held < 0.015f);
		CHECK(frames < 20);
		CHECK(f.g.suspect_since_ts > 0);
	}

	SECTION("The same speed while the IMU says the head moves is fine")
	{
		f.vel = {0, 0.5f, 0};
		for (int i = 0; i < 60; i++) {
			bool usable = false;
			CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_OK);
			CHECK(usable);
		}
	}

	SECTION("A short burst of speed is only suspicious, then usable again")
	{
		f.vel = {4.0f, 0, 0};
		bool usable = true;
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_SUSPECT);
		CHECK_FALSE(usable);
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_SUSPECT);
		f.vel = {0.5f, 0, 0};
		CHECK(f.step(false, &usable) == XREAL_AIR_SLAM_POSE_OK);
		CHECK(usable);
	}

	SECTION("Faster than a head for 200 ms: diverged")
	{
		f.vel = {4.0f, 0, 0};
		xreal_air_slam_pose_verdict v = XREAL_AIR_SLAM_POSE_OK;
		int frames = 0;
		while (v != XREAL_AIR_SLAM_POSE_DIVERGED && frames < 30) {
			v = f.step(false);
			frames++;
		}
		CHECK(v == XREAL_AIR_SLAM_POSE_DIVERGED);
		CHECK(frames == 8); // 7 frame intervals = 233 ms >= 200 ms.
	}

	SECTION("Latency hovering around the stale limit does not flap")
	{
		// The 2026-10-15 20:38 flapping: lost / acquired every few frames.
		int changes = 0;
		bool was = true;
		for (int i = 0; i < 300; i++) {
			f.lag = (i % 4 < 2 ? 280 : 320) * MS;
			bool usable = false;
			f.step(false, &usable);
			if (usable != was) {
				changes++;
				was = usable;
			}
		}
		CHECK(changes == 1); // Lost once, and not back while the lag stays above the fresh limit.

		// Back to a low latency: usable again after the hold time.
		f.lag = 60 * MS;
		int frames = 0;
		bool usable = false;
		while (!usable && frames < 100) {
			f.step(false, &usable);
			frames++;
		}
		CHECK(usable);
		CHECK(frames >= 15); // 500 ms.
		CHECK(frames <= 17);
	}
}

TEST_CASE("xreal_air_pos_history")
{
	xreal_air_pos_history h;
	xreal_air_pos_history_init(&h, 25 * MS);
	xrt_vec3 p = {9, 9, 9};
	CHECK_FALSE(xreal_air_pos_history_at(&h, 0, &p));

	// 11 ms queries, kept every 25 ms or more (so every third, 33 ms), over 5 s: the ring holds the
	// last 4.2 s.
	for (int i = 0; i <= 455; i++) {
		const xrt_vec3 pos = {(float)i, 0, 0};
		xreal_air_pos_history_push(&h, 1000 * MS + i * 11 * MS, &pos);
	}
	REQUIRE(xreal_air_pos_history_at(&h, 1000 * MS + 4000 * MS, &p));
	CHECK(p.x == Approx(362).margin(3)); // The newest kept at or before 4 s in (query 363, 3993 ms).
	REQUIRE(xreal_air_pos_history_at(&h, 1000 * MS + 5005 * MS, &p));
	CHECK(p.x >= 453.0f);
	// Older than everything kept: the oldest.
	REQUIRE(xreal_air_pos_history_at(&h, 0, &p));
	CHECK(p.x > 60.0f);
	CHECK(p.x < 80.0f);
}

TEST_CASE("xreal_air_pose_blend_hold")
{
	const int64_t s = 1000000000;
	xreal_air_pose_blend b;
	xreal_air_pose_blend_init(&b);
	xrt_space_relation out;
	const xrt_quat fusion_q = axis_angle(0, 1, 0, 20);
	xrt_space_relation fusion = orientation_relation(fusion_q);

	// Tracking, the head 0.4 m down from where it started (the glasses on their way to the desk).
	const xrt_quat slam_q = axis_angle(0, 1, 0, -70);
	xrt_space_relation slam = pose_relation(slam_q, {0, 0, 0});
	xreal_air_pose_blend_update(&b, 10 * s, &slam, 1, &fusion, &out);
	xrt_space_relation lowered = pose_relation(slam_q, {0, -0.4f, 0});
	xreal_air_pose_blend_update(&b, 12 * s, &lowered, 1, &fusion, &out);
	CHECK(out.pose.position.y == Approx(-0.4).margin(1e-3));
	const xrt_quat before = out.pose.orientation;

	// Hold where the head was before.
	const xrt_vec3 head = {0, 0, 0};
	xreal_air_pose_blend_hold(&b, 12 * s + 1000, &fusion, &head);
	CHECK_FALSE(b.slam_active);
	xreal_air_pose_blend_update(&b, 12 * s + 2000, NULL, 1, &fusion, &out);
	CHECK(out.pose.position.y == Approx(0).margin(1e-5));
	CHECK(quat_angle_deg(out.pose.orientation, before) < 0.05f);

	// A fresh tracker (new world) later anchors there, without a jump.
	xrt_space_relation fresh = pose_relation(axis_angle(0, 1, 0, 33), {2, 2, 2});
	xreal_air_pose_blend_update(&b, 30 * s, &fresh, 2, &fusion, &out);
	CHECK(out.pose.position.x == Approx(0).margin(1e-4));
	CHECK(out.pose.position.y == Approx(0).margin(1e-4));
	CHECK(out.pose.position.z == Approx(0).margin(1e-4));
}
