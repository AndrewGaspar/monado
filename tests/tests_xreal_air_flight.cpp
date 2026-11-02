// Copyright 2026, Andrew Gaspar
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tests for the Xreal Air flight recorder: the overwriting ring and the control files.
 */

#include "os/os_time.h"
#include "util/u_frame.h"
#include "xrt/xrt_frame.h"

#include "xreal_air/xreal_air_flight.h"
#include "xreal_air/xreal_air_flight_ring.h"

#include "catch_amalgamated.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>


/*
 *
 * The ring.
 *
 */

namespace {

struct Rec
{
	uint64_t seq;
	uint64_t check[5]; //!< seq * (i + 1), torn records do not match.
	uint8_t tail[3];   //!< Not a whole word.
};

Rec
make_rec(uint64_t seq)
{
	Rec r{};
	r.seq = seq;
	for (int i = 0; i < 5; i++) {
		r.check[i] = seq * (uint64_t)(i + 1);
	}
	r.tail[0] = (uint8_t)seq;
	r.tail[1] = (uint8_t)(seq >> 8);
	r.tail[2] = 0xAB;
	return r;
}

bool
rec_ok(const Rec &r)
{
	for (int i = 0; i < 5; i++) {
		if (r.check[i] != r.seq * (uint64_t)(i + 1)) {
			return false;
		}
	}
	return r.tail[0] == (uint8_t)r.seq && r.tail[1] == (uint8_t)(r.seq >> 8) && r.tail[2] == 0xAB;
}

} // namespace

TEST_CASE("flight_ring_basics")
{
	CHECK(xreal_air_flight_ring_create(sizeof(Rec), 0) == nullptr);

	xreal_air_flight_ring *ring = xreal_air_flight_ring_create(sizeof(Rec), 5);
	REQUIRE(ring != nullptr);
	CHECK(xreal_air_flight_ring_capacity(ring) == 8);
	CHECK(xreal_air_flight_ring_head(ring) == 0);
	CHECK(xreal_air_flight_ring_oldest(ring, 0) == 0);

	Rec out{};
	CHECK(xreal_air_flight_ring_read(ring, 0, &out) == XREAL_AIR_FLIGHT_RING_NOT_YET);

	for (uint64_t i = 0; i < 5; i++) {
		const Rec r = make_rec(i);
		xreal_air_flight_ring_push(ring, &r);
	}
	CHECK(xreal_air_flight_ring_head(ring) == 5);
	for (uint64_t i = 0; i < 5; i++) {
		REQUIRE(xreal_air_flight_ring_read(ring, i, &out) == XREAL_AIR_FLIGHT_RING_OK);
		CHECK(out.seq == i);
		CHECK(rec_ok(out));
	}
	CHECK(xreal_air_flight_ring_read(ring, 5, &out) == XREAL_AIR_FLIGHT_RING_NOT_YET);

	// Wrap: 20 pushed into 8 slots. The slot of index 12 is being reused by the next push (20), so
	// only 13..19 are readable.
	for (uint64_t i = 5; i < 20; i++) {
		const Rec r = make_rec(i);
		xreal_air_flight_ring_push(ring, &r);
	}
	CHECK(xreal_air_flight_ring_read(ring, 0, &out) == XREAL_AIR_FLIGHT_RING_OVERWRITTEN);
	CHECK(xreal_air_flight_ring_read(ring, 12, &out) == XREAL_AIR_FLIGHT_RING_OVERWRITTEN);
	for (uint64_t i = 13; i < 20; i++) {
		REQUIRE(xreal_air_flight_ring_read(ring, i, &out) == XREAL_AIR_FLIGHT_RING_OK);
		CHECK(out.seq == i);
		CHECK(rec_ok(out));
	}
	CHECK(xreal_air_flight_ring_oldest(ring, 0) == 12);
	CHECK(xreal_air_flight_ring_oldest(ring, 2) == 14);
	CHECK(xreal_air_flight_ring_oldest(ring, 100) == 20);

	xreal_air_flight_ring_destroy(&ring);
	CHECK(ring == nullptr);
}

TEST_CASE("flight_ring_concurrent_reader_never_sees_torn_records")
{
	// A small ring and a producer that laps the reader all the time: every read is either a whole,
	// correct record or reported as overwritten, and what was read plus what was lost adds up.
	xreal_air_flight_ring *ring = xreal_air_flight_ring_create(sizeof(Rec), 64);
	REQUIRE(ring != nullptr);

	constexpr uint64_t total = 2000000;
	std::atomic<bool> done{false};
	std::thread producer([&] {
		for (uint64_t i = 0; i < total; i++) {
			const Rec r = make_rec(i);
			xreal_air_flight_ring_push(ring, &r);
		}
		done = true;
	});

	uint64_t cursor = 0, read = 0, lost = 0, torn = 0, wrong = 0;
	while (cursor < total) {
		Rec out{};
		switch (xreal_air_flight_ring_read(ring, cursor, &out)) {
		case XREAL_AIR_FLIGHT_RING_NOT_YET: std::this_thread::yield(); break;
		case XREAL_AIR_FLIGHT_RING_OVERWRITTEN: {
			uint64_t next = xreal_air_flight_ring_oldest(ring, 8);
			next = next > cursor ? next : cursor + 1;
			lost += next - cursor;
			cursor = next;
			break;
		}
		case XREAL_AIR_FLIGHT_RING_OK:
			torn += rec_ok(out) ? 0 : 1;
			wrong += out.seq == cursor ? 0 : 1;
			read++;
			cursor++;
			break;
		}
	}
	producer.join();

	CHECK(torn == 0);
	CHECK(wrong == 0);
	CHECK(read + lost == total);
	CHECK(read > 0);
	INFO("read " << read << " lost " << lost);
	xreal_air_flight_ring_destroy(&ring);
}


/*
 *
 * The recorder, through its control files.
 *
 */

namespace {

struct TempDir
{
	std::string dir;
	TempDir()
	{
		const char *base = std::getenv("TMPDIR");
		std::string tmpl = std::string(base != nullptr ? base : "/tmp") + "/xreal_air_flight_XXXXXX";
		std::vector<char> buf(tmpl.begin(), tmpl.end());
		buf.push_back('\0');
		REQUIRE(mkdtemp(buf.data()) != nullptr);
		dir = buf.data();
	}
	~TempDir()
	{
		std::string cmd = "rm -rf '" + dir + "'";
		(void)std::system(cmd.c_str());
	}
};

std::string
slurp(const std::string &path)
{
	std::ifstream in(path, std::ios::binary);
	std::stringstream ss;
	ss << in.rdbuf();
	return ss.str();
}

std::map<std::string, std::string>
key_values(const std::string &text)
{
	std::map<std::string, std::string> kv;
	std::istringstream in(text);
	std::string line;
	while (std::getline(in, line)) {
		const auto eq = line.find('=');
		if (eq != std::string::npos) {
			kv[line.substr(0, eq)] = line.substr(eq + 1);
		}
	}
	return kv;
}

size_t
count_lines(const std::string &path)
{
	std::ifstream in(path);
	size_t n = 0;
	std::string line;
	while (std::getline(in, line)) {
		n++;
	}
	return n;
}

struct Recorder
{
	TempDir tmp;
	std::string ctl;
	xreal_air_flight *f = nullptr;
	int ids = 0;

	explicit Recorder(uint32_t frames_s = 0, uint32_t min_free_mb = 0)
	{
		ctl = tmp.dir + "/ctl";
		xreal_air_flight_config cfg{};
		cfg.log_level = U_LOGGING_WARN;
		snprintf(cfg.ctl_dir, sizeof(cfg.ctl_dir), "%s", ctl.c_str());
		cfg.lookback_s = 2;
		cfg.frames_lookback_s = frames_s;
		cfg.min_free_mb = min_free_mb;
		cfg.max_mb = 1024;
		cfg.default_max_s = 60;
		cfg.writer_nice = 0;
		f = xreal_air_flight_create(&cfg);
		REQUIRE(f != nullptr);
		CHECK(std::string(xreal_air_flight_ctl_dir(f)) == ctl);
	}

	~Recorder()
	{
		xreal_air_flight_destroy(&f);
	}

	//! Drop a command file the way the tool does (write elsewhere, rename in), wait for its answer.
	std::map<std::string, std::string>
	command(const std::string &name, const std::string &body)
	{
		const std::string id = "t" + std::to_string(++ids);
		const std::string tmp_path = ctl + "/." + name + ".tmp";
		{
			std::ofstream out(tmp_path);
			out << body << "\nid=" << id << "\n";
		}
		REQUIRE(rename(tmp_path.c_str(), (ctl + "/" + name).c_str()) == 0);

		for (int i = 0; i < 300; i++) {
			auto kv = key_values(slurp(ctl + "/status"));
			if (kv["cmd_id"] == id) {
				return kv;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		FAIL("no answer to " << name);
		return {};
	}

	std::map<std::string, std::string>
	wait_state(const std::string &state)
	{
		for (int i = 0; i < 300; i++) {
			auto kv = key_values(slurp(ctl + "/status"));
			if (kv["state"] == state) {
				return kv;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		FAIL("state never became " << state);
		return {};
	}
};

//! One "sensor tick": an IMU sample, every 33rd tick a stereo pair with its two telemetry rows.
struct Feeder
{
	xreal_air_flight *f;
	uint64_t tick = 0;
	uint64_t pairs = 0;

	void
	step()
	{
		const int64_t now = (int64_t)os_monotonic_get_ns();

		xreal_air_record_imu imu{};
		imu.host_ns = now;
		imu.device_ts = (uint64_t)now;
		imu.fusion_ns = now;
		imu.euroc_ns = now;
		imu.gyro = {0.1f, 0.2f, 0.3f};
		imu.accel = {0.0f, 9.81f, 0.0f};
		xreal_air_flight_push_imu(f, &imu);

		xreal_air_flight_head head{};
		head.query_ns = now;
		head.at_ns = now + 10000000;
		head.pose.orientation.w = 1.0f;
		head.fusion.w = 1.0f;
		head.slam = true;
		xreal_air_flight_push_head(f, &head);

		if (tick % 33 == 0) {
			xrt_frame *frames[2] = {nullptr, nullptr};
			for (int c = 0; c < 2; c++) {
				u_frame_create_one_off(XRT_FORMAT_L8, 480, 640, &frames[c]);
				for (uint32_t y = 0; y < 640; y++) {
					memset(frames[c]->data + y * frames[c]->stride, (int)((y + pairs + c) & 0xff),
					       480);
				}
				xreal_air_record_cam_frame row{};
				row.cam = (xreal_air_cam_id)c;
				row.kernel_ns = now;
				row.dequeue_ns = now;
				row.euroc_ns = now;
				xreal_air_flight_push_cam_frame(f, &row);
			}
			xreal_air_flight_push_stereo(f, now, frames[0], frames[1]);
			xrt_frame_reference(&frames[0], nullptr);
			xrt_frame_reference(&frames[1], nullptr);

			xreal_air_flight_slam_pose pose{};
			pose.ts_ns = now - 50000000;
			pose.seen_ns = now;
			pose.pose.orientation.w = 1.0f;
			pose.verdict = 2;
			xreal_air_flight_push_slam_pose(f, &pose);

			const int64_t ts[3] = {pose.ts_ns, pose.ts_ns + 20000000, now};
			xreal_air_flight_push_slam_timing(f, ts, 3);
			pairs++;
		}
		tick++;
	}

	//! Feed for @p ms at ~1 kHz.
	void
	run_ms(int ms)
	{
		for (int i = 0; i < ms; i++) {
			step();
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}
	}
};

struct FramesFile
{
	uint64_t pairs = 0;
	bool all_ok = true;
	int64_t first_ns = -1;
};

FramesFile
read_frames(const std::string &path)
{
	FramesFile out;
	FILE *f = fopen(path.c_str(), "rb");
	if (f == nullptr) {
		out.all_ok = false;
		return out;
	}
	struct
	{
		uint32_t magic, codec;
		int64_t euroc_ns;
		uint32_t width, height, size[2];
	} hdr;
	static_assert(sizeof(hdr) == 32, "frames.bin header");
	std::vector<uint8_t> buf;
	while (fread(&hdr, sizeof(hdr), 1, f) == 1) {
		if (hdr.magic != 0x50465258u || hdr.width != 480 || hdr.height != 640) {
			out.all_ok = false;
			break;
		}
		buf.resize((size_t)hdr.size[0] + hdr.size[1]);
		if (fread(buf.data(), 1, buf.size(), f) != buf.size()) {
			out.all_ok = false;
			break;
		}
		if (out.first_ns < 0) {
			out.first_ns = hdr.euroc_ns;
		}
		out.pairs++;
	}
	fclose(f);
	return out;
}

} // namespace

TEST_CASE("flight_capture_with_lookback")
{
	Recorder rec(/*frames_s=*/1);
	Feeder feed{rec.f};

	// The status file exists from the start.
	auto st = rec.wait_state("idle");
	CHECK(st["rings_s"] == "2");
	CHECK(st["frames_lookback_s"] == "1");

	xreal_air_parsed_calibration calib{};
	const char doc[] = "{\"factory\": true}";
	xreal_air_flight_set_calibration(rec.f, doc, sizeof(doc) - 1, &calib);
	xreal_air_flight_set_timing_columns(rec.f, "sampled,frontend,received_by_monado");

	// Before the capture is asked for: an event and 1.5 s of data, all of it inside the 2 s rings.
	xreal_air_flight_event(rec.f, "tracking", "lost #%u", 1u);
	feed.run_ms(1500);
	const uint64_t pairs_before = feed.pairs;

	const std::string out = rec.tmp.dir + "/cap";
	st = rec.command("start", "out=" + out + "\ncodec=raw\nmax_s=30");
	REQUIRE(st["cmd_result"] == "ok");
	CHECK(st["state"] == "recording");

	feed.run_ms(300);
	st = rec.command("mark", "label=swim now\nmono_ns=" + std::to_string(os_monotonic_get_ns() - 50000000));
	CHECK(st["cmd_result"] == "ok");
	feed.run_ms(300);

	// A second start while recording is refused.
	st = rec.command("start", "out=" + rec.tmp.dir + "/cap2");
	CHECK(st["cmd_result"].rfind("error:", 0) == 0);

	st = rec.command("stop", "reason=test");
	CHECK(st["cmd_result"] == "ok");
	CHECK(st["state"] == "idle");
	CHECK(st["last_stop_reason"] == "test");
	CHECK(st["last_out"] == out);

	// The look-back reached back to before the start: most of the 1.5 s, plus the 0.6 s live.
	const size_t imu_rows = count_lines(out + "/xreal/imu_raw.csv") - 1;
	CHECK(imu_rows > 1500);
	CHECK(count_lines(out + "/mav0/imu0/data.csv") - 1 == imu_rows);
	// The head ring is 2048 records (2 s at 1024 / s): only the newest of the ~2100 pushed.
	CHECK(count_lines(out + "/flight/head_pose.csv") - 1 > 1500);
	CHECK(count_lines(out + "/xreal/camera_frames.csv") - 1 > 2 * pairs_before - 4);
	CHECK(count_lines(out + "/flight/slam_poses.csv") > pairs_before);

	const std::string timing = slurp(out + "/flight/slam_timing.csv");
	CHECK(timing.rfind("#sampled,frontend,received_by_monado\n", 0) == 0);

	const std::string events = slurp(out + "/flight/events.csv");
	CHECK(events.find(",tracking,lost #1") != std::string::npos);
	CHECK(events.find(",mark,swim now") != std::string::npos);
	CHECK(events.find(",capture,start") != std::string::npos);
	CHECK(events.find(",capture,stop reason=test") != std::string::npos);

	CHECK(slurp(out + "/xreal/factory_calibration.json") == doc);

	// The frame look-back (1 s armed) covers the second before the start, then everything live.
	const FramesFile frames = read_frames(out + "/xreal/frames.bin");
	CHECK(frames.all_ok);
	CHECK(frames.pairs >= 25);
	CHECK(frames.pairs <= feed.pairs);

	auto cap = key_values(slurp(out + "/capture.txt"));
	CHECK(cap["stop_reason"] == "test");
	CHECK(cap["lost_imu"] == "0");
	CHECK(cap["lost_pairs"] == "0");
	CHECK(cap["written_pairs"] == std::to_string(frames.pairs));
	CHECK(cap["codec"] == "raw");
}

TEST_CASE("flight_capture_refusals_and_limits")
{
	SECTION("refuses a non-empty or relative out")
	{
		Recorder rec;
		auto st = rec.command("start", "out=relative/dir");
		CHECK(st["cmd_result"].rfind("error:", 0) == 0);
		CHECK(st["state"] == "idle");

		const std::string full = rec.tmp.dir + "/full";
		mkdir(full.c_str(), 0700);
		std::ofstream(full + "/x") << "x";
		st = rec.command("start", "out=" + full);
		CHECK(st["cmd_result"].rfind("error:", 0) == 0);

		st = rec.command("stop", "");
		CHECK(st["cmd_result"] == "error: not recording");
	}

	SECTION("refuses when the disk is low")
	{
		Recorder rec(0, /*min_free_mb=*/1u << 30);
		const std::string out = rec.tmp.dir + "/cap";
		auto st = rec.command("start", "out=" + out);
		CHECK(st["cmd_result"].find("MB free") != std::string::npos);
		struct stat sb;
		CHECK(stat(out.c_str(), &sb) != 0); // Not left behind.
	}

	SECTION("stops after max_s, frames off")
	{
		Recorder rec;
		Feeder feed{rec.f};
		const std::string out = rec.tmp.dir + "/cap";
		auto st = rec.command("start", "out=" + out + "\nmax_s=1\nframes=0");
		REQUIRE(st["cmd_result"] == "ok");
		feed.run_ms(200);
		rec.wait_state("idle");
		st = key_values(slurp(rec.ctl + "/status"));
		CHECK(st["last_stop_reason"] == "max_s reached");
		struct stat sb;
		CHECK(stat((out + "/xreal/frames.bin").c_str(), &sb) != 0);
	}

	SECTION("a mark while idle stays in the event ring")
	{
		Recorder rec;
		auto st = rec.command("mark", "label=early");
		CHECK(st["cmd_result"] == "ok");
		const std::string out = rec.tmp.dir + "/cap";
		st = rec.command("start", "out=" + out + "\nframes=0");
		REQUIRE(st["cmd_result"] == "ok");
		st = rec.command("stop", "");
		CHECK(slurp(out + "/flight/events.csv").find(",mark,early") != std::string::npos);
	}

	SECTION("arm and disarm the frame look-back")
	{
		Recorder rec;
		Feeder feed{rec.f};
		auto st = rec.command("arm", "frames_s=1");
		CHECK(st["frames_lookback_s"] == "1");
		feed.run_ms(400);
		st = rec.command("arm", "frames_s=0");
		CHECK(st["frames_lookback_s"] == "0");
		// Nothing held any more: a capture now starts with no pairs from before.
		const std::string out = rec.tmp.dir + "/cap";
		const uint64_t before = feed.pairs;
		st = rec.command("start", "out=" + out + "\ncodec=raw");
		REQUIRE(st["cmd_result"] == "ok");
		st = rec.command("stop", "");
		const FramesFile frames = read_frames(out + "/xreal/frames.bin");
		CHECK(frames.all_ok);
		CHECK(frames.pairs <= feed.pairs - before);
	}
}
