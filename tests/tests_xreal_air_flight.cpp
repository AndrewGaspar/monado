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


/*
 *
 * Cost measurement on recorded data, hidden: run by hand with
 *   XREAL_FLIGHT_BENCH_FRAMES=frames.raw  (N x 2 x 480x640 L8, left then right)
 *   XREAL_FLIGHT_BENCH_TS=frames.ts       (N EuRoC timestamps, one per line)
 *   XREAL_FLIGHT_BENCH_EUROC=<dataset>    (mav0/imu0/data.csv, xreal/factory_calibration.json)
 *   XREAL_FLIGHT_BENCH_MODE=idle|zstd|raw|csv|png|noflight   XREAL_FLIGHT_BENCH_OUT=<empty dir>
 *   XREAL_FLIGHT_BENCH_START=0.5          when the capture starts (fraction of the slice)
 *   XREAL_FLIGHT_BENCH_ARM_S=0            frame look-back armed before the capture starts
 *   tests_xreal_air_flight "[.bench]"
 * Feeds the pairs and the IMU between them at their recorded rate (1x), as the driver would. In the
 * flight modes the capture starts halfway through with look-back, so the whole slice is recorded;
 * png is the existing XREAL_AIR_EUROC_RECORD writer. Prints the process CPU per second of data.
 *
 */

#include <sys/resource.h>

namespace {

double
process_cpu_s()
{
	rusage ru{};
	getrusage(RUSAGE_SELF, &ru);
	return (double)ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6 + (double)ru.ru_stime.tv_sec +
	       ru.ru_stime.tv_usec * 1e-6;
}

long
rss_kb()
{
	std::ifstream in("/proc/self/status");
	std::string line;
	while (std::getline(in, line)) {
		if (line.rfind("VmRSS:", 0) == 0) {
			return std::stol(line.substr(6));
		}
	}
	return -1;
}

std::string
env_or(const char *name, const char *fallback)
{
	const char *v = std::getenv(name);
	return v != nullptr ? v : fallback;
}

} // namespace

TEST_CASE("flight_bench", "[.bench]")
{
	const std::string frames_path = env_or("XREAL_FLIGHT_BENCH_FRAMES", "");
	const std::string euroc = env_or("XREAL_FLIGHT_BENCH_EUROC", "");
	const std::string mode = env_or("XREAL_FLIGHT_BENCH_MODE", "zstd");
	const std::string out_root = env_or("XREAL_FLIGHT_BENCH_OUT", "");
	const int arm_s = std::atoi(env_or("XREAL_FLIGHT_BENCH_ARM_S", "0").c_str());
	REQUIRE(!frames_path.empty());
	REQUIRE(!euroc.empty());
	REQUIRE(!out_root.empty());

	const std::string pixels = slurp(frames_path);
	std::vector<int64_t> pair_ts;
	{
		std::ifstream in(env_or("XREAL_FLIGHT_BENCH_TS", ""));
		std::string line;
		while (std::getline(in, line)) {
			pair_ts.push_back(std::stoll(line));
		}
	}
	const size_t image = 480 * 640;
	REQUIRE(pixels.size() == pair_ts.size() * 2 * image);

	struct ImuRow
	{
		int64_t ts;
		float w[3], a[3];
	};
	std::vector<ImuRow> imu;
	{
		std::ifstream in(euroc + "/mav0/imu0/data.csv");
		std::string line;
		while (std::getline(in, line)) {
			ImuRow r{};
			long long ts = 0;
			if (sscanf(line.c_str(), "%lld,%f,%f,%f,%f,%f,%f", &ts, &r.w[0], &r.w[1], &r.w[2], &r.a[0],
			           &r.a[1], &r.a[2]) == 7 &&
			    ts >= pair_ts.front() - 200000000 && ts <= pair_ts.back()) {
				r.ts = ts;
				imu.push_back(r);
			}
		}
	}
	REQUIRE(imu.size() > 1000);
	const std::string calib_doc = slurp(euroc + "/xreal/factory_calibration.json");

	xreal_air_flight *f = nullptr;
	xreal_air_record *record = nullptr;
	std::string ctl = out_root + "/ctl";
	const bool flight = mode == "idle" || mode == "zstd" || mode == "raw" || mode == "csv";
	if (flight) {
		xreal_air_flight_config cfg{};
		cfg.log_level = U_LOGGING_WARN;
		snprintf(cfg.ctl_dir, sizeof(cfg.ctl_dir), "%s", ctl.c_str());
		cfg.lookback_s = 60;
		cfg.frames_lookback_s = (uint32_t)arm_s;
		cfg.min_free_mb = 0;
		cfg.max_mb = 4096;
		cfg.default_max_s = 600;
		cfg.writer_nice = 10;
		f = xreal_air_flight_create(&cfg);
		REQUIRE(f != nullptr);
		xreal_air_parsed_calibration calib{};
		xreal_air_flight_set_calibration(f, calib_doc.data(), calib_doc.size(), &calib);
	} else if (mode == "png") {
		record = xreal_air_record_create((out_root + "/cap").c_str(), U_LOGGING_WARN);
		REQUIRE(record != nullptr);
	}

	auto drop = [&](const std::string &name, const std::string &body) {
		std::ofstream(ctl + "/." + name) << body << "\n";
		REQUIRE(rename((ctl + "/." + name).c_str(), (ctl + "/" + name).c_str()) == 0);
	};

	const long rss0 = rss_kb();
	const double cpu0 = process_cpu_s();
	const int64_t t0 = (int64_t)os_monotonic_get_ns();
	const int64_t data0 = imu.front().ts;
	const double start_frac = std::atof(env_or("XREAL_FLIGHT_BENCH_START", "0.5").c_str());
	const size_t start_at = (size_t)(start_frac * (double)pair_ts.size());
	// The recording's clock onto ours, as the driver's mapping does it live (the look-back cut-off
	// compares EuRoC times with now).
	const int64_t shift = t0 - data0;
	long rss_peak = rss0;
	size_t next_imu = 0;
	for (size_t p = 0; p < pair_ts.size(); p++) {
		// The IMU samples up to this pair, each at its recorded time.
		while (next_imu < imu.size() && imu[next_imu].ts <= pair_ts[p]) {
			const ImuRow &r = imu[next_imu++];
			const int64_t due = t0 + (r.ts - data0);
			int64_t now = (int64_t)os_monotonic_get_ns();
			if (due > now) {
				std::this_thread::sleep_for(std::chrono::nanoseconds(due - now));
				now = (int64_t)os_monotonic_get_ns();
			}
			xreal_air_record_imu s{};
			s.host_ns = now;
			s.device_ts = (uint64_t)r.ts;
			s.fusion_ns = now;
			s.euroc_ns = r.ts + shift;
			s.gyro = {r.w[0], r.w[1], r.w[2]};
			s.accel = {r.a[0], r.a[1], r.a[2]};
			if (f != nullptr) {
				xreal_air_flight_push_imu(f, &s);
				xreal_air_flight_head head{};
				head.query_ns = now;
				head.at_ns = now;
				head.pose.orientation.w = 1;
				head.fusion.w = 1;
				xreal_air_flight_push_head(f, &head);
			} else if (record != nullptr) {
				xreal_air_record_push_imu(record, &s);
			}
		}

		const int64_t now = (int64_t)os_monotonic_get_ns();
		xrt_frame *frames[2] = {nullptr, nullptr};
		for (int c = 0; c < 2; c++) {
			u_frame_create_one_off(XRT_FORMAT_L8, 480, 640, &frames[c]);
			memcpy(frames[c]->data, pixels.data() + (p * 2 + c) * image, image);
			xreal_air_record_cam_frame row{};
			row.cam = (xreal_air_cam_id)c;
			row.kernel_ns = now;
			row.dequeue_ns = now;
			row.euroc_ns = pair_ts[p] + shift;
			row.telemetry.exposure = 8000000;
			if (f != nullptr) {
				xreal_air_flight_push_cam_frame(f, &row);
			} else if (record != nullptr) {
				xreal_air_record_push_cam_frame(record, &row);
			}
		}
		if (f != nullptr) {
			xreal_air_flight_push_stereo(f, pair_ts[p] + shift, frames[0], frames[1]);
		} else if (record != nullptr) {
			xreal_air_record_push_stereo(record, pair_ts[p] + shift, frames[0], frames[1]);
		}
		xrt_frame_reference(&frames[0], nullptr);
		xrt_frame_reference(&frames[1], nullptr);

		if (p == start_at && f != nullptr && mode != "idle") {
			drop("start", "out=" + out_root + "/cap\nlookback_s=60\n" +
			                  (mode == "csv" ? std::string("frames=0") : "codec=" + mode));
		}
		rss_peak = std::max(rss_peak, rss_kb());
	}
	if (f != nullptr && mode != "idle") {
		drop("stop", "reason=bench");
	}
	// Let the writers finish what they have, then stop.
	if (f != nullptr) {
		for (int i = 0; i < 600 && mode != "idle"; i++) {
			if (key_values(slurp(ctl + "/status"))["state"] == "idle") {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		xreal_air_flight_destroy(&f);
	}
	xreal_air_record_destroy(&record);
	const double cpu = process_cpu_s() - cpu0;
	const double data_s = (double)(pair_ts.back() - data0) * 1e-9;

	printf(
	    "BENCH mode=%s arm_s=%d pairs=%zu imu=%zu data=%.1f s cpu=%.3f s -> %.1f ms CPU per s; rss +%.1f MB "
	    "peak\n",
	    mode.c_str(), arm_s, pair_ts.size(), imu.size(), data_s, cpu, cpu / data_s * 1e3,
	    (double)(rss_peak - rss0) / 1024.0);
}

/*!
 * A stand-in runtime for testing the control tools (HypXRland's scripts/xreal-capture) without the
 * glasses, hidden: XREAL_FLIGHT_BENCH_FRAMES / _TS as above, XREAL_FLIGHT_SERVE_CTL=<control dir>,
 * XREAL_FLIGHT_SERVE_S=<seconds>. Loops the frames at 1x with a synthetic IMU, SLAM poses (latency
 * rising to 400 ms for a second every 20 s), a tracking loss and a camera gap every 20 s.
 */
TEST_CASE("flight_serve", "[.serve]")
{
	const std::string pixels = slurp(env_or("XREAL_FLIGHT_BENCH_FRAMES", ""));
	const size_t image = 480 * 640;
	const size_t pairs = pixels.size() / (2 * image);
	REQUIRE(pairs > 0);
	const double serve_s = std::atof(env_or("XREAL_FLIGHT_SERVE_S", "60").c_str());

	xreal_air_flight_config cfg{};
	REQUIRE(xreal_air_flight_default_config(U_LOGGING_WARN, &cfg));
	snprintf(cfg.ctl_dir, sizeof(cfg.ctl_dir), "%s", env_or("XREAL_FLIGHT_SERVE_CTL", "").c_str());
	cfg.min_free_mb = 0;
	xreal_air_flight *f = xreal_air_flight_create(&cfg);
	REQUIRE(f != nullptr);
	xreal_air_parsed_calibration calib{};
	const char doc[] = "{\"synthetic\": true}";
	xreal_air_flight_set_calibration(f, doc, sizeof(doc) - 1, &calib);
	xreal_air_flight_set_timing_columns(f, "sampled,frontend_start,received_by_monado");

	const int64_t t0 = (int64_t)os_monotonic_get_ns();
	const int64_t pair_ns = 35460992; // 28.2 Hz
	int64_t next_pair = t0;
	bool lost = false;
	uint64_t p = 0;
	for (int64_t now = t0; now - t0 < (int64_t)(serve_s * 1e9); now = (int64_t)os_monotonic_get_ns()) {
		const double t = (double)(now - t0) * 1e-9;
		const double phase = std::fmod(t, 20.0);

		xreal_air_record_imu s{};
		s.host_ns = now;
		s.euroc_ns = now;
		s.device_ts = (uint64_t)now;
		s.gyro = {0.01f, 0.0f, 0.0f};
		s.accel = {0.0f, -9.81f, 0.0f};
		xreal_air_flight_push_imu(f, &s);

		const bool tracking = !(phase > 10.0 && phase < 13.0);
		xreal_air_flight_head head{};
		head.query_ns = now;
		head.at_ns = now + 20000000;
		head.pose.orientation.w = 1;
		head.fusion.w = 1;
		head.slam = tracking;
		xreal_air_flight_push_head(f, &head);
		if (tracking == lost) {
			lost = !tracking;
			xreal_air_flight_event(f, "tracking",
			                       lost ? "lost #%u, newest pose 320 ms old" : "acquired #%u",
			                       (unsigned)(t / 20) + 1);
		}

		if (now >= next_pair) {
			next_pair += pair_ns;
			const bool gap = phase > 9.0 && phase < 9.2; // No frames: a 200 ms camera gap.
			if (!gap) {
				xrt_frame *frames[2] = {nullptr, nullptr};
				for (int c = 0; c < 2; c++) {
					u_frame_create_one_off(XRT_FORMAT_L8, 480, 640, &frames[c]);
					memcpy(frames[c]->data, pixels.data() + ((p % pairs) * 2 + c) * image, image);
					xreal_air_record_cam_frame row{};
					row.cam = (xreal_air_cam_id)c;
					row.v4l2_sequence = (uint32_t)p;
					row.kernel_ns = now - 9000000;
					row.dequeue_ns = now;
					row.euroc_ns = now - 30000000;
					row.telemetry.exposure = 8000000;
					xreal_air_flight_push_cam_frame(f, &row);
				}
				xreal_air_flight_push_stereo(f, now - 30000000, frames[0], frames[1]);
				xrt_frame_reference(&frames[0], nullptr);
				xrt_frame_reference(&frames[1], nullptr);

				const int64_t latency = (phase > 9.0 && phase < 10.0) ? 400000000 : 60000000;
				xreal_air_flight_slam_pose pose{};
				pose.ts_ns = now - latency;
				pose.seen_ns = now;
				pose.pose.orientation.w = 1;
				pose.world_id = 1;
				pose.verdict = p == 0 ? 1 : 2;
				xreal_air_flight_push_slam_pose(f, &pose);
				const int64_t ts[3] = {pose.ts_ns, pose.ts_ns + 5000000, now};
				xreal_air_flight_push_slam_timing(f, ts, 3);
			}
			p++;
		}
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
	xreal_air_flight_destroy(&f);
}
