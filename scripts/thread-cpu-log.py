#!/usr/bin/env python3
# Copyright 2026, Andrew Gaspar
# SPDX-License-Identifier: BSL-1.0
"""
Log the per-thread CPU use of a running process (monado-service by default) to CSV.

Meant for questions like "which core class does the SLAM tracker run on, and how busy is it"
on hybrid CPUs. Every interval, for every thread of the process:

  - CPU% over the interval (utime + stime, 100 = one core),
  - the CPU it last ran on (/proc/<pid>/task/<tid>/stat field 39), that CPU's core type and current
    frequency,
  - nice, scheduling policy, RT priority, voluntary / involuntary context switches over the interval,

plus, per sample, the power profile, AC / battery state, battery discharge power (a whole-system
energy measure when on battery) and /proc/pressure/cpu.

Read-only: it only reads /proc and /sys, it never signals or reconfigures the process.

Threads inherit their creator's name: Basalt's threads all show up as "XREAL Air SLAM", tell them
apart by tid. A summary per thread goes to stderr at the end (Ctrl-C or --duration).

Examples:
  scripts/thread-cpu-log.py -o slam-threads.csv                 # monado-xreal user unit, 1 s
  scripts/thread-cpu-log.py --pid 1234 --interval 2 --duration 600 -o out.csv
  scripts/thread-cpu-log.py --match 'XREAL|SLAM|Camera|IMU' -o out.csv
"""

import argparse
import csv
import glob
import os
import re
import signal
import subprocess
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK")
POLICIES = {0: "OTHER", 1: "FIFO", 2: "RR", 3: "BATCH", 5: "IDLE", 6: "DEADLINE"}


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def parse_cpulist(s):
    cpus = set()
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-")
            cpus.update(range(int(a), int(b) + 1))
        else:
            cpus.add(int(part))
    return cpus


def cpu_types():
    """CPU number -> core type: Intel hybrid PMU names if present, else the max frequency."""
    types = {}
    for pmu, name in (("cpu_core", "P"), ("cpu_atom", "E")):
        cpus = read(f"/sys/devices/{pmu}/cpus")
        for c in parse_cpulist(cpus):
            types[c] = name
    # Lunar Lake's LP E-cores are also cpu_atom; they differ by max frequency, so always add it.
    for d in glob.glob("/sys/devices/system/cpu/cpu[0-9]*"):
        c = int(d.rsplit("cpu", 1)[1])
        mhz = read(f"{d}/cpufreq/cpuinfo_max_freq")
        mhz = f"{int(mhz) // 1000}MHz" if mhz.isdigit() else "?"
        types[c] = f"{types[c]}/{mhz}" if c in types else f"max{mhz}"
    return types


def unit_pid(unit):
    try:
        out = subprocess.run(["systemctl", "--user", "show", "-P", "MainPID", unit],
                             capture_output=True, text=True, timeout=5).stdout.strip()
        return int(out) if out.isdigit() and int(out) > 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def power_state():
    profile = read("/sys/firmware/acpi/platform_profile")
    try:
        ppd = subprocess.run(["powerprofilesctl", "get"], capture_output=True, text=True,
                             timeout=2).stdout.strip()
        if ppd:
            profile = ppd if not profile else f"{ppd}/{profile}"
    except (OSError, subprocess.SubprocessError):
        pass
    ac, bat_status, bat_w = "", "", ""
    for d in glob.glob("/sys/class/power_supply/*"):
        kind = read(f"{d}/type")
        if kind == "Mains":
            ac = read(f"{d}/online")
        elif kind == "Battery" and read(f"{d}/scope") != "Device":
            bat_status = read(f"{d}/status")
            uw = read(f"{d}/power_now")
            if not uw:  # Some batteries only report current and voltage.
                ua, uv = read(f"{d}/current_now"), read(f"{d}/voltage_now")
                if ua.isdigit() and uv.isdigit():
                    uw = str(int(ua) * int(uv) // 1000000)
            if uw.lstrip("-").isdigit():
                bat_w = f"{abs(int(uw)) / 1e6:.2f}"
    return profile, ac, bat_status, bat_w


def psi_cpu():
    some, full = "", ""
    for line in read("/proc/pressure/cpu").splitlines():
        m = re.match(r"(some|full) avg10=([\d.]+)", line)
        if m and m.group(1) == "some":
            some = m.group(2)
        elif m:
            full = m.group(2)
    return some, full


def read_threads(pid):
    """tid -> dict, or None if the process is gone."""
    try:
        tids = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return None
    threads = {}
    for tid in tids:
        stat = read(f"/proc/{pid}/task/{tid}/stat")
        if not stat:
            continue  # Exited meanwhile.
        # The name can contain spaces and parentheses: the fields start after the last ')'.
        comm = stat[stat.index("(") + 1:stat.rindex(")")]
        f = stat[stat.rindex(")") + 2:].split()
        # f[0] is field 3 (state), so field N is f[N - 3].
        ctx = {}
        for line in read(f"/proc/{pid}/task/{tid}/status").splitlines():
            k, _, v = line.partition(":")
            if k in ("voluntary_ctxt_switches", "nonvoluntary_ctxt_switches"):
                ctx[k] = int(v)
        threads[int(tid)] = {
            "comm": comm,
            "state": f[0],
            "ticks": int(f[11]) + int(f[12]),
            "nice": int(f[16]),
            "cpu": int(f[36]),
            "rt_prio": int(f[37]),
            "policy": POLICIES.get(int(f[38]), f[38]),
            "vcsw": ctx.get("voluntary_ctxt_switches", 0),
            "ivcsw": ctx.get("nonvoluntary_ctxt_switches", 0),
        }
    return threads


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pid", type=int, help="process to watch (default: the --unit's MainPID)")
    ap.add_argument("--unit", default="monado-xreal", help="systemd user unit (default monado-xreal)")
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between samples (default 1)")
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (default: Ctrl-C)")
    ap.add_argument("--match", help="only log threads whose name matches this regex")
    ap.add_argument("-o", "--output", help="CSV file (default stdout)")
    args = ap.parse_args()

    pid = args.pid or unit_pid(args.unit)
    if pid is None:
        sys.exit(f"No --pid given and the user unit '{args.unit}' has no main PID (not running?)")
    match = re.compile(args.match) if args.match else None
    types = cpu_types()

    out = open(args.output, "w", newline="") if args.output else sys.stdout
    w = csv.writer(out)
    w.writerow(["t_s", "wall", "pid", "tid", "comm", "cpu_pct", "last_cpu", "cpu_type", "cpu_mhz", "state",
                "nice", "policy", "rt_prio", "vcsw", "ivcsw", "profile", "ac_online", "bat_status",
                "bat_power_w", "psi_cpu_some10", "psi_cpu_full10"])

    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    summary = {}  # tid -> [comm, cpu_pct sum, samples, {type: count}]
    t0 = time.monotonic()
    prev, prev_t = read_threads(pid), time.monotonic()
    print(f"Watching pid {pid} ({len(prev or {})} threads), every {args.interval} s", file=sys.stderr)
    while not stop:
        time.sleep(args.interval)
        now = time.monotonic()
        cur = read_threads(pid)
        if cur is None:
            if args.pid:
                print(f"pid {pid} exited", file=sys.stderr)
                break
            new_pid = unit_pid(args.unit)
            if new_pid and new_pid != pid:
                print(f"{args.unit}: pid {pid} -> {new_pid}", file=sys.stderr)
                pid, prev, prev_t = new_pid, read_threads(new_pid), now
            continue
        dt = now - prev_t
        profile, ac, bat_status, bat_w = power_state()
        some, full = psi_cpu()
        wall = time.strftime("%Y-%m-%dT%H:%M:%S")
        for tid, t in sorted(cur.items()):
            if match and not match.search(t["comm"]):
                continue
            p = (prev or {}).get(tid)
            if p is None:
                continue  # New thread: CPU% from the next sample on.
            pct = 100.0 * (t["ticks"] - p["ticks"]) / CLK_TCK / dt
            ctype = types.get(t["cpu"], "?")
            mhz = read(f"/sys/devices/system/cpu/cpu{t['cpu']}/cpufreq/scaling_cur_freq")
            mhz = str(int(mhz) // 1000) if mhz.isdigit() else ""
            w.writerow([f"{now - t0:.1f}", wall, pid, tid, t["comm"], f"{pct:.1f}", t["cpu"], ctype, mhz,
                        t["state"], t["nice"], t["policy"], t["rt_prio"], t["vcsw"] - p["vcsw"],
                        t["ivcsw"] - p["ivcsw"], profile, ac, bat_status, bat_w, some, full])
            s = summary.setdefault(tid, [t["comm"], 0.0, 0, {}])
            s[1] += pct
            s[2] += 1
            s[3][ctype] = s[3].get(ctype, 0) + 1
        out.flush()
        prev, prev_t = cur, now
        if args.duration and now - t0 >= args.duration:
            break

    if summary:
        print("\ntid       name                  mean CPU%  samples  last-ran-on core types", file=sys.stderr)
        for tid, (comm, total, n, ctypes) in sorted(summary.items(), key=lambda kv: -kv[1][1] / kv[1][2]):
            mean = total / n
            if mean < 0.5:
                continue
            where = ", ".join(f"{k} {100 * v / n:.0f}%" for k, v in sorted(ctypes.items(), key=lambda kv: -kv[1]))
            print(f"{tid:<9} {comm:<21} {mean:9.1f} {n:8d}  {where}", file=sys.stderr)
    if out is not sys.stdout:
        out.close()


if __name__ == "__main__":
    main()
