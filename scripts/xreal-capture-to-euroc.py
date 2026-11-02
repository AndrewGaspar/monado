#!/usr/bin/env python3
# Copyright 2026, Andrew Gaspar
# SPDX-License-Identifier: BSL-1.0
"""
Expand an XREAL Air flight recorder capture's stereo pairs (xreal/frames.bin) into the EuRoC
image layout: mav0/cam0/data/<ns>.png, mav0/cam1/data/<ns>.png and their data.csv.

A flight capture (src/xrt/drivers/xreal_air/xreal_air_flight.h) already has mav0/imu0 and xreal/
in the layout of an XREAL_AIR_EUROC_RECORD dataset; after this it is one, so it replays unchanged:

  scripts/xreal-capture-to-euroc.py <capture>
  VIT_SYSTEM_LIBRARY_PATH=... XREAL_AIR_SLAM_CONFIG=... monado-cli xreal-slam-replay <capture> <out>

Standard library only (zstd frames need Python 3.14's compression.zstd, or the zstandard module).

  --out DIR       write the images there instead (a copy of mav0/imu0 and xreal/ goes along)
  --format pgm    uncompressed PGM instead of PNG: much faster, 1.3-1.8x bigger (OpenCV and so
                  the replay read either)
  --level N       PNG zlib level (default 1)
  --delete-frames remove xreal/frames.bin once expanded (in place only)
"""

import argparse
import os
import shutil
import struct
import sys
import zlib

HEADER = struct.Struct("<IIqIIII")  # magic, codec, euroc_ns, width, height, size0, size1
MAGIC = 0x50465258
CODEC_RAW, CODEC_ZSTD = 0, 1


def zstd_decompressor():
    try:
        from compression import zstd  # Python 3.14+

        return lambda data, size: zstd.decompress(data)
    except ImportError:
        pass
    try:
        import zstandard

        d = zstandard.ZstdDecompressor()
        return lambda data, size: d.decompress(data, max_output_size=size)
    except ImportError:
        return None


def png_bytes(width, height, pixels, level):
    rows = b"".join(b"\x00" + pixels[y * width:(y + 1) * width] for y in range(height))

    def chunk(kind, data):
        c = struct.pack(">I", len(data)) + kind + data
        return c + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(rows, level)) + chunk(b"IEND", b"")


def pgm_bytes(width, height, pixels):
    return b"P5\n%d %d\n255\n" % (width, height) + pixels


def read_pairs(path):
    """Yields (euroc_ns, codec, width, height, left, right); stops at a truncated tail."""
    with open(path, "rb") as f:
        while True:
            hdr = f.read(HEADER.size)
            if len(hdr) < HEADER.size:
                return
            magic, codec, ns, w, h, s0, s1 = HEADER.unpack(hdr)
            if magic != MAGIC:
                print(f"bad magic at offset {f.tell() - HEADER.size}, stopping", file=sys.stderr)
                return
            left, right = f.read(s0), f.read(s1)
            if len(left) < s0 or len(right) < s1:
                print("truncated last pair (the runtime was killed mid-write), skipped", file=sys.stderr)
                return
            yield ns, codec, w, h, left, right


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--out")
    ap.add_argument("--format", choices=("png", "pgm"), default="png")
    ap.add_argument("--level", type=int, default=1)
    ap.add_argument("--delete-frames", action="store_true", help="remove xreal/frames.bin when done")
    args = ap.parse_args()

    src = os.path.abspath(args.capture)
    frames = os.path.join(src, "xreal", "frames.bin")
    if not os.path.isfile(frames):
        sys.exit(f"{frames}: no stereo frames in this capture (recorded with frames=0?)")

    out = os.path.abspath(args.out) if args.out else src
    if out != src:
        for sub in ("mav0/imu0", "xreal"):
            dst = os.path.join(out, sub)
            if not os.path.exists(dst):
                shutil.copytree(os.path.join(src, sub), dst, ignore=shutil.ignore_patterns("frames.bin"))

    dirs = [os.path.join(out, "mav0", f"cam{i}", "data") for i in (0, 1)]
    for d in dirs:
        os.makedirs(d, exist_ok=True)

    unzstd = None
    names = []
    for ns, codec, w, h, left, right in read_pairs(frames):
        images = [left, right]
        if codec == CODEC_ZSTD:
            if unzstd is None:
                unzstd = zstd_decompressor()
                if unzstd is None:
                    sys.exit("zstd frames: needs Python 3.14 (compression.zstd) or the zstandard module")
            images = [unzstd(img, w * h) for img in images]
        elif codec != CODEC_RAW:
            sys.exit(f"unknown codec {codec}")
        name = f"{ns}.{args.format}"
        for d, img in zip(dirs, images):
            if len(img) != w * h:
                sys.exit(f"pair {ns}: image of {len(img)} bytes, expected {w * h}")
            data = png_bytes(w, h, img, args.level) if args.format == "png" else pgm_bytes(w, h, img)
            with open(os.path.join(d, name), "wb") as f:
                f.write(data)
        names.append((ns, name))
        if len(names) % 500 == 0:
            print(f"{len(names)} pairs", file=sys.stderr)

    for i in (0, 1):
        with open(os.path.join(out, "mav0", f"cam{i}", "data.csv"), "w", newline="") as f:
            f.write("#timestamp [ns],filename\r\n")
            for ns, name in names:
                f.write(f"{ns},{name}\r\n")

    if args.delete_frames and out == src:
        os.remove(frames)
    print(f"{len(names)} stereo pairs -> {out}/mav0/cam0, cam1 ({args.format})")


if __name__ == "__main__":
    main()
