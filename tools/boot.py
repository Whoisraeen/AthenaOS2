#!/usr/bin/env python3
"""Build a freestanding x86-64 Multiboot image and exercise it in QEMU."""
import argparse
import pathlib
import shutil
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
ELF = ROOT / "target/x86_64-unknown-none/release/athena-kernel"
IMAGE = ROOT / "target/athena-x86_64.bin"


def build():
    subprocess.run(["cargo", "build", "--release", "-p", "athena-kernel",
                    "--target", "x86_64-unknown-none"], cwd=ROOT, check=True)
    data = ELF.read_bytes()
    if data[:6] != b"\x7fELF\x02\x01":
        raise ValueError("expected a little-endian ELF64 kernel")
    machine = struct.unpack_from("<H", data, 18)[0]
    if machine != 62:
        raise ValueError("expected an x86-64 kernel")
    phoff = struct.unpack_from("<Q", data, 32)[0]
    phsize, count = struct.unpack_from("<HH", data, 54)
    segments = []
    for i in range(count):
        kind, _, offset, _, physical, size, memory_size, _ = struct.unpack_from(
            "<IIQQQQQQ", data, phoff + i * phsize)
        if kind == 1 and memory_size:
            if size > memory_size or offset + size > len(data):
                raise ValueError("invalid ELF load segment")
            segments.append((physical, data[offset:offset + size], memory_size))
    base = min(address for address, _, _ in segments)
    end = max(address + len(payload) for address, payload, _ in segments)
    if base != 0x100000 or end - base > 16 * 1024 * 1024:
        raise ValueError("unexpected kernel load layout")
    image = bytearray(end - base)
    for address, payload, _ in segments:
        image[address - base:address - base + len(payload)] = payload
    magic, flags, checksum, header, load, load_end, bss_end, entry = struct.unpack_from("<8I", image)
    if magic != 0x1BADB002 or (magic + flags + checksum) & 0xFFFFFFFF:
        raise ValueError("invalid Multiboot header")
    if header != base or load != base or load_end != end or not base <= entry < end:
        raise ValueError("Multiboot addresses disagree with ELF segments")
    if bss_end != max(address + size for address, _, size in segments):
        raise ValueError("Multiboot BSS end disagrees with ELF segments")
    IMAGE.write_bytes(image)
    print(f"Built {IMAGE.relative_to(ROOT)} ({len(image):,} bytes, memory through 0x{bss_end:x})", flush=True)


def boot(test):
    qemu = shutil.which("qemu-system-x86_64")
    if not qemu:
        raise RuntimeError("qemu-system-x86_64 is required (macOS: brew install qemu)")
    command = [qemu, "-machine", "pc", "-accel", "tcg", "-cpu", "qemu64", "-m", "128M",
               "-kernel", str(IMAGE), "-display", "none", "-serial", "stdio", "-monitor", "none",
               "-no-reboot", "-device", "isa-debug-exit,iobase=0xf4,iosize=0x04"]
    try:
        result = subprocess.run(command, cwd=ROOT, capture_output=test, text=True, timeout=30)
    except subprocess.TimeoutExpired as error:
        if error.stdout:
            sys.stdout.write(error.stdout.decode() if isinstance(error.stdout, bytes) else error.stdout)
        if error.stderr:
            sys.stderr.write(error.stderr.decode() if isinstance(error.stderr, bytes) else error.stderr)
        raise RuntimeError("QEMU exceeded the 30-second boot-test timeout") from error
    if test:
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        if "ATHENA_BOOT_OK" not in result.stdout or result.returncode != 33:
            raise RuntimeError(f"kernel boot test failed (QEMU exit {result.returncode})")
        print("QEMU boot test passed.")
    elif result.returncode != 33:
        raise RuntimeError(f"kernel boot failed (QEMU exit {result.returncode})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["build", "test", "run"], nargs="?", default="test")
    args = parser.parse_args()
    build()
    if args.action != "build":
        boot(args.action == "test")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        sys.exit(f"Athena boot: {error}")
