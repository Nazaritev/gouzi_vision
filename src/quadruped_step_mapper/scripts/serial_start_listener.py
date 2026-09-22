#!/usr/bin/env python3
"""Launch the obstacle-race supervisor from a bracket-framed start message.

Default protocol:
  controller -> PC: [start,0]  standard route
  controller -> PC: [start,1]  mirrored route

Legacy [start] is still accepted and treated as mode 0.

Examples:
  python3 serial_start_listener.py
  python3 serial_start_listener.py --disable-mode-selection
  python3 serial_start_listener.py --device /dev/ttyUSB1 --baud 115200
  python3 serial_start_listener.py --listen-only --once --timeout 10
  python3 serial_start_listener.py --listen-only --echo-raw
"""

from __future__ import annotations

import argparse
import os
import select
import subprocess
import sys
import termios
import time
from dataclasses import dataclass
from typing import Optional


BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


@dataclass
class SerialFrame:
    payload: str
    raw: bytes


class BracketFrameParser:
    def __init__(self, start_byte: int = ord("["), end_byte: int = ord("]"), max_len: int = 128) -> None:
        self.start_byte = start_byte
        self.end_byte = end_byte
        self.max_len = max(2, max_len)
        self.buffer = bytearray()
        self.in_frame = False

    def feed(self, data: bytes) -> list[SerialFrame]:
        frames: list[SerialFrame] = []
        for byte in data:
            if byte == self.start_byte:
                self.buffer.clear()
                self.buffer.append(byte)
                self.in_frame = True
                continue

            if not self.in_frame:
                continue

            self.buffer.append(byte)
            if len(self.buffer) > self.max_len:
                self.buffer.clear()
                self.in_frame = False
                continue

            if byte == self.end_byte:
                raw = bytes(self.buffer)
                payload_bytes = raw[1:-1].strip()
                payload = payload_bytes.decode("utf-8", errors="replace")
                frames.append(SerialFrame(payload=payload, raw=raw))
                self.buffer.clear()
                self.in_frame = False
        return frames


class RawSerial:
    def __init__(self, device: str, baud_rate: int) -> None:
        self.device = device
        self.baud_rate = baud_rate
        self.fd: Optional[int] = None

    def open(self) -> None:
        if self.baud_rate not in BAUD_MAP:
            raise ValueError(f"Unsupported baud_rate={self.baud_rate}")

        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] &= ~(
            termios.IGNBRK
            | termios.ICRNL
            | termios.INLCR
            | termios.IXON
            | termios.IXOFF
            | termios.IXANY
        )
        attrs[1] &= ~(termios.OPOST | termios.ONLCR | termios.OCRNL)
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CRTSCTS)
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        speed = BAUD_MAP[self.baud_rate]
        if hasattr(termios, "cfsetispeed") and hasattr(termios, "cfsetospeed"):
            termios.cfsetispeed(attrs, speed)
            termios.cfsetospeed(attrs, speed)
        else:
            attrs[4] = speed
            attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read_available(self, timeout_sec: float) -> bytes:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")

        readable, _, _ = select.select([self.fd], [], [], timeout_sec)
        if not readable:
            return b""
        try:
            return os.read(self.fd, 1024)
        except BlockingIOError:
            return b""

    def write_frame(self, payload: str) -> None:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")

        data = f"[{payload}]".encode("utf-8")
        total = 0
        while total < len(data):
            written = os.write(self.fd, data[total:])
            if written <= 0:
                raise RuntimeError("Serial write returned no data")
            total += written


def timestamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Wait for [start,0] or [start,1], then launch the matching obstacle-race mode."
        )
    )
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device")
    parser.add_argument("--baud", type=int, default=115200, choices=sorted(BAUD_MAP), help="Baud rate")
    parser.add_argument("--token", default="start", help="Expected command before the mode field")
    parser.add_argument(
        "--disable-mode-selection",
        action="store_true",
        help=(
            "Rollback switch: ignore the received mode and always use "
            "obstacle_waypoints.yaml plus orange_pole_body_relative_test_v4.yaml"
        ),
    )
    parser.add_argument(
        "--listen-only",
        action="store_true",
        help="Only print valid start frames; do not launch the ROS 2 supervisor",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="In --listen-only mode, exit after the first valid start frame",
    )
    parser.add_argument("--timeout", type=float, default=0.0, help="Exit with failure after N seconds; 0 means wait forever")
    parser.add_argument("--poll-timeout", type=float, default=0.20, help="Serial read poll timeout in seconds")
    parser.add_argument("--max-frame-len", type=int, default=128, help="Maximum accepted frame length")
    parser.add_argument("--case-sensitive", action="store_true", help="Match token case-sensitively")
    parser.add_argument("--echo-raw", action="store_true", help="Print raw bytes received from serial")
    parser.add_argument("--ack", help="Optional payload to send back when the expected token is received, for example ack")
    return parser


def token_matches(payload: str, token: str, case_sensitive: bool) -> bool:
    if case_sensitive:
        return payload == token
    return payload.lower() == token.lower()


def parse_start_mode(
    payload: str,
    token: str,
    case_sensitive: bool,
) -> Optional[int]:
    fields = [field.strip() for field in payload.split(",")]
    if not fields or not token_matches(fields[0], token, case_sensitive):
        return None

    if len(fields) == 1:
        return 0
    if len(fields) != 2:
        return None

    try:
        mode = int(fields[1], 10)
    except ValueError:
        return None
    return mode if mode in (0, 1) else None


def launch_supervisor(
    mode: int,
    serial_device: str,
    mode_selection_enabled: bool,
) -> int:
    command = [
        "ros2",
        "launch",
        "auto_nav_pkg",
        "obstacle_race_visual_supervisor.launch.py",
        f"race_mode:={mode}",
        "enable_race_mode_selection:="
        f"{'true' if mode_selection_enabled else 'false'}",
        "supervisor_wait_for_start_signal:=false",
        f"serial_device:={serial_device}",
    ]
    print(f"{timestamp()} launching: {' '.join(command)}", flush=True)
    try:
        return subprocess.run(command, check=False).returncode
    except FileNotFoundError:
        print("ros2 executable was not found; source the ROS 2 environment first.", file=sys.stderr)
        return 127


def main() -> int:
    parser = build_argparser()
    args = parser.parse_args()

    if args.timeout < 0.0:
        parser.error("--timeout must be >= 0")
    if args.poll_timeout <= 0.0:
        parser.error("--poll-timeout must be > 0")

    serial = RawSerial(args.device, args.baud)
    frame_parser = BracketFrameParser(max_len=args.max_frame_len)
    start_time = time.monotonic()
    matched = False

    try:
        serial.open()
        print(
            f"Opened {args.device} @ {args.baud}, "
            f"waiting for [{args.token},0] or [{args.token},1]"
        )
        while True:
            if args.timeout > 0.0 and (time.monotonic() - start_time) >= args.timeout:
                print(
                    f"{timestamp()} timeout waiting for "
                    f"[{args.token},0] or [{args.token},1]",
                    file=sys.stderr,
                )
                return 1

            data = serial.read_available(args.poll_timeout)
            if not data:
                continue

            if args.echo_raw:
                print(f"{timestamp()} raw={data!r}")

            for frame in frame_parser.feed(data):
                print(f"{timestamp()} frame={frame.raw!r} payload={frame.payload!r}")
                mode = parse_start_mode(frame.payload, args.token, args.case_sensitive)
                if mode is None:
                    continue

                matched = True
                print(f"{timestamp()} received start mode {mode}: [{frame.payload}]")
                if args.ack:
                    serial.write_frame(args.ack)
                    print(f"{timestamp()} sent ack: [{args.ack}]")

                if args.listen_only:
                    if args.once:
                        return 0
                    continue

                mode_selection_enabled = not args.disable_mode_selection
                effective_mode = mode if mode_selection_enabled else 0
                if not mode_selection_enabled:
                    print(
                        f"{timestamp()} mode selection disabled; "
                        "forcing standard mode 0"
                    )
                serial.close()
                return launch_supervisor(
                    effective_mode,
                    args.device,
                    mode_selection_enabled,
                )
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 0 if matched else 130
    finally:
        serial.close()


if __name__ == "__main__":
    raise SystemExit(main())
