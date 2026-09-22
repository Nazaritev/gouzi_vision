#!/usr/bin/env python3
"""Send raw [mode,left,right] step commands directly over serial.

Examples:
  python3 serial_step_tester.py --mode 0 --left 0.20 --right 0.80 --duration 3
  python3 serial_step_tester.py --mode 0 --left 0.00 --right 1.00 --hz 20 --continuous
  python3 serial_step_tester.py --mode 0 --left 0.00 --right 1.00 --duration 2 --second-mode 6 --second-left 0.00 --second-right 0.00 --second-duration 1
  python3 serial_step_tester.py --interactive
"""

from __future__ import annotations

import argparse
import math
import os
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
class StepCommand:
    mode: int
    left: float
    right: float


class RawSerialSender:
    def __init__(self, device: str, baud_rate: int) -> None:
        self.device = device
        self.baud_rate = baud_rate
        self.fd: Optional[int] = None

    def open(self) -> None:
        if self.baud_rate not in BAUD_MAP:
            raise ValueError(f"Unsupported baud_rate={self.baud_rate}")

        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] &= ~(termios.IGNBRK | termios.ICRNL | termios.INLCR | termios.IXON | termios.IXOFF | termios.IXANY)
        attrs[1] &= ~(termios.OPOST | termios.ONLCR | termios.OCRNL)
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CRTSCTS)
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5
        speed = BAUD_MAP[self.baud_rate]
        if hasattr(termios, "cfsetispeed") and hasattr(termios, "cfsetospeed"):
            termios.cfsetispeed(attrs, speed)
            termios.cfsetospeed(attrs, speed)
        else:
            # Some Python builds expose termios speeds only through the raw tcgetattr slots.
            attrs[4] = speed
            attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def send_bytes(self, payload: bytes) -> None:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")

        total = 0
        while total < len(payload):
            written = os.write(self.fd, payload[total:])
            if written <= 0:
                raise RuntimeError("Serial write returned no data")
            total += written


def clamp_norm(value: float) -> float:
    return max(-1.4, min(1.4, value))


def format_command(
    command: StepCommand,
    precision: int,
    prefix: str,
    separator: str,
    suffix: str,
) -> str:
    left = clamp_norm(command.left)
    right = clamp_norm(command.right)
    return (
        f"{prefix}{command.mode}{separator}"
        f"{left:.{precision}f}{separator}{right:.{precision}f}{suffix}"
    )


def send_repeated(
    sender: RawSerialSender,
    command: StepCommand,
    hz: float,
    duration: float,
    precision: int,
    prefix: str,
    separator: str,
    suffix: str,
    dry_run: bool,
    quiet: bool,
    continuous: bool = False,
) -> None:
    payload = format_command(command, precision, prefix, separator, suffix)
    count = None if continuous else max(1, int(math.ceil(max(duration, 0.0) * hz)))
    period = 1.0 / hz
    next_deadline = time.monotonic()
    index = 0

    while count is None or index < count:
        index += 1
        if not quiet:
            if count is None:
                print(f"[{index}] {payload.rstrip()}")
            else:
                print(f"[{index}/{count}] {payload.rstrip()}")
        if not dry_run:
            sender.send_bytes(payload.encode("utf-8"))
        next_deadline += period
        sleep_sec = next_deadline - time.monotonic()
        if sleep_sec > 0:
            time.sleep(sleep_sec)


def parse_step_command(parts: list[str], default_mode: int, default_hz: float, default_duration: float):
    if len(parts) < 2:
        raise ValueError("Need at least: left right")

    if len(parts) == 2:
        mode = default_mode
        left = float(parts[0])
        right = float(parts[1])
        duration = default_duration
        hz = default_hz
    else:
        mode = int(parts[0])
        left = float(parts[1])
        right = float(parts[2])
        duration = float(parts[3]) if len(parts) >= 4 else default_duration
        hz = float(parts[4]) if len(parts) >= 5 else default_hz

    return StepCommand(mode=mode, left=left, right=right), hz, duration


def run_interactive(args: argparse.Namespace, sender: RawSerialSender) -> None:
    print("Interactive mode")
    print("Input format:")
    print("  left right")
    print("  mode left right [duration_sec] [hz]")
    print("Special commands: stop, quit")

    while True:
        try:
            line = input("serial-step> ").strip()
        except EOFError:
            print()
            break

        if not line:
            continue
        if line.lower() in {"quit", "exit"}:
            break
        if line.lower() == "stop":
            send_repeated(
                sender,
                StepCommand(mode=args.mode, left=0.0, right=0.0),
                args.hz,
                args.stop_duration,
                args.precision,
                args.prefix,
                args.separator,
                args.suffix,
                args.dry_run,
                args.quiet,
            )
            continue

        try:
            command, hz, duration = parse_step_command(
                line.split(),
                default_mode=args.mode,
                default_hz=args.hz,
                default_duration=args.duration,
            )
        except ValueError as exc:
            print(f"Invalid input: {exc}", file=sys.stderr)
            continue

        send_repeated(
            sender,
            command,
            hz,
            duration,
            args.precision,
            args.prefix,
            args.separator,
            args.suffix,
            args.dry_run,
            args.quiet,
        )


def second_command_requested(args: argparse.Namespace) -> bool:
    return (
        args.second_mode is not None
        or args.second_left is not None
        or args.second_right is not None
        or args.second_duration is not None
        or args.second_hz is not None
    )


def build_second_command(args: argparse.Namespace) -> tuple[StepCommand, float, float]:
    if args.second_mode is None or args.second_left is None or args.second_right is None:
        raise ValueError(
            "--second-mode, --second-left, and --second-right must be provided together"
        )

    duration = args.second_duration if args.second_duration is not None else args.duration
    hz = args.second_hz if args.second_hz is not None else args.hz
    return (
        StepCommand(mode=args.second_mode, left=args.second_left, right=args.second_right),
        hz,
        duration,
    )


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send raw step commands to the controller over serial")
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device")
    parser.add_argument("--baud", type=int, default=115200, choices=sorted(BAUD_MAP), help="Baud rate")
    parser.add_argument("--mode", type=int, default=0, help="Mode field in [mode,left,right]")
    parser.add_argument("--left", type=float, default=0.0, help="Left normalized step length in [-1, 1]")
    parser.add_argument("--right", type=float, default=0.0, help="Right normalized step length in [-1, 1]")
    parser.add_argument("--hz", type=float, default=20.0, help="Transmit rate")
    parser.add_argument("--duration", type=float, default=2.0, help="Transmit duration in seconds")
    parser.add_argument("--second-mode", type=int, help="Second command mode")
    parser.add_argument("--second-left", type=float, help="Second command left normalized step length")
    parser.add_argument("--second-right", type=float, help="Second command right normalized step length")
    parser.add_argument("--second-hz", type=float, help="Second command transmit rate")
    parser.add_argument("--second-duration", type=float, help="Second command transmit duration in seconds")
    parser.add_argument("--continuous", action="store_true", help="Transmit until Ctrl+C")
    parser.add_argument("--stop-after", action="store_true", help="Send zero command after the test command")
    parser.add_argument("--stop-duration", type=float, default=0.5, help="Stop command duration in seconds")
    parser.add_argument("--precision", type=int, default=3, help="Float precision in the payload")
    parser.add_argument("--prefix", default="[", help="Message prefix")
    parser.add_argument("--separator", default=",", help="Field separator")
    parser.add_argument("--suffix", default="]\\n", help="Message suffix")
    parser.add_argument("--interactive", action="store_true", help="Interactive REPL for manual testing")
    parser.add_argument("--dry-run", action="store_true", help="Print payloads without writing serial")
    parser.add_argument("--quiet", action="store_true", help="Suppress per-packet prints")
    return parser


def main() -> int:
    parser = build_argparser()
    args = parser.parse_args()

    if args.hz <= 0.0:
        parser.error("--hz must be > 0")
    if args.duration < 0.0:
        parser.error("--duration must be >= 0")
    if args.stop_duration < 0.0:
        parser.error("--stop-duration must be >= 0")
    if args.second_hz is not None and args.second_hz <= 0.0:
        parser.error("--second-hz must be > 0")
    if args.second_duration is not None and args.second_duration < 0.0:
        parser.error("--second-duration must be >= 0")
    if second_command_requested(args) and args.continuous:
        parser.error("--continuous cannot be used together with the second command")

    sender = RawSerialSender(args.device, args.baud)
    try:
        if not args.dry_run:
            sender.open()
            print(f"Opened {args.device} @ {args.baud}")

        if args.interactive:
            run_interactive(args, sender)
        else:
            send_repeated(
                sender,
                StepCommand(mode=args.mode, left=args.left, right=args.right),
                args.hz,
                args.duration,
                args.precision,
                args.prefix,
                args.separator,
                args.suffix,
                args.dry_run,
                args.quiet,
                args.continuous,
            )
            if second_command_requested(args):
                second_command, second_hz, second_duration = build_second_command(args)
                send_repeated(
                    sender,
                    second_command,
                    second_hz,
                    second_duration,
                    args.precision,
                    args.prefix,
                    args.separator,
                    args.suffix,
                    args.dry_run,
                    args.quiet,
                )
            if args.stop_after:
                send_repeated(
                    sender,
                    StepCommand(mode=args.mode, left=0.0, right=0.0),
                    args.hz,
                    args.stop_duration,
                    args.precision,
                    args.prefix,
                    args.separator,
                    args.suffix,
                    args.dry_run,
                    args.quiet,
                )
    except KeyboardInterrupt:
        print("\nInterrupted")
        if args.stop_after:
            send_repeated(
                sender,
                StepCommand(mode=args.mode, left=0.0, right=0.0),
                args.hz,
                args.stop_duration,
                args.precision,
                args.prefix,
                args.separator,
                args.suffix,
                args.dry_run,
                args.quiet,
            )
    finally:
        sender.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
