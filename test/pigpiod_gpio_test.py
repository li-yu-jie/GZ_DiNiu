#!/usr/bin/env python3
"""使用 pigpiod 单独控制 BCM17 或 BCM27 的高低电平。"""

import argparse
import sys
import time

import pigpio


VALID_PINS = (17, 27)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单独控制 BCM17 或 BCM27 输出高低电平。"
    )
    parser.add_argument(
        "--pin",
        type=int,
        required=True,
        choices=VALID_PINS,
        help="要控制的 BCM 引脚，只能是 17 或 27。",
    )
    parser.add_argument(
        "--level",
        type=int,
        required=True,
        choices=(0, 1),
        help="输出电平：0=低，1=高。",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=0.0,
        help="保持秒数；>0 时到时后自动拉低该引脚。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pi = pigpio.pi()
    if not pi.connected:
        print("无法连接到 pigpiod，请先启动: sudo systemctl start pigpiod")
        return 1

    try:
        pi.set_mode(args.pin, pigpio.OUTPUT)
        pi.write(args.pin, args.level)
        print(f"已设置 BCM{args.pin}={args.level}")

        if args.hold > 0:
            time.sleep(args.hold)
            pi.write(args.pin, 0)
            print(f"保持 {args.hold:.2f}s 后已自动拉低 BCM{args.pin}")
        return 0
    finally:
        pi.stop()


if __name__ == "__main__":
    sys.exit(main())
