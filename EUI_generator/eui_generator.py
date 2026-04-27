#!/usr/bin/env python3
"""
EUI-64 / EUI-48 generator for locally-administered (non-hardware) addresses.

Algorithm per IEEE 802 §8.2 and https://github.com/things-nyc/random-eui64:
  - Generate random bytes from os.urandom (backed by /dev/urandom)
  - Set  bit 1 of byte 0  (U/L = 1 → locally administered)
  - Clear bit 0 of byte 0 (I/G = 0 → individual / unicast)
"""

import os
import argparse


def _apply_local_individual(b: bytearray) -> bytearray:
    b[0] = (b[0] | 0x02) & 0xFE   # set U/L, clear I/G
    return b


def generate_eui64(sep: str = "-") -> str:
    raw = bytearray(os.urandom(8))
    _apply_local_individual(raw)
    return sep.join(f"{byte:02X}" for byte in raw)


def generate_eui48(sep: str = "-") -> str:
    raw = bytearray(os.urandom(6))
    _apply_local_individual(raw)
    return sep.join(f"{byte:02X}" for byte in raw)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate locally-administered EUI-64 or EUI-48 addresses."
    )
    parser.add_argument(
        "--eui48", action="store_true",
        help="Generate a 48-bit EUI-48 address instead of the default 64-bit"
    )
    parser.add_argument(
        "--colon", action="store_true",
        help="Use colons as separator instead of dashes"
    )
    parser.add_argument(
        "-n", "--count", type=int, default=1, metavar="N",
        help="Number of addresses to generate (default: 1)"
    )
    args = parser.parse_args()

    sep = ":" if args.colon else "-"
    fn = generate_eui48 if args.eui48 else generate_eui64

    for _ in range(args.count):
        print(fn(sep))


if __name__ == "__main__":
    main()
