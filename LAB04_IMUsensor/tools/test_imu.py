#!/usr/bin/env python3
import argparse
import math
import time

try:
    from smbus2 import SMBus
except Exception:
    from smbus import SMBus


REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_XOUT_H = 0x3B


def to_int16(msb: int, lsb: int) -> int:
    value = (msb << 8) | lsb
    if value & 0x8000:
        value = -((0xFFFF - value) + 1)
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description="Quick MPU6050 readout")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x68)
    parser.add_argument("--samples", type=int, default=0, help="0 means infinite")
    parser.add_argument("--delay", type=float, default=0.05)
    args = parser.parse_args()

    g = 9.80665
    sample_count = 0
    with SMBus(args.bus) as bus:
        bus.write_byte_data(args.address, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        while True:
            data = bus.read_i2c_block_data(args.address, REG_ACCEL_XOUT_H, 14)

            ax = to_int16(data[0], data[1])
            ay = to_int16(data[2], data[3])
            az = to_int16(data[4], data[5])
            gx = to_int16(data[8], data[9])
            gy = to_int16(data[10], data[11])
            gz = to_int16(data[12], data[13])

            ax_ms2 = ax * (g / 16384.0)
            ay_ms2 = ay * (g / 16384.0)
            az_ms2 = az * (g / 16384.0)

            gx_rads = gx * ((math.pi / 180.0) / 131.0)
            gy_rads = gy * ((math.pi / 180.0) / 131.0)
            gz_rads = gz * ((math.pi / 180.0) / 131.0)

            print(f"accel [m/s^2]: {ax_ms2:.4f} {ay_ms2:.4f} {az_ms2:.4f}")
            print(f"gyro  [rad/s]: {gx_rads:.4f} {gy_rads:.4f} {gz_rads:.4f}")
            print("-")

            sample_count += 1
            if args.samples > 0 and sample_count >= args.samples:
                break
            time.sleep(args.delay)


if __name__ == "__main__":
    main()
