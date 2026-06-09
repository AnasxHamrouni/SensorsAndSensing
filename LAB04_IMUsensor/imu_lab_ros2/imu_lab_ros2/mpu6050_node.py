#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

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


def diag_covariance(values: List[float]) -> List[float]:
    # ROS Imu covariance is a flattened 3x3 row-major matrix.
    return [values[0], 0.0, 0.0, 0.0, values[1], 0.0, 0.0, 0.0, values[2]]


class Mpu6050Node(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x68)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("frame_id", "imu_link")

        self.declare_parameter("gyro_cov_diag", [1e-3, 1e-3, 1e-3])
        self.declare_parameter("accel_cov_diag", [1e-2, 1e-2, 1e-2])

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_address = int(self.get_parameter("i2c_address").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        gyro_cov_diag = list(self.get_parameter("gyro_cov_diag").value)
        accel_cov_diag = list(self.get_parameter("accel_cov_diag").value)

        self.angular_velocity_covariance = diag_covariance(gyro_cov_diag)
        self.linear_acceleration_covariance = diag_covariance(accel_cov_diag)

        self.publisher = self.create_publisher(Imu, "/imu/raw", 50)

        self.bus = SMBus(self.i2c_bus)
        # MPU6050 starts in sleep mode after power-up; write 0x00 to wake it.
        self.bus.write_byte_data(self.i2c_address, REG_PWR_MGMT_1, 0x00)

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_imu)

        self.get_logger().info(
            f"MPU6050 node started on /dev/i2c-{self.i2c_bus}, address={hex(self.i2c_address)}, "
            f"rate={publish_rate_hz}Hz, frame_id={self.frame_id}"
        )

    def destroy_node(self) -> bool:
        try:
            self.bus.close()
        except Exception:
            pass
        return super().destroy_node()

    def publish_imu(self) -> None:
        # Read one contiguous sample window: accel(6) + temp(2) + gyro(6).
        data = self.bus.read_i2c_block_data(self.i2c_address, REG_ACCEL_XOUT_H, 14)

        ax_raw = to_int16(data[0], data[1])
        ay_raw = to_int16(data[2], data[3])
        az_raw = to_int16(data[4], data[5])
        # data[6:8] is temperature and intentionally ignored in this lab task.
        gx_raw = to_int16(data[8], data[9])
        gy_raw = to_int16(data[10], data[11])
        gz_raw = to_int16(data[12], data[13])

        gravity = 9.80665

        ax_ms2 = ax_raw * (gravity / 16384.0)
        ay_ms2 = ay_raw * (gravity / 16384.0)
        az_ms2 = az_raw * (gravity / 16384.0)

        gx_rads = gx_raw * ((math.pi / 180.0) / 131.0)
        gy_rads = gy_raw * ((math.pi / 180.0) / 131.0)
        gz_rads = gz_raw * ((math.pi / 180.0) / 131.0)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ROS convention: -1 in first entry means orientation estimate is not provided.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx_rads
        msg.angular_velocity.y = gy_rads
        msg.angular_velocity.z = gz_rads
        msg.angular_velocity_covariance = self.angular_velocity_covariance

        msg.linear_acceleration.x = ax_ms2
        msg.linear_acceleration.y = ay_ms2
        msg.linear_acceleration.z = az_ms2
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
