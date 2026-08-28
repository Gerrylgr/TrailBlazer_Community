#!/usr/bin/env python3
from collections import deque
from statistics import mean, pstdev

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class LidarImuOffsetChecker(Node):
    def __init__(self):
        super().__init__('lidar_imu_offset_checker')

        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('lidar_topic', '/points_raw')
        self.declare_parameter('imu_buffer_size', 2000)
        self.declare_parameter('report_every', 20)

        imu_topic = self.get_parameter('imu_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        imu_buffer_size = int(self.get_parameter('imu_buffer_size').value)
        self.report_every = int(self.get_parameter('report_every').value)

        self.imu_times = deque(maxlen=imu_buffer_size)
        self.offsets = []

        self.create_subscription(Imu, imu_topic, self.imu_callback, 100)
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)

        self.lidar_count = 0

        self.get_logger().info(
            f'Started. imu_topic={imu_topic}, lidar_topic={lidar_topic}'
        )

    def imu_callback(self, msg: Imu):
        self.imu_times.append(stamp_to_sec(msg.header.stamp))

    def lidar_callback(self, msg: PointCloud2):
        if not self.imu_times:
            self.get_logger().warn('No IMU messages received yet.')
            return

        lidar_t = stamp_to_sec(msg.header.stamp)

        # 找最近的 IMU 时间戳
        nearest_imu_t = min(self.imu_times, key=lambda t: abs(t - lidar_t))
        offset = lidar_t - nearest_imu_t   # >0 表示 lidar 时间戳比最近 imu 更晚

        self.offsets.append(offset)
        self.lidar_count += 1

        if self.lidar_count % self.report_every == 0:
            avg = mean(self.offsets)
            sd = pstdev(self.offsets) if len(self.offsets) > 1 else 0.0
            mn = min(self.offsets)
            mx = max(self.offsets)

            self.get_logger().info(
                f'[samples={len(self.offsets)}] '
                f'lidar - nearest_imu = avg {avg*1000:.3f} ms, '
                f'std {sd*1000:.3f} ms, '
                f'min {mn*1000:.3f} ms, max {mx*1000:.3f} ms'
            )


def main():
    rclpy.init()
    node = LidarImuOffsetChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.offsets:
            avg = mean(node.offsets)
            sd = pstdev(node.offsets) if len(node.offsets) > 1 else 0.0
            mn = min(node.offsets)
            mx = max(node.offsets)
            node.get_logger().info(
                f'FINAL: avg={avg*1000:.3f} ms, std={sd*1000:.3f} ms, '
                f'min={mn*1000:.3f} ms, max={mx*1000:.3f} ms'
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()