#!/usr/bin/python3

import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class TrajectoryFinishWaiter(Node):
    def __init__(self, topic_name: str):
        super().__init__("trajectory_finish_waiter")
        self._done_event = threading.Event()

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._sub = self.create_subscription(Bool, topic_name, self._callback, qos)
        self.get_logger().info(f"Waiting for trajectory finish signal on '{topic_name}'")

    def _callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("Received trajectory finish signal. Releasing deferred launch.")
            self._done_event.set()

    def wait(self) -> None:
        while rclpy.ok() and not self._done_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Block until trajectory finished Bool signal arrives.")
    parser.add_argument(
        "--topic",
        default="/trajectory_finished",
        help="Topic to listen for std_msgs/Bool trajectory finish signal.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TrajectoryFinishWaiter(args.topic)
    try:
        node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
