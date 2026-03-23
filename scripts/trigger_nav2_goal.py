#!/usr/bin/env python3
"""Send a simple Nav2 NavigateToPose goal for dogfooding."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class Nav2GoalTrigger(Node):
    def __init__(self) -> None:
        super().__init__("bagx_nav2_goal_trigger")
        self.latest_amcl_pose: PoseWithCovarianceStamped | None = None
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self._amcl_pose_cb,
            amcl_pose_qos,
        )

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_amcl_pose = msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=-1.0, help="Goal x position in map frame.")
    parser.add_argument("--y", type=float, default=-0.5, help="Goal y position in map frame.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw in radians.")
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=20.0,
        help="Timeout for localization and action server availability in seconds.",
    )
    parser.add_argument(
        "--result-timeout",
        type=float,
        default=20.0,
        help="Timeout for waiting on goal completion in seconds.",
    )
    parser.add_argument(
        "--no-wait-result",
        action="store_true",
        help="Return after goal acceptance instead of waiting for the navigation result.",
    )
    return parser.parse_args()


def wait_for_localization(node: Nav2GoalTrigger, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.latest_amcl_pose is not None:
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    return False


def build_goal(args: argparse.Namespace, node: Nav2GoalTrigger) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    goal.pose = pose
    return goal


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Nav2GoalTrigger()

    try:
        if not node.nav_to_pose_client.wait_for_server(timeout_sec=args.wait_timeout):
            print("navigate_to_pose action unavailable", file=sys.stderr)
            return 2

        if not wait_for_localization(node, timeout=args.wait_timeout):
            print("amcl_pose unavailable", file=sys.stderr)
            return 3

        goal = build_goal(args, node)
        deadline = time.monotonic() + args.wait_timeout
        goal_handle = None
        while time.monotonic() < deadline:
            future = node.nav_to_pose_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            goal_handle = future.result()
            if goal_handle is not None and goal_handle.accepted:
                break
            time.sleep(1.0)

        if goal_handle is None or not goal_handle.accepted:
            print("navigate_to_pose goal rejected", file=sys.stderr)
            return 4

        if args.no_wait_result:
            print(
                f"nav2 goal accepted x={args.x:.2f} y={args.y:.2f} yaw={args.yaw:.2f}",
            )
            return 0

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.result_timeout)
        goal_result = result_future.result()
        if goal_result is None:
            print("navigate_to_pose result timed out", file=sys.stderr)
            return 5

        status = goal_result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f"navigate_to_pose finished with status={status}", file=sys.stderr)
            return 6

        print(
            f"nav2 goal succeeded x={args.x:.2f} y={args.y:.2f} yaw={args.yaw:.2f} status={status}",
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
