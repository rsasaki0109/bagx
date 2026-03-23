#!/usr/bin/env python3
"""Send a simple MoveIt planning request for dogfooding."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


DEFAULT_JOINT_TARGETS = (
    ("panda_joint1", 0.0),
    ("panda_joint2", -0.6),
    ("panda_joint3", 0.0),
    ("panda_joint4", -2.0),
    ("panda_joint5", 0.0),
    ("panda_joint6", 1.6),
    ("panda_joint7", 0.8),
)


class MoveItPlanTrigger(Node):
    def __init__(self, controller_action: str) -> None:
        super().__init__("bagx_moveit_plan_trigger")
        self.latest_joint_state: JointState | None = None
        self.move_action_client = ActionClient(self, MoveGroup, "/move_action")
        self.controller_client = ActionClient(self, FollowJointTrajectory, controller_action)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState) -> None:
        if msg.name:
            self.latest_joint_state = msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--group",
        default="panda_arm",
        help="MoveIt planning group (default: %(default)s).",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=20.0,
        help="Timeout for service and joint state discovery in seconds.",
    )
    parser.add_argument(
        "--allowed-planning-time",
        type=float,
        default=5.0,
        help="Allowed planning time in seconds.",
    )
    parser.add_argument(
        "--result-timeout",
        type=float,
        default=30.0,
        help="Timeout for waiting on action completion in seconds.",
    )
    parser.add_argument(
        "--controller-action",
        default="/panda_arm_controller/follow_joint_trajectory",
        help="Trajectory controller action name used to confirm execution readiness.",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Request planning only instead of planning and execution.",
    )
    return parser.parse_args()


def build_goal(args: argparse.Namespace, joint_state: JointState | None) -> MoveGroup.Goal:
    goal = MoveGroup.Goal()
    motion_plan_request = goal.request
    motion_plan_request.group_name = args.group
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = args.allowed_planning_time
    motion_plan_request.max_velocity_scaling_factor = 0.2
    motion_plan_request.max_acceleration_scaling_factor = 0.2

    if joint_state is not None:
        motion_plan_request.start_state.joint_state = joint_state

    constraints = Constraints()
    for joint_name, position in DEFAULT_JOINT_TARGETS:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = position
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    motion_plan_request.goal_constraints = [constraints]
    goal.planning_options.plan_only = args.plan_only
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    return goal


def wait_for_joint_state(node: MoveItPlanTrigger, timeout: float) -> JointState | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.latest_joint_state is not None:
            return node.latest_joint_state
        rclpy.spin_once(node, timeout_sec=0.2)
    return None


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MoveItPlanTrigger(args.controller_action)

    try:
        if not node.move_action_client.wait_for_server(timeout_sec=args.wait_timeout):
            print("MoveGroup action unavailable: /move_action", file=sys.stderr)
            return 2
        if not args.plan_only and not node.controller_client.wait_for_server(timeout_sec=args.wait_timeout):
            print(
                f"trajectory controller action unavailable: {args.controller_action}",
                file=sys.stderr,
            )
            return 3

        joint_state = wait_for_joint_state(node, timeout=args.wait_timeout)
        goal = build_goal(args, joint_state)
        goal_future = node.move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=args.wait_timeout)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("MoveGroup goal rejected", file=sys.stderr)
            return 4

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.result_timeout)
        action_result = result_future.result()
        if action_result is None:
            print("MoveGroup result timed out", file=sys.stderr)
            return 5

        result = action_result.result
        planned_points = len(result.planned_trajectory.joint_trajectory.points)
        executed_points = len(result.executed_trajectory.joint_trajectory.points)
        print(
            "moveit "
            f"status={action_result.status} "
            f"error_code={result.error_code.val} "
            f"planned_points={planned_points} "
            f"executed_points={executed_points} "
            f"plan_only={args.plan_only}"
        )
        return 0 if (
            result.error_code.val == 1
            and action_result.status == GoalStatus.STATUS_SUCCEEDED
        ) else 6
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
