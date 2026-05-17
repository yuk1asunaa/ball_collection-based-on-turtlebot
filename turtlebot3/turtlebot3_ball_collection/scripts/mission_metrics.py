#!/usr/bin/env python3

import csv
import math
import os
import re
from datetime import datetime
from typing import Optional, Tuple

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node


class MissionMetrics(Node):
    def __init__(self) -> None:
        super().__init__("mission_metrics")

        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("robot_model_name", "waffle")
        self.declare_parameter("robot_name_fallback_substring", "waffle")
        self.declare_parameter("ball_name_regex", "(ball|tennis)")
        self.declare_parameter("metrics_period_sec", 2.0)
        self.declare_parameter("finish_zero_count_threshold", 25)
        self.declare_parameter("output_csv", "/tmp/ball_collection_metrics.csv")

        self.model_states_topic = self.get_parameter("model_states_topic").value
        self.robot_model_name = self.get_parameter("robot_model_name").value
        self.robot_name_fallback_substring = self.get_parameter("robot_name_fallback_substring").value
        self.ball_name_regex = self.get_parameter("ball_name_regex").value
        self.metrics_period_sec = float(self.get_parameter("metrics_period_sec").value)
        self.finish_zero_count_threshold = int(self.get_parameter("finish_zero_count_threshold").value)
        self.output_csv = self.get_parameter("output_csv").value

        self.ball_pattern = re.compile(self.ball_name_regex, re.IGNORECASE)

        self.max_ball_count_seen = 0
        self.current_ball_count = 0
        self.collected_count = 0
        self.zero_count_streak = 0

        self.path_length_m = 0.0
        self.last_robot_xy: Optional[Tuple[float, float]] = None

        self.started = False
        self.finished = False
        self.start_time_sec = 0.0
        self.finish_time_sec = 0.0

        self.sub = self.create_subscription(ModelStates, self.model_states_topic, self.on_model_states, 10)
        self.timer = self.create_timer(self.metrics_period_sec, self.report)

        self.get_logger().info(
            f"MissionMetrics started: topic={self.model_states_topic}, output_csv={self.output_csv}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def find_robot_index(self, msg: ModelStates) -> Optional[int]:
        for i, name in enumerate(msg.name):
            if name == self.robot_model_name:
                return i
        for i, name in enumerate(msg.name):
            if self.robot_name_fallback_substring and self.robot_name_fallback_substring in name:
                return i
        return None

    def count_balls(self, msg: ModelStates, robot_idx: Optional[int]) -> int:
        count = 0
        for i, name in enumerate(msg.name):
            if robot_idx is not None and i == robot_idx:
                continue
            if self.ball_pattern.search(name):
                count += 1
        return count

    def on_model_states(self, msg: ModelStates) -> None:
        robot_idx = self.find_robot_index(msg)
        if robot_idx is not None:
            p = msg.pose[robot_idx].position
            robot_xy = (p.x, p.y)
            if self.last_robot_xy is not None:
                self.path_length_m += math.hypot(robot_xy[0] - self.last_robot_xy[0], robot_xy[1] - self.last_robot_xy[1])
            self.last_robot_xy = robot_xy

        self.current_ball_count = self.count_balls(msg, robot_idx)
        if self.current_ball_count > self.max_ball_count_seen:
            self.max_ball_count_seen = self.current_ball_count

        if not self.started and self.max_ball_count_seen > 0:
            self.started = True
            self.start_time_sec = self.now_sec()

        self.collected_count = max(0, self.max_ball_count_seen - self.current_ball_count)

        if self.started and not self.finished:
            if self.current_ball_count == 0:
                self.zero_count_streak += 1
            else:
                self.zero_count_streak = 0

            if self.zero_count_streak >= self.finish_zero_count_threshold:
                self.finished = True
                self.finish_time_sec = self.now_sec()
                self.write_csv()
                self.report(final=True)

    def elapsed_sec(self) -> float:
        if not self.started:
            return 0.0
        end = self.finish_time_sec if self.finished else self.now_sec()
        return max(0.0, end - self.start_time_sec)

    def coverage(self) -> float:
        if self.max_ball_count_seen <= 0:
            return 0.0
        return 100.0 * float(self.collected_count) / float(self.max_ball_count_seen)

    def efficiency(self) -> float:
        if self.path_length_m <= 1e-6:
            return 0.0
        return float(self.collected_count) / self.path_length_m

    def report(self, final: bool = False) -> None:
        prefix = "FINAL" if final else "METRICS"
        self.get_logger().info(
            f"{prefix}: balls_total_seen={self.max_ball_count_seen}, balls_remaining={self.current_ball_count}, "
            f"balls_collected={self.collected_count}, coverage={self.coverage():.1f}%, "
            f"path_length={self.path_length_m:.2f}m, elapsed={self.elapsed_sec():.1f}s, "
            f"efficiency={self.efficiency():.3f} balls/m"
        )
        self.write_csv_snapshot()

    def write_csv_snapshot(self) -> None:
        out_dir = os.path.dirname(self.output_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        write_header = not os.path.exists(self.output_csv)
        with open(self.output_csv, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    "timestamp",
                    "balls_total_seen",
                    "balls_collected",
                    "balls_remaining",
                    "coverage_percent",
                    "path_length_m",
                    "elapsed_sec",
                    "efficiency_balls_per_m",
                    "final",
                ])
            writer.writerow([
                datetime.now().isoformat(timespec="seconds"),
                self.max_ball_count_seen,
                self.collected_count,
                self.current_ball_count,
                f"{self.coverage():.3f}",
                f"{self.path_length_m:.3f}",
                f"{self.elapsed_sec():.3f}",
                f"{self.efficiency():.6f}",
                "1" if self.finished else "0",
            ])

    def write_csv(self) -> None:
        out_dir = os.path.dirname(self.output_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        write_header = not os.path.exists(self.output_csv)
        with open(self.output_csv, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    "timestamp",
                    "balls_total_seen",
                    "balls_collected",
                    "balls_remaining",
                    "coverage_percent",
                    "path_length_m",
                    "elapsed_sec",
                    "efficiency_balls_per_m",
                ])
            writer.writerow([
                datetime.now().isoformat(timespec="seconds"),
                self.max_ball_count_seen,
                self.collected_count,
                self.current_ball_count,
                f"{self.coverage():.3f}",
                f"{self.path_length_m:.3f}",
                f"{self.elapsed_sec():.3f}",
                f"{self.efficiency():.6f}",
            ])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
