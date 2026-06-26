#!/usr/bin/env python3
"""
Quick RMSE evaluator: compares each filter's live pose estimate against
ground truth (tb4 pose bridged from gz sim).

For each filter, tracks:
  - cumulative RMSE: xy (combined, sqrt(mean(dx^2+dy^2))) and orientation
    (sqrt(mean(dtheta^2))), accumulated over every sample seen so far.
  - endstate error: same two quantities, but just the most recent sample —
    read this right when the robot reaches the goal / right before Ctrl+C.

Subscribes directly to each filter's PoseWithCovarianceStamped, NOT the
accumulated Path topics — Path is for visualization; for live per-instant
error pairing you just want the current best estimate, which is what
each filter node already publishes.

Also logs each tracker's cumulative RMSE over time to its own CSV file
(rmse_logs/...), for later plotting with plot_cumulative_rmse.py.

Run (after sourcing your workspace):
    python3 error_handler.py
"""

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
# from tf2_msgs.msg import TFMessage  # only needed for the alt ground-truth
#                                      # callback below, if that's how you
#                                      # bridged it


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


# Each entry: (key, topic, log_path)
#   - key: internal label for this specific node/variant — just needs to
#     be unique, doesn't need to match a "family" name
#   - topic: the PoseWithCovarianceStamped topic THIS node publishes
#   - log_path: where its CSV log gets written (read later by
#     plot_cumulative_rmse.py)
#
# Edit this to match whatever's actually running together in a given sim
# launch. Two variants of the same filter type running concurrently (e.g.
# two KF nodes) is just two rows here with different topics — nothing else
# needs to know they're "the same family."
FILTER_CONFIG = [
    ('KF',  '/pose_kf',  'rmse_logs/rmse_log_KF.csv'),
    ('EKFO', '/pose_ekfo', 'rmse_logs/rmse_log_EKFO.csv'),
    ('EKFI', '/pose_ekfi', 'rmse_logs/rmse_log_EKFI.csv'),
    ('PF',  '/pf_pose',  'rmse_logs/rmse_log_PF.csv'),
]
# ADJUST: index of your robot's pose within the ground-truth PoseArray.
# Confirm with `ros2 topic echo /ground_truth_pose_array` and recheck it if
# you ever change the world file or spawn order — that ordering, not
# anything named/stable, is what determines this.
GROUND_TRUTH_INDEX = 0


class FilterTracker:
    """One of these per filter node/variant being evaluated."""

    def __init__(self, name, log_path=None):
        self.name = name
        self.sum_sq_xy = 0.0
        self.sum_sq_theta = 0.0
        self.n = 0
        self.last_xy_err = None
        self.last_theta_err = None
        self.start_time = time.time()

        self._log_file = None
        self._log_writer = None
        if log_path is not None:
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(log_path, 'w', newline='')
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(['t', 'cum_rmse_xy', 'cum_rmse_theta'])

    def update(self, est_x, est_y, est_yaw, gt_x, gt_y, gt_yaw):
        dx = est_x - gt_x
        dy = est_y - gt_y
        dtheta = wrap_angle(est_yaw - gt_yaw)

        self.sum_sq_xy += dx * dx + dy * dy
        self.sum_sq_theta += dtheta * dtheta
        self.n += 1

        self.last_xy_err = math.hypot(dx, dy)
        self.last_theta_err = abs(dtheta)

        if self._log_writer:
            elapsed = time.time() - self.start_time
            self._log_writer.writerow(
                [elapsed, self.cumulative_rmse_xy(), self.cumulative_rmse_theta()])
            self._log_file.flush()  # so partial data survives a crash, not
                                     # just a clean exit

    def cumulative_rmse_xy(self):
        return math.sqrt(self.sum_sq_xy / self.n) if self.n else float('nan')

    def cumulative_rmse_theta(self):
        return math.sqrt(self.sum_sq_theta / self.n) if self.n else float('nan')

    def report(self):
        return (
            f"[{self.name}] n={self.n}  "
            f"cumulative RMSE: xy={self.cumulative_rmse_xy():.4f} m, "
            f"theta={math.degrees(self.cumulative_rmse_theta()):.2f} deg  |  "
            f"endstate: xy={self.last_xy_err:.4f} m, "
            f"theta={math.degrees(self.last_theta_err):.2f} deg"
        )

    def close(self):
        if self._log_file:
            self._log_file.close()


class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')

        # Latest ground truth, cached and paired against whichever filter
        # estimate arrives next. Fine as long as ground truth publishes at
        # least as often as the filters do — if you need tighter sync
        # later, swap to message_filters.ApproximateTimeSynchronizer.
        self.gt_x = None
        self.gt_y = None
        self.gt_yaw = None
        self.gt_index = GROUND_TRUTH_INDEX

        # --- Ground truth ---
        # Bridged as a PoseArray (e.g. gz.msgs.Pose_V -> geometry_msgs/msg/PoseArray);
        # individual entries don't carry names, so the robot's pose has to be
        # picked out by a fixed index instead.
        self.create_subscription(
            PoseArray, '/tb4_dynamic_pose', self.gt_callback, 10)

        # --- Filter estimates, one tracker + subscription per FILTER_CONFIG row ---
        self.trackers = {}
        for key, topic, log_path in FILTER_CONFIG:
            self.trackers[key] = FilterTracker(key, log_path=log_path)
            self.create_subscription(
                PoseWithCovarianceStamped, topic, self._make_filter_cb(key), 10)

        # Periodic summary; also printed once more on shutdown (Ctrl+C),
        # which doubles as your "endstate" readout right after a test run.
        self.create_timer(2.0, self.print_summary)

    def gt_callback(self, msg: PoseArray):
        if len(msg.poses) <= self.gt_index:
            self.get_logger().warn(
                f"PoseArray has {len(msg.poses)} poses but gt_index="
                f"{self.gt_index} — check the index/bridge config.",
                throttle_duration_sec=5.0)
            return
        pose = msg.poses[self.gt_index]
        self.gt_x = pose.position.x
        self.gt_y = pose.position.y
        self.gt_yaw = yaw_from_quat(pose.orientation)

    # def gt_callback_tf(self, msg):  # tf2_msgs.msg.TFMessage
    #     GROUND_TRUTH_FRAME = 'turtlebot4'  # ADJUST: check `ros2 topic echo`
    #     for t in msg.transforms:
    #         if t.child_frame_id == GROUND_TRUTH_FRAME:
    #             self.gt_x = t.transform.translation.x
    #             self.gt_y = t.transform.translation.y
    #             self.gt_yaw = yaw_from_quat(t.transform.rotation)
    #             return

    def _make_filter_cb(self, key):
        def callback(msg: PoseWithCovarianceStamped):
            if self.gt_x is None:
                return  # no ground truth received yet
            est = msg.pose.pose
            self.trackers[key].update(
                est.position.x, est.position.y, yaw_from_quat(est.orientation),
                self.gt_x, self.gt_y, self.gt_yaw)
        return callback

    def print_summary(self):
        for tracker in self.trackers.values():
            if tracker.n > 0:
                self.get_logger().info(tracker.report())


def main():
    rclpy.init()
    node = ErrorHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        for tracker in node.trackers.values():
            tracker.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
