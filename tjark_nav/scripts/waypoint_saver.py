#!/usr/bin/env python3
"""
waypoint_saver.py
─────────────────
Saves the AGV's start and end waypoints during a mapping session.

In RViz while mapping:
  1. Click "2D Pose Estimate"  →  sets the START waypoint
  2. Click "2D Nav Goal"       →  sets the END   waypoint

The file is written (or overwritten) to ~output_file as soon as both
waypoints have been received.  Re-clicking either tool updates that
waypoint and re-saves immediately.
"""
import math
import os
import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointSaver:
    def __init__(self):
        rospy.init_node("waypoint_saver")

        default_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../maps/waypoints.yaml"
        )
        self._outfile = os.path.abspath(
            rospy.get_param("~output_file", default_path)
        )

        self._start = None
        self._end   = None

        rospy.Subscriber("/initialpose",         PoseWithCovarianceStamped, self._cb_start)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,             self._cb_end)

        rospy.loginfo("[waypoint_saver] Ready — saving to: %s", self._outfile)
        rospy.loginfo("[waypoint_saver]   RViz '2D Pose Estimate' → START")
        rospy.loginfo("[waypoint_saver]   RViz '2D Nav Goal'      → END")
        rospy.spin()

    def _cb_start(self, msg):
        p = msg.pose.pose
        self._start = {
            "x":   round(p.position.x, 3),
            "y":   round(p.position.y, 3),
            "yaw": round(_quat_to_yaw(p.orientation), 4),
        }
        rospy.loginfo("[waypoint_saver] START → (%.2f, %.2f, yaw=%.2f)",
                      self._start["x"], self._start["y"], self._start["yaw"])
        self._save()

    def _cb_end(self, msg):
        p = msg.pose
        self._end = {
            "x":   round(p.position.x, 3),
            "y":   round(p.position.y, 3),
            "yaw": round(_quat_to_yaw(p.orientation), 4),
        }
        rospy.loginfo("[waypoint_saver] END   → (%.2f, %.2f, yaw=%.2f)",
                      self._end["x"], self._end["y"], self._end["yaw"])
        self._save()

    def _save(self):
        if self._start is None:
            rospy.loginfo("[waypoint_saver] Waiting for START (use 2D Pose Estimate)…")
            return
        if self._end is None:
            rospy.loginfo("[waypoint_saver] Waiting for END (use 2D Nav Goal)…")
            return

        data = {"start": self._start, "end": self._end}
        os.makedirs(os.path.dirname(self._outfile), exist_ok=True)
        with open(self._outfile, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        rospy.loginfo("[waypoint_saver] ✓ Waypoints saved → %s", self._outfile)


if __name__ == "__main__":
    WaypointSaver()
