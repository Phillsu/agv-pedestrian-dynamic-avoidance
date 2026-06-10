#!/usr/bin/env python3
"""
navigation_demo.py
──────────────────
Sends two sequential navigation goals to move_base, demonstrating two distinct
obstacle-avoidance events with pedestrian crowds.

  Start  : (-10, -5)   ← robot spawn position
  Goal 1 : ( -2.8, -5.9) static obstacle bypass
  Goal 2 : (  1.0, -1.0) central corridor checkpoint
  Goal 3 : (  9.0,  5.0) final upper goal

The script waits for move_base to be ready, optionally publishes the known
initial pose to AMCL, then drives the full sequence autonomously.
"""

import math
import os
import yaml
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

# ── defaults (used when ROS params are not set) ───────────────────────────────
_DEFAULT_GOALS = [
    {"label": "Goal 1 – static obstacle bypass", "x": -2.8, "y": -5.9,
     "yaw": 0.0, "pass_through": True},
    {"label": "Goal 2 – central corridor checkpoint", "x": 1.0, "y": -1.0, "yaw": 0.0},
    {"label": "Goal 3 – final upper goal", "x": 9.0, "y": 5.0, "yaw": 0.0},
]

GOAL_TIMEOUT_S  = 180.0
AMCL_SETTLE_S   = 12.0
BETWEEN_GOALS_S = 3.0
PASS_THROUGH_RADIUS_M = 1.0
# ─────────────────────────────────────────────────────────────────────────────


def _load_waypoints(path: str):
    """
    Load start/end waypoints saved by waypoint_saver.py.
    Returns (start_dict, end_dict) or (None, None) if unavailable.
    """
    if not path:
        return None, None
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        rospy.logwarn("[nav_demo] waypoints file not found: %s", path)
        return None, None
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
        start = data.get("start")
        end   = data.get("end")
        if start and end:
            rospy.loginfo("[nav_demo] Loaded waypoints from %s", path)
            rospy.loginfo("[nav_demo]   START: (%.2f, %.2f, yaw=%.2f)",
                          start["x"], start["y"], start.get("yaw", 0.0))
            rospy.loginfo("[nav_demo]   END:   (%.2f, %.2f, yaw=%.2f)",
                          end["x"],   end["y"],   end.get("yaw", 0.0))
            return start, end
        rospy.logwarn("[nav_demo] waypoints file missing 'start' or 'end' key")
    except Exception as exc:
        rospy.logwarn("[nav_demo] failed to load waypoints: %s", exc)
    return None, None


def _yaw_to_quat(yaw: float) -> tuple:
    """Return (qz, qw) for a pure-yaw quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_goal(x: float, y: float, yaw: float = 0.0) -> MoveBaseGoal:
    qz, qw = _yaw_to_quat(yaw)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.pose.position.x        = x
    goal.target_pose.pose.position.y        = y
    goal.target_pose.pose.orientation.z     = qz
    goal.target_pose.pose.orientation.w     = qw
    return goal


def publish_initial_pose(x: float, y: float, yaw: float, cov_xy: float = 0.25,
                         cov_aa: float = 0.0685) -> None:
    """
    Publish an /initialpose message so AMCL seeds its particle cloud at the
    known spawn location rather than waiting for the particle filter to drift.
    """
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                          queue_size=1, latch=True)
    rospy.sleep(0.5)   # allow subscriber handshake

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp    = rospy.Time.now()
    msg.pose.pose.position.x    = x
    msg.pose.pose.position.y    = y
    qz, qw = _yaw_to_quat(yaw)
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    # diagonal covariance [x, y, z, rx, ry, rz]
    msg.pose.covariance[0]  = cov_xy   # x
    msg.pose.covariance[7]  = cov_xy   # y
    msg.pose.covariance[35] = cov_aa   # yaw

    pub.publish(msg)
    rospy.loginfo("[nav_demo] published /initialpose  (%.1f, %.1f, yaw=%.2f)", x, y, yaw)


def publish_waypoint_markers(start: dict, end: dict) -> rospy.Publisher:
    """
    Re-publish the camera-detected start (green) and end (red) points as RViz
    markers so the recognised locations stay visible during navigation.
    Returns the latched publisher (keep a reference so it is not GC'd).
    """
    pub = rospy.Publisher("/detected_markers", MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.3)
    arr = MarkerArray()
    mid = 0
    for wp, rgb, label in ((start, (0.0, 1.0, 0.0), "START"),
                           (end,   (1.0, 0.0, 0.0), "END")):
        if not wp:
            continue
        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = rospy.Time.now()
        sphere.ns = "detected_markers"
        sphere.id = mid; mid += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = wp["x"]
        sphere.pose.position.y = wp["y"]
        sphere.pose.position.z = 0.3
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6
        sphere.color.r, sphere.color.g, sphere.color.b = rgb
        sphere.color.a = 0.9
        arr.markers.append(sphere)

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = rospy.Time.now()
        text.ns = "detected_markers"
        text.id = mid; mid += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = wp["x"]
        text.pose.position.y = wp["y"]
        text.pose.position.z = 1.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.5
        text.color.r, text.color.g, text.color.b = rgb
        text.color.a = 1.0
        text.text = label
        arr.markers.append(text)
    pub.publish(arr)
    return pub


def navigate_to(client: actionlib.SimpleActionClient,
                x: float, y: float, yaw: float, label: str) -> bool:
    """Send a single goal, block until done (or timeout), return success."""
    rospy.loginfo("[nav_demo] ── Sending  %s  → (%.1f, %.1f) ──", label, x, y)
    goal = make_goal(x, y, yaw)
    client.send_goal(goal)

    finished = client.wait_for_result(rospy.Duration(GOAL_TIMEOUT_S))
    if not finished:
        rospy.logwarn("[nav_demo] %s timed out after %.0f s", label, GOAL_TIMEOUT_S)
        client.cancel_goal()
        return False

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("[nav_demo] ✓  %s  SUCCEEDED", label)
        return True

    state_str = {
        GoalStatus.ABORTED:   "ABORTED",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.REJECTED:  "REJECTED",
    }.get(state, f"state={state}")
    rospy.logwarn("[nav_demo] ✗  %s  %s", label, state_str)
    return False


def navigate_through(client: actionlib.SimpleActionClient, pose_ref: dict,
                     x: float, y: float, yaw: float, label: str,
                     radius: float = PASS_THROUGH_RADIUS_M) -> bool:
    """Send a goal and continue once the robot passes within radius."""
    rospy.loginfo("[nav_demo] ── Passing  %s  → (%.1f, %.1f), radius %.1f m ──",
                  label, x, y, radius)
    goal = make_goal(x, y, yaw)
    client.send_goal(goal)

    deadline = rospy.Time.now() + rospy.Duration(GOAL_TIMEOUT_S)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        pose = pose_ref.get("pose")
        if pose is not None:
            dx = pose.position.x - x
            dy = pose.position.y - y
            if math.hypot(dx, dy) <= radius:
                rospy.loginfo("[nav_demo] ✓  passed %s, continuing", label)
                client.cancel_goal()
                rospy.sleep(0.2)
                return True

        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[nav_demo] ✓  %s  SUCCEEDED before pass-through switch", label)
            return True
        if state in (GoalStatus.ABORTED, GoalStatus.REJECTED):
            rospy.logwarn("[nav_demo] ✗  %s  failed before pass-through radius", label)
            return False
        rate.sleep()

    rospy.logwarn("[nav_demo] %s pass-through timed out after %.0f s",
                  label, GOAL_TIMEOUT_S)
    client.cancel_goal()
    return False


def main():
    rospy.init_node("navigation_demo", anonymous=False)

    # ── 1. try saved waypoints file (set during mapping) ─────────────────────
    wp_file        = rospy.get_param("~waypoints_file", "")
    wp_start, wp_end = _load_waypoints(wp_file)
    _marker_pub = None  # keep a ref so the latched publisher stays alive

    if wp_start and wp_end:
        init_x = wp_start["x"]
        init_y = wp_start["y"]
        init_a = wp_start.get("yaw", 0.0)
        goals  = [{"x": wp_end["x"], "y": wp_end["y"],
                   "yaw": wp_end.get("yaw", 0.0), "label": "Saved end waypoint"}]
        rospy.loginfo("[nav_demo] Using saved waypoints (start→end mission)")
        _marker_pub = publish_waypoint_markers(wp_start, wp_end)
    else:
        # ── 2. fall back to launch-file params or hardcoded defaults ─────────
        goals_param = rospy.get_param("~goals", None)
        if goals_param is not None:
            if isinstance(goals_param, str):
                goals_param = yaml.safe_load(goals_param)
            goals = goals_param
        else:
            goals = _DEFAULT_GOALS

        init_x = rospy.get_param("~initial_pose_x", -10.0)
        init_y = rospy.get_param("~initial_pose_y",  -5.0)
        init_a = rospy.get_param("~initial_pose_a",  0.0)

    # ── wait for move_base ────────────────────────────────────────────────────
    rospy.loginfo("[nav_demo] waiting for move_base action server…")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(90.0)):
        rospy.logerr("[nav_demo] move_base did not become ready in time — aborting")
        return
    rospy.loginfo("[nav_demo] move_base ready")

    # ── seed AMCL at spawn position ───────────────────────────────────────────
    publish_initial_pose(x=init_x, y=init_y, yaw=init_a)
    pose_ref = {"pose": None}
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                     lambda msg: pose_ref.__setitem__("pose", msg.pose.pose),
                     queue_size=1)

    rospy.loginfo(
        "[nav_demo] waiting %.0f s for AMCL to converge…", AMCL_SETTLE_S
    )
    rospy.sleep(AMCL_SETTLE_S)

    # ── run goals in sequence ─────────────────────────────────────────────────
    results = []
    for g in goals:
        if g.get("pass_through", False):
            ok = navigate_through(client, pose_ref, g["x"], g["y"], g["yaw"], g["label"],
                                  float(g.get("pass_radius", PASS_THROUGH_RADIUS_M)))
        else:
            ok = navigate_to(client, g["x"], g["y"], g["yaw"], g["label"])
        results.append((g["label"], ok))
        if not ok:
            rospy.logwarn("[nav_demo] Stopping sequence after failed goal.")
            break
        if g is not goals[-1] and not g.get("pass_through", False):
            rospy.sleep(BETWEEN_GOALS_S)

    # ── summary ───────────────────────────────────────────────────────────────
    rospy.loginfo("[nav_demo] ── Demo Summary ──────────────────────────────")
    for label, ok in results:
        status = "SUCCEEDED ✓" if ok else "FAILED    ✗"
        rospy.loginfo("[nav_demo]   %s  →  %s", label, status)
    rospy.loginfo("[nav_demo] ─────────────────────────────────────────────")

    if all(ok for _, ok in results):
        rospy.loginfo("[nav_demo] Full demo completed successfully!")
    else:
        rospy.logwarn("[nav_demo] Demo ended with failures – see above.")


if __name__ == "__main__":
    main()
