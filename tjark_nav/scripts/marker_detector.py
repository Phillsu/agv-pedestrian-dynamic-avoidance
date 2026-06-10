#!/usr/bin/env python3
"""
marker_detector.py
──────────────────
RGB-D vision node that finds the START and END points of the mission during
the mapping phase.

How it works
────────────
A bright GREEN square panel is mounted on a sign next to the START point and a
bright RED square panel next to the END point (see worlds/obstacle_avoidance.world).
While the robot drives around building the RTAB-Map, this node:

  1. Detects the coloured square in the RGB image (HSV threshold + contour).
  2. Reads the matching 3-D point straight out of the organised depth point
     cloud at the square's centroid pixel — no pin-hole / optical-frame maths
     needed, the cloud already carries correct geometry in its own frame.
  3. Transforms that point into the `map` frame via TF.
  4. Once a colour has been seen on enough consecutive frames at a stable
     location, it LOCKS the position, publishes a coloured RViz marker
     (green sphere = start, red sphere = end) and writes both waypoints to a
     YAML file that demo.launch / navigation_demo.py consume.

Because the depth camera far-clip is 3 m, drive the robot to within ~3 m of
each sign and point the camera at it; the lock fires automatically.

Parameters (~private)
  ~rgb_topic            (str)   default /my_camera/color/image_raw
  ~cloud_topic          (str)   default /my_camera/depth/points
  ~map_frame            (str)   default map
  ~output_file          (str)   default ../maps/waypoints.yaml
  ~marker_topic         (str)   default /detected_markers
  ~min_area_px          (int)   default 600    min coloured blob area
  ~lock_frames          (int)   default 8      stable frames before locking
  ~lock_tolerance_m     (float) default 0.40   max spread within window (m)
"""

import os
import math
from collections import deque

import numpy as np
import cv2
import yaml

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

# ── HSV colour gates (OpenCV H ∈ [0,179]) ─────────────────────────────────────
#   Green: single band.   Red: wraps 0°, so two bands OR'd together.
_GREEN_LO = np.array([40,  80,  50], dtype=np.uint8)
_GREEN_HI = np.array([85, 255, 255], dtype=np.uint8)

_RED_LO1 = np.array([0,   90,  60], dtype=np.uint8)
_RED_HI1 = np.array([10, 255, 255], dtype=np.uint8)
_RED_LO2 = np.array([170, 90,  60], dtype=np.uint8)
_RED_HI2 = np.array([179, 255, 255], dtype=np.uint8)


def _yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MarkerDetector:
    def __init__(self):
        rospy.init_node("marker_detector")

        rgb_topic   = rospy.get_param("~rgb_topic",   "/my_camera/color/image_raw")
        cloud_topic = rospy.get_param("~cloud_topic", "/my_camera/depth/points")
        self._map_frame    = rospy.get_param("~map_frame", "map")
        self._min_area     = int(rospy.get_param("~min_area_px", 600))
        self._lock_frames  = int(rospy.get_param("~lock_frames", 8))
        self._lock_tol     = float(rospy.get_param("~lock_tolerance_m", 0.40))
        marker_topic       = rospy.get_param("~marker_topic", "/detected_markers")

        default_out = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "../maps/waypoints.yaml")
        self._outfile = os.path.abspath(rospy.get_param("~output_file", default_out))

        self._bridge = CvBridge()
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf)

        # rolling windows of recent map-frame detections, and locked results
        self._window = {"start": deque(maxlen=self._lock_frames),
                        "end":   deque(maxlen=self._lock_frames)}
        self._locked = {"start": None, "end": None}

        self._marker_pub = rospy.Publisher(marker_topic, MarkerArray,
                                           queue_size=1, latch=True)

        # synchronise colour image with depth cloud (organised, same camera)
        sub_rgb   = message_filters.Subscriber(rgb_topic,   Image)
        sub_cloud = message_filters.Subscriber(cloud_topic, PointCloud2)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_cloud], queue_size=5, slop=0.3)
        self._sync.registerCallback(self._cb)

        # re-publish markers periodically so late RViz subscribers see them
        rospy.Timer(rospy.Duration(2.0), lambda _e: self._publish_markers())

        rospy.loginfo("[marker_detector] ready")
        rospy.loginfo("[marker_detector]   GREEN square → START,  RED square → END")
        rospy.loginfo("[marker_detector]   RGB:   %s", rgb_topic)
        rospy.loginfo("[marker_detector]   cloud: %s", cloud_topic)
        rospy.loginfo("[marker_detector]   waypoints → %s", self._outfile)

    # ── per-colour blob centroid in the RGB image ────────────────────────────
    def _find_centroid(self, hsv, which):
        if which == "start":
            mask = cv2.inRange(hsv, _GREEN_LO, _GREEN_HI)
        else:
            mask = cv2.inRange(hsv, _RED_LO1, _RED_HI1) | \
                   cv2.inRange(hsv, _RED_LO2, _RED_HI2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self._min_area:
            return None
        m = cv2.moments(c)
        if m["m00"] == 0:
            return None
        return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

    # ── XYZ from the organised cloud near pixel (u,v), searching a window ─────
    def _cloud_point(self, cloud, u, v):
        w, h = cloud.width, cloud.height
        if h <= 1:          # not an organised cloud → cannot index by pixel
            return None
        for r in (0, 2, 4, 6, 8):           # expand search ring until valid
            uvs = []
            for du in range(-r, r + 1):
                for dv in range(-r, r + 1):
                    uu, vv = u + du, v + dv
                    if 0 <= uu < w and 0 <= vv < h:
                        uvs.append((uu, vv))
            if not uvs:
                continue
            for px, py, pz in pc2.read_points(cloud, field_names=("x", "y", "z"),
                                              skip_nans=True, uvs=uvs):
                if np.isfinite(px) and np.isfinite(py) and np.isfinite(pz):
                    return px, py, pz
        return None

    def _to_map(self, cloud_frame, stamp, xyz):
        ps = PointStamped()
        ps.header.frame_id = cloud_frame
        ps.header.stamp    = stamp
        ps.point.x, ps.point.y, ps.point.z = xyz
        try:
            out = self._tf_buf.transform(ps, self._map_frame,
                                         timeout=rospy.Duration(0.5))
            return out.point.x, out.point.y
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            rospy.logwarn_throttle(5.0, "[marker_detector] TF to %s failed: %s",
                                   self._map_frame, exc)
            return None

    def _cb(self, img_msg, cloud_msg):
        if self._locked["start"] and self._locked["end"]:
            return  # nothing left to find

        try:
            bgr = self._bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[marker_detector] cv_bridge: %s", exc)
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        for which in ("start", "end"):
            if self._locked[which]:
                continue
            uv = self._find_centroid(hsv, which)
            if uv is None:
                continue
            xyz = self._cloud_point(cloud_msg, *uv)
            if xyz is None:
                continue
            mp = self._to_map(cloud_msg.header.frame_id,
                              cloud_msg.header.stamp, xyz)
            if mp is None:
                continue
            self._window[which].append(mp)
            self._try_lock(which)

    def _try_lock(self, which):
        win = self._window[which]
        if len(win) < self._lock_frames:
            return
        arr = np.array(win)
        spread = float(np.max(np.linalg.norm(arr - arr.mean(axis=0), axis=1)))
        if spread > self._lock_tol:
            return  # robot/marker still moving relative to each other
        x, y = float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))
        self._locked[which] = {"x": round(x, 3), "y": round(y, 3)}
        label = "START (green)" if which == "start" else "END (red)"
        rospy.loginfo("[marker_detector] ✓ LOCKED %s at map (%.2f, %.2f)",
                      label, x, y)
        self._finalise_yaws()
        self._save()
        self._publish_markers()

    def _finalise_yaws(self):
        s, e = self._locked["start"], self._locked["end"]
        if s and e:
            yaw = math.atan2(e["y"] - s["y"], e["x"] - s["x"])
            s["yaw"] = round(yaw, 4)
            e["yaw"] = round(yaw, 4)
        else:
            for wp in (s, e):
                if wp is not None:
                    wp.setdefault("yaw", 0.0)

    def _save(self):
        data = {}
        if self._locked["start"]:
            data["start"] = self._locked["start"]
        if self._locked["end"]:
            data["end"] = self._locked["end"]
        if not data:
            return
        os.makedirs(os.path.dirname(self._outfile), exist_ok=True)
        with open(self._outfile, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        rospy.loginfo("[marker_detector] waypoints saved → %s", self._outfile)

    # ── RViz markers: green sphere = start, red sphere = end (+ labels) ───────
    def _publish_markers(self):
        arr = MarkerArray()
        mid = 0
        for which, rgb in (("start", (0.0, 1.0, 0.0)), ("end", (1.0, 0.0, 0.0))):
            wp = self._locked[which]
            if wp is None:
                continue
            sphere = Marker()
            sphere.header.frame_id = self._map_frame
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
            text.header.frame_id = self._map_frame
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
            text.text = which.upper()
            arr.markers.append(text)
        if arr.markers:
            self._marker_pub.publish(arr)


if __name__ == "__main__":
    try:
        MarkerDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
