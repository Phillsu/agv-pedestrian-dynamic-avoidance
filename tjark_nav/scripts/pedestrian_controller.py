#!/usr/bin/env python3
"""
pedestrian_controller.py
────────────────────────
Generic pedestrian controller.  Spawns lightweight SDF pedestrian models into
Gazebo and animates them at UPDATE_HZ using /gazebo/set_model_state.

Configuration — set the ~pedestrians ROS param (list of dicts) to
define pedestrians for any scene:

  name   : Gazebo model name (string, must be unique)
  x      : centre x position  (metres)
  y      : centre y position  (metres)
  ax     : x-oscillation amplitude (0 = fixed in x)
  ay     : y-oscillation amplitude (0 = fixed in y)
  period : seconds per full oscillation
  phase  : initial phase offset (radians for sine, 0..1 cycle fraction for linear)
  motion : 'sine' | 'linear'  default sine
  color  : 'blue' | 'orange'  (visual only)

If ~pedestrians is NOT set, the controller falls back to the original
obstacle-avoidance layout: 10 pedestrians crossing two corridor pinch points,
each moving on a dispersed linear path with a deterministic random seed.
"""

import math
import random
import yaml
import rospy
from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion

UPDATE_HZ = 10

# ── default (obstacle-avoidance) constants ────────────────────────────────────
# Five pedestrians per zone so the robot must avoid MORE THAN four people at
# each of the two crossing points along the route.
_DEFAULT_AMPLITUDE = 1.35
_DEFAULT_PERIOD    = 18.0
_DEFAULT_SEED      = 42
_ZONE_A_X = [-7.8, -8.0, -7.2, -6.4, -6.2]
_ZONE_B_X = [ 1.8,  3.0,  4.0,  6.4,  8.3]
_ZONE_Y = {
    "A": [-4.6, -3.7, -2.6, -4.1, -3.0],
    "B": [-1.6, -0.2,  1.1,  1.4,  2.8],
}
_ZONE_AY = {
    "A": (1.2, _DEFAULT_AMPLITUDE),
    "B": (0.35, 0.70),
}
_ZONE_PHASES = {
    "A": [0.22, 0.47, 0.68, 0.84, 0.36],
    "B": [0.12, 0.39, 0.58, 0.79, 0.28],
}

SPAWN_Z = 0.0
PEDESTRIAN_RADIUS = 0.30
SAFETY_MARGIN = 0.08

_SDF_TEMPLATE = """\
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <kinematic>true</kinematic>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>11.43</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>11.43</iyy><iyz>0</iyz>
          <izz>3.15</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 0.90 0 0 0</pose>
        <geometry>
          <cylinder><radius>0.30</radius><length>1.80</length></cylinder>
        </geometry>
        <surface>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
          <contact>
            <ode>
              <kp>20000</kp>
              <kd>20</kd>
              <max_vel>0.1</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="head">
        <pose>0 0 1.68 0 0 0</pose>
        <geometry>
          <sphere><radius>0.16</radius></sphere>
        </geometry>
        <material>
          <ambient>0.86 0.64 0.46 1</ambient>
          <diffuse>0.86 0.64 0.46 1</diffuse>
        </material>
      </visual>
      <visual name="torso">
        <pose>0 0 1.10 0 0 0</pose>
        <geometry><box><size>0.42 0.24 0.72</size></box></geometry>
        <material>
          <ambient>{r} {g} 0.2 1</ambient>
          <diffuse>{r} {g} 0.2 1</diffuse>
        </material>
      </visual>
      <visual name="left_arm">
        <pose>0 0.27 1.12 0.18 0 0</pose>
        <geometry><cylinder><radius>0.055</radius><length>0.70</length></cylinder></geometry>
        <material><ambient>0.86 0.64 0.46 1</ambient><diffuse>0.86 0.64 0.46 1</diffuse></material>
      </visual>
      <visual name="right_arm">
        <pose>0 -0.27 1.12 -0.18 0 0</pose>
        <geometry><cylinder><radius>0.055</radius><length>0.70</length></cylinder></geometry>
        <material><ambient>0.86 0.64 0.46 1</ambient><diffuse>0.86 0.64 0.46 1</diffuse></material>
      </visual>
      <visual name="left_leg">
        <pose>0 0.10 0.45 0 0.10 0</pose>
        <geometry><cylinder><radius>0.07</radius><length>0.90</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
      </visual>
      <visual name="right_leg">
        <pose>0 -0.10 0.45 0 -0.10 0</pose>
        <geometry><cylinder><radius>0.07</radius><length>0.90</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

_SCENES = {
    "obstacle": {
        "bounds": (-11.7, 11.7, -6.7, 6.7),
        "keepouts": [(-10.0, -5.0, 2.0)],
        "boxes": [
            (-5.2, -1.3, 0.7, 5.4),
            (0.0, 2.3, 0.7, 5.0),
            (5.0, -1.7, 0.7, 5.0),
            (-0.5, -5.2, 3.2, 0.7),
            (5.0, 3.9, 3.0, 0.7),
        ],
    },
    "cafe": {
        "bounds": (-4.2, 4.2, -10.7, 2.7),
        "keepouts": [],
        "boxes": [
            (0.5, -1.6, 0.7, 0.7),
            (2.4, -5.5, 0.7, 0.7),
            (-1.5, -5.5, 0.7, 0.7),
            (2.4, -9.0, 0.7, 0.7),
            (-1.5, -9.0, 0.7, 0.7),
        ],
    },
}


def _color_rgb(color: str):
    return ("0.2", "0.4") if color == "blue" else ("0.9", "0.4")


def _make_sdf(name: str, color: str) -> str:
    r, g = _color_rgb(color)
    return _SDF_TEMPLATE.format(name=name, r=r, g=g)


def _scene_profile():
    scene = rospy.get_param("~scene", "obstacle")
    return _SCENES.get(scene, _SCENES["obstacle"])


def _inside_box(x: float, y: float, box: tuple) -> bool:
    cx, cy, sx, sy = box
    clearance = PEDESTRIAN_RADIUS + SAFETY_MARGIN
    return (abs(x - cx) <= sx / 2.0 + clearance and
            abs(y - cy) <= sy / 2.0 + clearance)


def _is_safe(x: float, y: float, profile: dict) -> bool:
    min_x, max_x, min_y, max_y = profile["bounds"]
    if x < min_x or x > max_x or y < min_y or y > max_y:
        return False
    for cx, cy, radius in profile.get("keepouts", []):
        if math.hypot(x - cx, y - cy) < radius:
            return False
    return not any(_inside_box(x, y, box) for box in profile["boxes"])


def _safe_position_at(p: dict, t: float, profile: dict):
    x, y = _position_at(p, t)
    if _is_safe(x, y, profile):
        return x, y
    return p["_last_safe_x"], p["_last_safe_y"]


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _load_config() -> list:
    """Return a list of pedestrian dicts, from param or hardcoded defaults."""
    param = rospy.get_param("~pedestrians", None)
    if param is not None:
        if isinstance(param, str):
            param = yaml.safe_load(param)
        rospy.loginfo("[pedestrian_ctrl] loaded %d pedestrian(s) from ~pedestrians param",
                      len(param))
        return param

    # ── hardcoded obstacle-avoidance default ──────────────────────────────────
    rng = random.Random(int(rospy.get_param("~random_seed", _DEFAULT_SEED)))
    peds = []
    for zone, xs in (("A", _ZONE_A_X), ("B", _ZONE_B_X)):
        color = "blue" if zone == "A" else "orange"
        for idx, x in enumerate(xs):
            # Mostly vertical crossing, with slight x drift and varied start
            # offsets so the group reads as scattered individuals, not a wall.
            ax = rng.uniform(-0.35, 0.35)
            ay_min, ay_max = _ZONE_AY[zone]
            ay = rng.uniform(ay_min, ay_max)
            y_jitter = rng.uniform(-0.25, 0.25) if zone == "A" else 0.0
            peds.append({
                "name":   f"ped_{zone}{idx + 1}",
                "x":      x + rng.uniform(-0.18, 0.18),
                "y":      _ZONE_Y[zone][idx] + y_jitter,
                "ax":     ax,
                "ay":     ay,
                "period": rng.uniform(16.0, 24.0),
                "phase":  (_ZONE_PHASES[zone][idx] + rng.uniform(-0.03, 0.03)) % 1.0,
                "motion": "linear",
                "color":  color,
            })
    rospy.loginfo("[pedestrian_ctrl] using default obstacle-avoidance layout (%d peds)",
                  len(peds))
    return peds


def _triangle_wave(cycle: float) -> float:
    """Return a linear back-and-forth value in [-1, 1]."""
    cycle = cycle % 1.0
    if cycle < 0.5:
        return -1.0 + 4.0 * cycle
    return 3.0 - 4.0 * cycle


def _position_at(p: dict, t: float):
    period = max(float(p.get("period", _DEFAULT_PERIOD)), 0.1)
    phase = float(p.get("phase", 0.0))
    ax = float(p.get("ax", 0.0))
    ay = float(p.get("ay", 0.0))

    if p.get("motion", "sine") == "linear":
        s = _triangle_wave(t / period + phase)
    else:
        omega = 2.0 * math.pi / period
        s = math.sin(omega * t + phase)

    return float(p["x"]) + ax * s, float(p["y"]) + ay * s


def _spawn_all(spawn_srv, peds: list, profile: dict) -> list:
    spawned = []
    for p in peds:
        x0, y0 = _position_at(p, 0.0)
        if not _is_safe(x0, y0, profile):
            x0, y0 = float(p["x"]), float(p["y"])
        if not _is_safe(x0, y0, profile):
            rospy.logwarn("[pedestrian_ctrl] skipping %s: initial path is inside an obstacle",
                          p["name"])
            continue
        p["_last_safe_x"] = x0
        p["_last_safe_y"] = y0
        p["_last_yaw"] = 0.0
        pose = Pose(
            position=Point(x=x0, y=y0, z=SPAWN_Z),
            orientation=Quaternion(x=0, y=0, z=0, w=1),
        )
        sdf = _make_sdf(p["name"], p.get("color", "blue"))
        try:
            spawn_srv(
                model_name=p["name"],
                model_xml=sdf,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world",
            )
            rospy.loginfo("[pedestrian_ctrl] spawned %s at (%.1f, %.1f)", p["name"], x0, y0)
            spawned.append(p)
        except Exception as exc:
            rospy.logerr("[pedestrian_ctrl] failed to spawn %s: %s", p["name"], exc)
    return spawned


def _animate(peds: list, set_state_srv, profile: dict) -> None:
    rate = rospy.Rate(UPDATE_HZ)
    start_time = rospy.get_time()
    rospy.loginfo("[pedestrian_ctrl] animation loop started (%d pedestrians)", len(peds))

    while not rospy.is_shutdown():
        t = max(0.0, rospy.get_time() - start_time)
        for p in peds:
            prev_x = p["_last_safe_x"]
            prev_y = p["_last_safe_y"]
            x, y = _safe_position_at(p, t, profile)
            if abs(x - prev_x) > 1e-3 or abs(y - prev_y) > 1e-3:
                p["_last_yaw"] = math.atan2(y - prev_y, x - prev_x)
                p["_last_safe_x"] = x
                p["_last_safe_y"] = y
            state = ModelState()
            state.model_name         = p["name"]
            state.pose.position.x    = x
            state.pose.position.y    = y
            state.pose.position.z    = SPAWN_Z
            state.pose.orientation   = _yaw_to_quaternion(p["_last_yaw"])
            state.reference_frame    = "world"

            try:
                set_state_srv(state)
            except Exception as exc:
                rospy.logwarn_throttle(5, "[pedestrian_ctrl] set_model_state failed: %s", exc)

        rate.sleep()


def main():
    rospy.init_node("pedestrian_controller", anonymous=False)

    rospy.loginfo("[pedestrian_ctrl] waiting for Gazebo services…")
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
    rospy.wait_for_service("/gazebo/set_model_state",  timeout=60.0)

    spawn_srv     = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state",  SetModelState)

    rospy.sleep(2.0)

    profile = _scene_profile()
    peds = _load_config()
    spawned = _spawn_all(spawn_srv, peds, profile)
    _animate(spawned, set_state_srv, profile)


if __name__ == "__main__":
    main()
