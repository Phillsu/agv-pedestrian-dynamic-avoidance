#!/usr/bin/env python3
"""
Generate map.pgm for the obstacle_avoidance world.

World geometry (matches obstacle_avoidance.world):
  x : -12.5 … +12.5   (wall outer edges)
  y :  -7.5 … +7.5

Map covers a margin around the world so walls land inside the image:
  origin : (-13, -8, 0)
  size   : 26 m × 16 m  →  520 × 320 px  at 0.05 m/px

Pixel convention (ROS map_server):
  row 0  = y_max   (top of PGM = north)
  col 0  = x_min   (left of PGM = west)

Walls (1 m thick in the world, 20 px in the map):
  North : rows   0 – 19   (y ∈ [7.0, 8.0])
  South : rows 300 – 319  (y ∈ [-8.0, -7.0])
  West  : cols   0 – 19   (x ∈ [-13.0, -12.0])
  East  : cols 500 – 519  (x ∈ [12.0, 13.0])

Static obstacles match worlds/obstacle_avoidance.world.  They create an S-like
route from the lower-left start to the upper-right goal while leaving enough
clearance for inflation around the AGV.
"""
import os
import struct

# ── map parameters ────────────────────────────────────────────────────────────
RESOLUTION  = 0.05          # metres per pixel
ORIGIN_X    = -13.0
ORIGIN_Y    = -8.0
WIDTH       = 520           # pixels  (26 m / 0.05)
HEIGHT      = 320           # pixels  (16 m / 0.05)
WALL_PX     = 20            # 1 m wall thickness in pixels

FREE        = 254
OCCUPIED    = 0

# (name, centre_x, centre_y, size_x, size_y) in metres.
STATIC_OBSTACLES = [
    ("shelf_west",  -5.2, -1.3, 0.7, 5.4),
    ("shelf_mid",    0.0,  2.3, 0.7, 5.0),
    ("shelf_east",   5.0, -1.7, 0.7, 5.0),
    ("crate_south", -0.5, -5.2, 3.2, 0.7),
    ("crate_north",  5.0,  3.9, 3.0, 0.7),
]
# ──────────────────────────────────────────────────────────────────────────────


def row_for_y(y):
    """World y → PGM row (row 0 = y_max = ORIGIN_Y + HEIGHT*RESOLUTION)."""
    return HEIGHT - 1 - int((y - ORIGIN_Y) / RESOLUTION)


def col_for_x(x):
    """World x → PGM column."""
    return int((x - ORIGIN_X) / RESOLUTION)


def fill_rect(data, cx, cy, sx, sy):
    """Mark an axis-aligned world-space rectangle as occupied."""
    min_col = max(0, col_for_x(cx - sx / 2.0))
    max_col = min(WIDTH - 1, col_for_x(cx + sx / 2.0))
    min_row = max(0, row_for_y(cy + sy / 2.0))
    max_row = min(HEIGHT - 1, row_for_y(cy - sy / 2.0))

    for row in range(min_row, max_row + 1):
        for col in range(min_col, max_col + 1):
            data[row * WIDTH + col] = OCCUPIED


def build_map():
    data = bytearray([FREE] * (WIDTH * HEIGHT))

    # outer walls
    for row in range(HEIGHT):
        for col in range(WIDTH):
            if (row < WALL_PX or row >= HEIGHT - WALL_PX or
                    col < WALL_PX or col >= WIDTH - WALL_PX):
                data[row * WIDTH + col] = OCCUPIED

    for _name, cx, cy, sx, sy in STATIC_OBSTACLES:
        fill_rect(data, cx, cy, sx, sy)

    return data


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path   = os.path.join(script_dir, "map.pgm")
    yaml_path  = os.path.join(script_dir, "map.yaml")

    data = build_map()

    # write binary PGM (P5)
    with open(pgm_path, "wb") as f:
        header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
        f.write(header.encode("ascii"))
        f.write(bytes(data))

    # write YAML sidecar (in case it doesn't exist yet)
    if not os.path.exists(yaml_path):
        with open(yaml_path, "w") as f:
            f.write(
                f"image: map.pgm\n"
                f"resolution: {RESOLUTION}\n"
                f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
        print(f"Wrote {yaml_path}")

    occupied = sum(1 for b in data if b == OCCUPIED)
    free     = len(data) - occupied
    print(f"Wrote {pgm_path}")
    print(f"  {WIDTH}×{HEIGHT} px  |  {WIDTH*RESOLUTION:.0f}×{HEIGHT*RESOLUTION:.0f} m  "
          f"|  {free} free  {occupied} occupied px")


if __name__ == "__main__":
    main()
