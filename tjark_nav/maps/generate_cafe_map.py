#!/usr/bin/env python3
"""
Generate cafe_map.pgm / cafe_map.yaml to match cafe_world.world.

World geometry (cafe_world.world):
  Interior free space : x ∈ [-4.5,  4.5],  y ∈ [-11.0,  3.0]
  Wall inner edges    : West x=-4.5, East x=+4.5, North y=3.0, South y=-11.0

Map adds 1.5 m margin on all sides:
  origin : (-6, -13)
  size   : 12 m × 17 m  →  240 × 340 px  at 0.05 m/px

Tables (static obstacles, 0.7×0.7 m):
  (0.5, -1.6)  (2.4, -5.5)  (-1.5, -5.5)  (2.4, -9.0)  (-1.5, -9.0)
"""
import os

RESOLUTION = 0.05
ORIGIN_X   = -6.0
ORIGIN_Y   = -13.0
WIDTH      = 240
HEIGHT     = 340

FREE     = 254
OCCUPIED = 0

# wall inner edges (world coordinates)
WALL_W = -4.5
WALL_E =  4.5
WALL_N =  3.0
WALL_S = -11.0

TABLES = [
    (0.5,  -1.6),
    (2.4,  -5.5),
    (-1.5, -5.5),
    (2.4,  -9.0),
    (-1.5, -9.0),
]
TABLE_HALF = 0.35  # half of 0.7 m table


def world_to_px(x, y):
    col = int((x - ORIGIN_X) / RESOLUTION)
    row = HEIGHT - 1 - int((y - ORIGIN_Y) / RESOLUTION)
    return col, row


def build_map():
    data = bytearray([FREE] * (WIDTH * HEIGHT))

    for row in range(HEIGHT):
        for col in range(WIDTH):
            # world coordinate of this pixel centre
            wx = ORIGIN_X + (col + 0.5) * RESOLUTION
            wy = ORIGIN_Y + (HEIGHT - row - 0.5) * RESOLUTION

            # outer walls
            if wx < WALL_W or wx > WALL_E or wy < WALL_S or wy > WALL_N:
                data[row * WIDTH + col] = OCCUPIED
                continue

            # tables
            for tx, ty in TABLES:
                if abs(wx - tx) <= TABLE_HALF and abs(wy - ty) <= TABLE_HALF:
                    data[row * WIDTH + col] = OCCUPIED
                    break

    return data


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path   = os.path.join(script_dir, "cafe_map.pgm")
    yaml_path  = os.path.join(script_dir, "cafe_map.yaml")

    data = build_map()

    with open(pgm_path, "wb") as f:
        header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
        f.write(header.encode("ascii"))
        f.write(bytes(data))

    with open(yaml_path, "w") as f:
        f.write(
            f"image: cafe_map.pgm\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    occupied = sum(1 for b in data if b == OCCUPIED)
    free = len(data) - occupied
    print(f"Wrote {pgm_path}  ({WIDTH}×{HEIGHT} px | {WIDTH*RESOLUTION:.0f}×{HEIGHT*RESOLUTION:.0f} m | {free} free {occupied} occupied)")
    print(f"Wrote {yaml_path}")


if __name__ == "__main__":
    main()
