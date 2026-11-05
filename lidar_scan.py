"""Cast a stationary, ideal 2D LiDAR scan against solid rectangles."""
import argparse
import json
import math
from pathlib import Path

Box = tuple[float, float, float, float]
Point = tuple[float, float]


def first_contact(origin: Point, direction: Point, box: Box) -> float | None:
    """Return metres along a unit ray to a closed box, or None for a miss."""
    enter, leave = 0.0, math.inf
    for p, v, low, high in zip(origin, direction, box[:2], box[2:]):
        if v == 0:
            if not low <= p <= high:
                return None
            continue
        a, b = sorted(((low - p) / v, (high - p) / v))
        enter, leave = max(enter, a), min(leave, b)
        if enter > leave:
            return None
    return enter


def number(value, label):
    if type(value) not in (int, float) or not -1e6 <= value <= 1e6:
        raise ValueError(f"{label}: expected a finite number in ±1,000,000")
    return float(value)


def vector(value, n, label):
    if not isinstance(value, list) or len(value) != n:
        raise ValueError(f"{label}: expected an array of {n} numbers")
    return tuple(number(v, label) for v in value)


def scan(scene: dict) -> list[dict]:
    if not isinstance(scene, dict):
        raise ValueError("scene: expected a JSON object")
    x, y, yaw = vector(scene["pose"], 3, "pose")
    start, step = vector(scene["angles"], 2, "angles")
    near, far = vector(scene["limits"], 2, "limits")
    count = scene["count"]
    if type(count) is not int or not 1 <= count <= 1441:
        raise ValueError("count: expected an integer from 1 to 1441")
    if not 0 < step <= 360 or (count - 1) * step >= 360:
        raise ValueError("angles: use a positive step and a scan spanning less than 360 degrees")
    if not 0 <= near < far:
        raise ValueError("limits: expected 0 <= minimum < maximum")
    raw_boxes = scene["boxes"]
    if not isinstance(raw_boxes, list) or len(raw_boxes) > 500:
        raise ValueError("boxes: expected an array of at most 500 rectangles")
    boxes = [vector(b, 4, "box") for b in raw_boxes]
    for x0, y0, x1, y1 in boxes:
        if x0 >= x1 or y0 >= y1:
            raise ValueError("box: minimum corner must be below maximum on both axes")
        if x0 <= x <= x1 and y0 <= y <= y1:
            raise ValueError("pose: scanner must start outside every closed box")
    rows = []
    for i in range(count):
        angle = start + i * step
        bearing = (yaw + angle) % 360
        cardinal = {0: (1.0, 0.0), 90: (0.0, 1.0),
                    180: (-1.0, 0.0), 270: (0.0, -1.0)}
        if bearing in cardinal:
            direction = cardinal[bearing]
        else:
            rad = math.radians(bearing)
            direction = math.cos(rad), math.sin(rad)
        contacts = (first_contact((x, y), direction, box) for box in boxes)
        distance = min((t for t in contacts if t is not None), default=math.inf)
        if distance < near:
            status, measured = "too_close", None
        elif distance > far:
            status, measured = "no_return", None
        else:
            status, measured = "hit", distance
        rows.append({"angle_deg": angle, "status": status, "range_m": measured})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path, help="scene JSON file")
    args = parser.parse_args()
    try:
        rows = scan(json.loads(args.scene.read_text(encoding="utf-8")))
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(json.dumps(rows, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
