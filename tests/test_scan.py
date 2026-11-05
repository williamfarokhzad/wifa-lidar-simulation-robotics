import json
import math
from pathlib import Path
import random
import subprocess
import sys
import unittest

from lidar_scan import scan

ROOT = Path(__file__).resolve().parents[1]


def sample(name):
    return json.loads((ROOT / "examples" / f"{name}.json").read_text())


def edge_reference(origin, angle, box):
    """Independent oracle: intersect the ray with the four rectangle edges."""
    px, py = origin
    dx, dy = math.cos(angle), math.sin(angle)
    x0, y0, x1, y1 = box
    contacts = []
    for x in (x0, x1):
        t = (x - px) / dx
        if t >= 0 and y0 <= py + t * dy <= y1:
            contacts.append(t)
    for y in (y0, y1):
        t = (y - py) / dy
        if t >= 0 and x0 <= px + t * dx <= x1:
            contacts.append(t)
    return min(contacts, default=math.inf)


class ScanTests(unittest.TestCase):
    def test_aisle_and_occlusion(self):
        rows = scan(sample("aisle"))
        self.assertEqual([r["angle_deg"] for r in rows], [-45, 0, 45])
        self.assertEqual([r["status"] for r in rows], ["no_return", "hit", "hit"])
        self.assertEqual(rows[1]["range_m"], 2)
        self.assertAlmostEqual(rows[2]["range_m"], math.sqrt(2))

    def test_positive_yaw_changes_world_bearings(self):
        before, after = scan(sample("aisle")), scan(sample("quarter-turn"))
        self.assertAlmostEqual(before[2]["range_m"], after[0]["range_m"])
        self.assertEqual([r["status"] for r in after], ["hit", "no_return", "no_return"])

    def test_blind_zone_does_not_reveal_hidden_box(self):
        scene = sample("blind-zone")
        self.assertEqual([r["status"] for r in scan(scene)],
                         ["too_close", "hit", "no_return", "no_return"])
        self.assertEqual(scan(scene)[1]["range_m"], 5)
        scene["boxes"].pop(1)
        self.assertEqual(scan(scene)[0]["range_m"], 2)

    def test_inclusive_minimum_and_exact_corner(self):
        scene = {"pose": [0, 0, 0], "angles": [0, 90], "count": 1,
                 "limits": [2, 5], "boxes": [[2, 0, 4, 1]]}
        self.assertEqual(scan(scene)[0]["range_m"], 2)

    def test_translation_preserves_sensor_ranges(self):
        scene = sample("aisle")
        baseline = scan(scene)
        scene["pose"][:2] = [5, -2]
        scene["boxes"] = [[a+5, b-2, c+5, d-2] for a, b, c, d in scene["boxes"]]
        for a, b in zip(baseline, scan(scene)):
            self.assertEqual(a["status"], b["status"])
            if a["range_m"] is not None:
                self.assertAlmostEqual(a["range_m"], b["range_m"])

    def test_seeded_against_four_edges(self):
        rng = random.Random(741)
        hits, misses = 0, 0
        for _ in range(250):
            origin = (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5))
            a = rng.uniform(-179, 179)
            boxes = []
            for x, y in [(1, 1), (1, -2), (-3, 1), (-2, -3)]:
                boxes.append([x, y, x+rng.uniform(0.3, 1.7), y+rng.uniform(0.3, 1.7)])
            want = min(edge_reference(origin, math.radians(a), b) for b in boxes)
            actual = scan({"pose": [*origin, 0], "angles": [a, 1], "count": 1,
                           "limits": [0, 10], "boxes": boxes})[0]
            if math.isinf(want):
                self.assertIsNone(actual["range_m"])
                misses += 1
            else:
                self.assertAlmostEqual(want, actual["range_m"])
                hits += 1
        self.assertGreater(hits, 30)
        self.assertGreater(misses, 30)

    def test_rejects_invalid_inputs(self):
        for key, value in [("count", True), ("angles", [0, 180]),
                           ("limits", [math.nan, 5]), ("boxes", [[0, 0, 1, 1]]),
                           ("boxes", [[1, 1, 1, 2]])]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                scan({**sample("aisle"), key: value})

    def test_cli_emits_strict_json_and_fails_on_missing_input(self):
        args = [sys.executable, "lidar_scan.py"]
        result = subprocess.run(args + ["examples/blind-zone.json"], cwd=ROOT,
                                check=True, capture_output=True, text=True)
        self.assertEqual(json.loads(result.stdout), scan(sample("blind-zone")))
        self.assertNotIn("Infinity", result.stdout)
        missing = subprocess.run(args + ["nope.json"], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(missing.returncode, 2)
        self.assertIn("nope.json", missing.stderr)


if __name__ == "__main__":
    unittest.main()
