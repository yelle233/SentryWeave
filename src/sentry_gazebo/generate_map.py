"""Generate an uninflated Nav2 map from the same geometry used by tactical tests."""
from pathlib import Path
import sys

import yaml


def generate(config_path, directory):
    raw = yaml.safe_load(Path(config_path).read_text())
    a, b, c, d = raw["bounds"]
    resolution = 0.1
    width, height = round((c - a) / resolution), round((d - b) / resolution)
    pixels = bytearray()
    for row in range(height):
        y = d - (row + 0.5) * resolution
        for column in range(width):
            x = a + (column + 0.5) * resolution
            wall = min(x - a, c - x, y - b, d - y) < 0.15
            occupied = wall or any(x0 <= x <= x1 and y0 <= y <= y1
                                   for x0, y0, x1, y1 in raw["obstacles"])
            pixels.append(0 if occupied else 254)
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    (output / "demo.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    (output / "demo.yaml").write_text(yaml.safe_dump({
        "image": "demo.pgm", "mode": "trinary", "resolution": resolution,
        "origin": [a, b, 0.0], "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25,
    }, sort_keys=False))


if __name__ == "__main__":
    generate(*sys.argv[1:])
