"""Rasterize tactical exclusions into a Nav2 KeepoutFilter mask.

The mask spans the complete navigation map, including areas outside the legal
tactical bounds. It remains active during Nav2 replanning and recovery.
"""

import math
from pathlib import Path
import struct

import numpy as np
import yaml

from .config import Config


def _image_size(path):
    with open(path, "rb") as stream:
        header = stream.read(24)
        if header[:8] == b"\x89PNG\r\n\x1a\n" and header[12:16] == b"IHDR":
            return struct.unpack(">II", header[16:24])
        stream.seek(0)
        tokens = []
        for _ in range(128):
            line = stream.readline(8192)
            if not line:
                break
            tokens.extend(line.split(b"#", 1)[0].split())
            if len(tokens) >= 3:
                if tokens[0] not in (b"P2", b"P5"):
                    raise ValueError("keepout generation supports PNG and PGM maps")
                return int(tokens[1]), int(tokens[2])
    raise ValueError("invalid map image header")


def render_keepout(config: Config, map_yaml, output_directory) -> Path:
    source = Path(map_yaml)
    metadata = yaml.safe_load(source.read_text(encoding="utf-8"))
    resolution = float(metadata["resolution"])
    origin = tuple(float(v) for v in metadata["origin"])
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
            or not all(math.isfinite(v) for v in origin)):
        raise ValueError("invalid navigation map geometry")
    width, height = _image_size(source.parent / metadata["image"])
    if width <= 0 or height <= 0 or width * height > 25_000_000:
        raise ValueError("invalid or excessively large navigation map")
    columns = (np.arange(width)[None, :] + 0.5) * resolution
    rows = (height - np.arange(height)[:, None] - 0.5) * resolution
    tx, ty, angle = origin
    x = tx + math.cos(angle) * columns - math.sin(angle) * rows
    y = ty + math.sin(angle) * columns + math.cos(angle) * rows
    # Cover intersecting raster cells; robot footprint inflation is performed by
    # Nav2's inflation filter after KeepoutFilter, consistently for both costmaps.
    radius = resolution / math.sqrt(2)
    a, b, c, d = config.bounds
    forbidden = ((x <= a + radius) | (x >= c - radius)
                 | (y <= b + radius) | (y >= d - radius))
    for x0, y0, x1, y1 in config.obstacles:
        forbidden |= ((x >= x0 - radius) & (x <= x1 + radius)
                      & (y >= y0 - radius) & (y <= y1 + radius))
    pixels = np.where(forbidden, 0, 254).astype(np.uint8)
    directory = Path(output_directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "keepout.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode() + pixels.tobytes()
    )
    result = directory / "keepout.yaml"
    result.write_text(yaml.safe_dump({
        "image": "keepout.pgm", "mode": "trinary", "resolution": resolution,
        "origin": list(origin), "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25,
    }, sort_keys=False))
    return result
