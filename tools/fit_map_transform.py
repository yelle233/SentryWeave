#!/usr/bin/env python3
"""Fit a planar world->map transform from surveyed corresponding points, in meters."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def fit_transform(anchors, max_rmse=0.2):
    source = np.asarray([p["world"] for p in anchors], dtype=float)
    target = np.asarray([p["map"] for p in anchors], dtype=float)
    if (source.ndim != 2 or source.shape[1:] != (2,) or target.shape != source.shape
            or len(source) < 2 or not np.isfinite(source).all() or not np.isfinite(target).all()
            or not math.isfinite(max_rmse) or max_rmse <= 0):
        raise ValueError("at least two finite world/map point pairs and positive tolerance required")
    source_center, target_center = source.mean(axis=0), target.mean(axis=0)
    s, t = source - source_center, target - target_center
    if np.linalg.norm(s) < 1e-6 or np.linalg.norm(t) < 1e-6:
        raise ValueError("survey points must be distinct")
    u, _, vt = np.linalg.svd(s.T @ t)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    errors = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    if rmse > max_rmse:
        raise ValueError(f"survey residual {rmse:.3f} m exceeds {max_rmse:.3f} m; check units/anchors")
    angle = math.atan2(rotation[1, 0], rotation[0, 0])
    return {"world_to_map": [float(translation[0]), float(translation[1]), angle],
            "rmse_m": rmse, "point_errors_m": errors.tolist(), "points": len(anchors)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("anchors", help='JSON array: [{"world":[x,y], "map":[x,y]}, ...]')
    parser.add_argument("--max-rmse", type=float, default=0.2)
    args = parser.parse_args()
    value = fit_transform(json.loads(Path(args.anchors).read_text()), args.max_rmse)
    print(json.dumps(value, indent=2))
    print("Launch argument: world_to_map:='" + json.dumps(value["world_to_map"]) + "'")


if __name__ == "__main__":
    main()
