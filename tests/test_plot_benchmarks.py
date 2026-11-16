"""Tests for the data-driven benchmark chart generator."""

import copy
import json
from pathlib import Path

from PIL import Image
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def sample_payloads():
    latency = json.loads((ROOT / "benchmarks/decision-latency.json").read_text(encoding="utf-8"))
    footprint = json.loads((ROOT / "benchmarks/runtime-footprint.json").read_text(encoding="utf-8"))
    return latency, footprint


def test_chart_renders_all_quadrants_and_tracks_json_data(tmp_path):
    from plot_benchmarks import render_performance_chart

    latency, footprint = sample_payloads()
    out_a = tmp_path / "perf_a.png"
    meta = render_performance_chart(latency, footprint, out_a)
    assert out_a.is_file()
    assert 15_000 <= meta["bytes"] <= 800_000
    with Image.open(out_a) as img:
        assert img.format == "PNG"
        assert img.size == (1200, 680)
        arr_a = np.asarray(img.convert("RGB"))
    # Every quadrant must contain non-trivial chart content (not blank background).
    h, w, _ = arr_a.shape
    for q in (arr_a[:h // 2, :w // 2], arr_a[:h // 2, w // 2:],
              arr_a[h // 2:, :w // 2], arr_a[h // 2:, w // 2:]):
        assert float(q.std()) > 12.0

    # Changing JSON values must change rendered bar heights/labels (proves zero hardcoding).
    mutated = copy.deepcopy(latency)
    mutated["results"]["rmuc2027"]["step_wall_ms"]["p95"] *= 2.2
    mutated_fp = copy.deepcopy(footprint)
    mutated_fp["models"]["dt"]["e_core_us"] *= 1.8
    out_b = tmp_path / "perf_b.png"
    render_performance_chart(mutated, mutated_fp, out_b)
    with Image.open(out_b) as img_b:
        arr_b = np.asarray(img_b.convert("RGB"))
    assert not np.array_equal(arr_a, arr_b)


def test_chart_supports_ascii_fallback_and_rejects_bad_inputs(tmp_path):
    from plot_benchmarks import main, render_performance_chart

    latency, footprint = sample_payloads()
    fallback_png = tmp_path / "fallback.png"
    meta = render_performance_chart(latency, footprint, fallback_png, font=None)
    assert fallback_png.is_file() and meta["font"] is None

    bad = copy.deepcopy(latency)
    bad["results"]["rmuc2027"]["step_wall_ms"]["p50"] = -1.0
    with pytest.raises(ValueError, match="nonnegative|positive"):
        render_performance_chart(bad, footprint, tmp_path / "bad.png")

    cli_out = tmp_path / "cli.png"
    assert main(["--out", str(cli_out)]) == 0
    assert cli_out.is_file()
