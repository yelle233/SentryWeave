#!/usr/bin/env python3
"""Render docs/assets/performance.png strictly from JSON files in benchmarks/."""

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONTS = (
    ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 2),
    ("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc", 2),
    ("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc", 0),
    ("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf", 0),
)
PALETTE = {
    "bg": (248, 250, 252),
    "card": (255, 255, 255),
    "border": (222, 228, 236),
    "grid": (236, 240, 245),
    "title": (20, 29, 43),
    "text": (51, 65, 85),
    "muted": (100, 116, 139),
    "blue": (37, 99, 235),
    "sky": (56, 189, 248),
    "amber": (217, 119, 6),
    "teal": (13, 148, 136),
    "indigo": (79, 70, 229),
    "slate": (148, 163, 184),
}
FIELD_LABELS = {
    "rmuc2027": ("RMUC 2027", "RMUC 2027"),
    "rmuc2026": ("RMUC 2026", "RMUC 2026"),
    "demo": ("演示场地", "Demo Field"),
}


def find_font():
    for path, index in FONTS:
        if Path(path).is_file():
            return path, index
    return None


def _load_font(spec, size: int):
    if spec is not None:
        return ImageFont.truetype(spec[0], size, index=spec[1])
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def _num(value, name: str, positive: bool = False) -> float:
    val = float(value)
    if not math.isfinite(val) or (positive and val <= 0) or (not positive and val < 0):
        kind = "positive" if positive else "nonnegative"
        raise ValueError(f"{name} must be finite and {kind}, got {value!r}")
    return val


def _validate(latency: dict, footprint: dict) -> None:
    results = latency.get("results") or {}
    if not results:
        raise ValueError("latency JSON has empty results")
    for field_key, entry in results.items():
        for stat in ("p50", "p95", "mean"):
            _num(entry["step_wall_ms"][stat], f"{field_key}.step_wall_ms.{stat}", positive=True)
        _num(entry["cpu_share_at_2hz"], f"{field_key}.cpu_share_at_2hz")
        _num(entry["tracemalloc_peak_kib"], f"{field_key}.tracemalloc_peak_kib", positive=True)
        _num(entry["obstacles"], f"{field_key}.obstacles")
        _num(entry["graph_corners"], f"{field_key}.graph_corners")
    _num(latency["max_rss_kib"], "max_rss_kib", positive=True)
    for model_key, item in (footprint.get("models") or {}).items():
        _num(item["p_core_us"], f"models.{model_key}.p_core_us", positive=True)
        _num(item["e_core_us"], f"models.{model_key}.e_core_us", positive=True)
        _num(item["artifact_kib"], f"models.{model_key}.artifact_kib", positive=True)
    ros = footprint["ros_decision_node_2hz"]
    for key in ("single_core_cpu_pct_low", "single_core_cpu_pct_high",
                "rss_mib_low", "rss_mib_high"):
        _num(ros[key], f"ros_decision_node_2hz.{key}", positive=True)


def _card(draw: ImageDraw.ImageDraw, box, title: str, subtitle: str, fonts) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = box
    draw.rounded_rectangle(box, radius=20, fill=PALETTE["card"], outline=PALETTE["border"], width=2)
    draw.text((x0 + 32, y0 + 22), title, fill=PALETTE["title"], font=fonts["card"])
    draw.text((x0 + 32, y0 + 62), subtitle, fill=PALETTE["muted"], font=fonts["small"])
    return x0 + 36, y0 + 108, x1 - 36, y1 - 28


def _hbar_pair(draw, area, rows, max_val: float, unit: str, legend, fonts):
    x0, y0, x1, y1 = area
    max_label_w = max(
        (
            max(
                draw.textbbox((0, 0), label, font=fonts["body"])[2],
                draw.textbbox((0, 0), sub, font=fonts["tiny"])[2] if sub else 0,
            )
            for label, sub, *_ in rows
        ),
        default=0,
    )
    label_w = max(368, max_label_w + 24)
    bar_x0, bar_x1 = x0 + label_w, x1 - 130
    span = max(1, bar_x1 - bar_x0)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        gx = round(bar_x0 + span * frac)
        draw.line([(gx, y0 + 8), (gx, y1 - 34)], fill=PALETTE["grid"], width=2)
        draw.text((gx, y1 - 24), f"{max_val * frac:.1f}{unit}", fill=PALETTE["muted"],
                  font=fonts["tiny"], anchor="mt")
    row_h = (y1 - 44 - y0) / max(1, len(rows))
    for idx, (label, sub, v1, c1, v2, c2) in enumerate(rows):
        cy = y0 + (idx + 0.5) * row_h
        draw.text((x0, cy - 12), label, fill=PALETTE["text"], font=fonts["body"], anchor="lm")
        if sub:
            draw.text((x0, cy + 16), sub, fill=PALETTE["muted"], font=fonts["tiny"], anchor="lm")
        if v2 is None:
            bh = min(30, round(row_h * 0.42))
            w1 = max(4, round(span * min(1.0, v1 / max_val)))
            draw.rounded_rectangle([bar_x0, cy - bh // 2, bar_x0 + w1, cy + bh // 2],
                                   radius=7, fill=c1)
            draw.text((bar_x0 + w1 + 10, cy), f"{v1:.2f}{unit}", fill=PALETTE["text"],
                      font=fonts["small"], anchor="lm")
        else:
            bh = min(20, round(row_h * 0.28))
            for offset, val, col in ((-bh - 2, v1, c1), (2, v2, c2)):
                bw = max(4, round(span * min(1.0, val / max_val)))
                draw.rounded_rectangle([bar_x0, cy + offset, bar_x0 + bw, cy + offset + bh],
                                       radius=6, fill=col)
                draw.text((bar_x0 + bw + 10, cy + offset + bh / 2), f"{val:.2f}{unit}",
                          fill=PALETTE["text"], font=fonts["tiny"], anchor="lm")
    lx = bar_x1
    for text, color in reversed(legend):
        bbox = draw.textbbox((0, 0), text, font=fonts["tiny"])
        tw = bbox[2] - bbox[0]
        draw.text((lx, y0 - 16), text, fill=PALETTE["muted"], font=fonts["tiny"], anchor="rm")
        draw.rounded_rectangle([lx - tw - 24, y0 - 24, lx - tw - 8, y0 - 8], radius=4, fill=color)
        lx -= tw + 44


def render_performance_chart(
    latency: dict, footprint: dict, out_path: str | Path, font="auto"
) -> dict:
    """Draw a 2x-supersampled 1200x680 multi-panel PNG from benchmark JSON dicts."""
    _validate(latency, footprint)
    spec = find_font() if font == "auto" else font
    zh = spec is not None
    scale = 2
    width, height = 1200 * scale, 680 * scale
    image = Image.new("RGB", (width, height), PALETTE["bg"])
    draw = ImageDraw.Draw(image)
    fonts = {
        "header": _load_font(spec, 34),
        "sub": _load_font(spec, 21),
        "card": _load_font(spec, 26),
        "body": _load_font(spec, 21),
        "small": _load_font(spec, 19),
        "tiny": _load_font(spec, 17),
    }

    env = latency.get("environment") or {}
    header = ("SentryWeave 决策与机载推理性能实测"
              if zh else "SentryWeave Decision & Onboard Inference Footprint")
    sub = ((f"纯 Python / NumPy 实测（Python {env.get('python', '?')}, NumPy {env.get('numpy', '?')}, "
            f"{latency.get('recorded_on', '')}）· 车端零 PyTorch / CUDA / GPU 依赖")
           if zh else
           (f"Measured on Python {env.get('python', '?')}, NumPy {env.get('numpy', '?')} "
            f"({latency.get('recorded_on', '')}) - Zero PyTorch / CUDA / GPU onboard"))
    draw.text((44, 26), header, fill=PALETTE["title"], font=fonts["header"])
    draw.text((44, 72), sub, fill=PALETTE["muted"], font=fonts["sub"])

    pad, gap, top = 40, 32, 118
    cw = (width - 2 * pad - gap) // 2
    ch = (height - top - pad - gap) // 2
    boxes = [
        (pad, top, pad + cw, top + ch),
        (pad + cw + gap, top, pad + 2 * cw + gap, top + ch),
        (pad, top + ch + gap, pad + cw, top + 2 * ch + gap),
        (pad + cw + gap, top + ch + gap, pad + 2 * cw + gap, top + 2 * ch + gap),
    ]

    results = latency["results"]
    order = [k for k in ("rmuc2027", "rmuc2026", "demo") if k in results]
    # Panel 1: DecisionEngine.step latency (ms)
    a1 = _card(draw, boxes[0],
               "1. 单步决策耗时 DecisionEngine.step" if zh else "1. DecisionEngine.step Latency",
               "含意图推断、可见图路径与动态候选评分（2 Hz 周期 = 500 ms）"
               if zh else "Includes intent, visibility routing & dynamic candidate scoring",
               fonts)
    lat_rows = []
    for key in order:
        r = results[key]
        zh_name, en_name = FIELD_LABELS.get(key, (key, key))
        detail = (f"{int(r['obstacles'])} 障碍/禁区 · {int(r['graph_corners'])} 可见图角点"
                  if zh else f"{int(r['obstacles'])} obstacles · {int(r['graph_corners'])} corners")
        lat_rows.append((zh_name if zh else en_name, detail,
                         float(r["step_wall_ms"]["p50"]), PALETTE["blue"],
                         float(r["step_wall_ms"]["p95"]), PALETTE["amber"]))
    lat_max = max(max(row[2], row[4]) for row in lat_rows) * 1.22
    _hbar_pair(draw, a1, lat_rows, max(5.0, round(lat_max, 1)), " ms",
               [("p50", PALETTE["blue"]), ("p95", PALETTE["amber"])], fonts)

    # Panel 2: Pure NumPy RL inference latency (us)
    a2 = _card(draw, boxes[1],
               "2. 离线 RL 纯 NumPy 单次推理耗时" if zh else "2. Pure NumPy Offline RL Inference",
               "NumpyNavigationPolicy.predict（float32，无 PyTorch 运行时）"
               if zh else "Single forward pass in float32 NumPy without PyTorch",
               fonts)
    mod_rows = []
    for mkey in ("bc_iql", "dt"):
        m = footprint["models"][mkey]
        kib = float(m["artifact_kib"])
        size_str = f"{kib / 1024:.1f} MB JSON" if kib >= 1024 else f"{kib:.0f} KB JSON"
        mod_rows.append((m["label_zh"] if zh else m["label_en"],
                         (f"导出体积 {size_str}" if zh else f"Artifact {size_str}"),
                         float(m["p_core_us"]), PALETTE["teal"],
                         float(m["e_core_us"]), PALETTE["indigo"]))
    mod_max = max(max(row[2], row[4]) for row in mod_rows) * 1.22
    _hbar_pair(draw, a2, mod_rows, max(100.0, round(mod_max, -1)), " us",
               [("P-core", PALETTE["teal"]), ("E-core", PALETTE["indigo"])], fonts)

    # Panel 3: Single-core CPU share at 2 Hz (%)
    a3 = _card(draw, boxes[2],
               "3. 2 Hz 在线运行单核 CPU 占用率" if zh else "3. Single-Core CPU Usage at 2 Hz",
               "占单个 CPU 核心百分比（越低越好）" if zh else "Percentage of 1 CPU core at 2 Hz",
               fonts)
    ros = footprint["ros_decision_node_2hz"]
    cpu_rows = []
    for key in order:
        r = results[key]
        zh_name, en_name = FIELD_LABELS.get(key, (key, key))
        cpu_rows.append(((f"纯核心 ({zh_name})" if zh else f"Core ({en_name})"),
                         "DecisionEngine.step @ 2 Hz",
                         float(r["cpu_share_at_2hz"]) * 100.0, PALETTE["blue"], None, None))
    ros_low, ros_high = float(ros["single_core_cpu_pct_low"]), float(ros["single_core_cpu_pct_high"])
    cpu_rows.append((
        "ROS 2 decision 节点" if zh else "ROS 2 decision node",
        (f"含 DDS 编解码与 JSONL 轨迹（{ros_low:.1f}%–{ros_high:.1f}%）"
         if zh else f"Incl. DDS & JSONL trace ({ros_low:.1f}%-{ros_high:.1f}%)"),
        ros_high, PALETTE["teal"], None, None,
    ))
    cpu_max = max(row[2] for row in cpu_rows) * 1.28
    _hbar_pair(draw, a3, cpu_rows, max(4.0, round(cpu_max, 1)), "%",
               [("CPU / 1 core", PALETTE["blue"])], fonts)

    # Panel 4: Memory & model artifact footprint (MB)
    a4 = _card(draw, boxes[3],
               "4. 内存与模型产物占用" if zh else "4. Memory & Artifact Footprint",
               "Python 峰值分配、ROS 2 节点常驻内存（RSS）与导出权重体积"
               if zh else "Peak traced heap, ROS 2 node RSS & JSON model sizes",
               fonts)
    rmuc27_heap = float(results.get("rmuc2027", next(iter(results.values())))["tracemalloc_peak_kib"]) / 1024.0
    rss_low, rss_high = float(ros["rss_mib_low"]), float(ros["rss_mib_high"])
    dt_mb = float(footprint["models"]["dt"]["artifact_kib"]) / 1024.0
    bc_mb = float(footprint["models"]["bc_iql"]["artifact_kib"]) / 1024.0
    mem_rows = [
        ("ROS 2 decision 常驻 RSS" if zh else "ROS 2 decision RSS",
         f"Jazzy 2 Hz ({rss_low:.0f}{'–' if zh else '-'}{rss_high:.0f} MB)",
         rss_high, PALETTE["indigo"], None, None),
        ("决策核心峰值堆内存" if zh else "Core Peak Python Heap",
         "tracemalloc (RMUC 2027)", rmuc27_heap, PALETTE["blue"], None, None),
        ("DT 导出模型 (actor.json)" if zh else "DT Exported Actor",
         "NumPy JSON", dt_mb, PALETTE["amber"], None, None),
        ("BC / IQL 导出模型" if zh else "BC / IQL Exported Actor",
         "NumPy JSON", bc_mb, PALETTE["teal"], None, None),
    ]
    mem_max = max(row[2] for row in mem_rows) * 1.22
    _hbar_pair(draw, a4, mem_rows, max(80.0, round(mem_max, -1)), " MB",
               [("Peak / Upper", PALETTE["indigo"])], fonts)

    final = image.resize((1200, 680), Image.Resampling.LANCZOS)
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    final.save(out, format="PNG", optimize=True)
    return {"path": str(out), "bytes": out.stat().st_size, "size": [1200, 680],
            "font": spec[0] if spec else None}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latency", default=str(ROOT / "benchmarks/decision-latency.json"))
    parser.add_argument("--footprint", default=str(ROOT / "benchmarks/runtime-footprint.json"))
    parser.add_argument("--out", default=str(ROOT / "docs/assets/performance.png"))
    args = parser.parse_args(argv)
    latency = json.loads(Path(args.latency).read_text(encoding="utf-8"))
    footprint = json.loads(Path(args.footprint).read_text(encoding="utf-8"))
    meta = render_performance_chart(latency, footprint, args.out)
    print(json.dumps(meta, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
