"""Bundled RMUC 2027/2026 fields: validity, mirroring and decisions on real geometry."""

from dataclasses import replace

import pytest

from conftest import ROOT

CONFIG = ROOT / "src/sentry_bringup/config"
SEASONS = ("rmuc2027", "rmuc2026")


@pytest.mark.parametrize("season", SEASONS)
@pytest.mark.parametrize("team", ("RED", "BLUE"))
def test_season_field_passes_every_check(season, team):
    from sentry_core.config import load_config
    from sentry_core.field import field_problems

    config = load_config(CONFIG / f"tactics_{season}.yaml", own_team=team)
    assert config.season == season and config.mirror == "auto"
    assert config.mirrored == (team == "BLUE")
    assert field_problems(config, reachability=True) == []
    assert "Arena-1" in (CONFIG / f"field_{season}.yaml").read_text()


@pytest.mark.parametrize("season, base", [("rmuc2027", (2.411, 7.5)), ("rmuc2026", (2.407, 7.5))])
def test_blue_side_is_the_point_reflection_of_red(season, base):
    from sentry_core.config import load_config
    from sentry_core.model import Vec2

    red = load_config(CONFIG / f"tactics_{season}.yaml")
    blue = load_config(CONFIG / f"tactics_{season}.yaml", own_team="BLUE")
    assert red.structure("own_base").position == Vec2(*base)
    image = blue.structure("own_base").position
    assert image.distance(Vec2(28 - base[0], 15 - base[1])) < 1e-9
    assert blue.structure("enemy_base").position.distance(Vec2(*base)) < 0.01
    assert blue.zones[-1].contains(Vec2(26.5, 1.5))


@pytest.mark.parametrize("season", SEASONS)
def test_decisions_on_season_geometry_use_legal_generated_goals(season, world_factory):
    from sentry_core.config import load_config
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from benchmark_decision import scenario

    config = load_config(CONFIG / f"tactics_{season}.yaml")
    arena = TacticalMap(config)
    engine = DecisionEngine(config, session_id="season")
    for world in scenario(config, 6):
        decision = engine.step(world)
        assert decision.task.kind != TaskType.HOLD
        for candidate in decision.candidates:
            if candidate.goal is not None:
                assert arena.reachable(world.self_track.position, candidate.goal), candidate
    sources = {c.goal_id.split(":")[0] for c in decision.candidates if c.goal is not None}
    assert {"intercept", "defense"} <= sources


def test_benchmark_tool_reports_finite_numbers():
    from sentry_core.config import load_config
    from benchmark_decision import measure

    result = measure(load_config(CONFIG / "tactics_rmuc2027.yaml"), 4)
    assert result["steps"] == 4 and result["step_wall_ms"]["p95"] > 0
    assert result["graph_corners"] > 0 and result["tracemalloc_peak_kib"] > 0


def test_season_keepout_mask_marks_enemy_supply(tmp_path):
    import yaml
    from sentry_core.config import load_config
    from sentry_core.keepout import render_keepout

    config = load_config(CONFIG / "tactics_rmuc2027.yaml")
    (tmp_path / "map.pgm").write_bytes(b"P5\n280 150\n255\n" + bytes([254]) * 280 * 150)
    source = tmp_path / "map.yaml"
    source.write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 0.1,
                                      "origin": [0., 0., 0.]}))
    result = render_keepout(replace(config), source, tmp_path / "mask")
    pixels = (result.parent / "keepout.pgm").read_bytes().split(b"\n", 3)[3]

    def cell(x, y):
        return pixels[(149 - int(y / 0.1)) * 280 + int(x / 0.1)]

    assert cell(26.0, 1.5) == 0      # enemy supply (rule keep-out)
    assert cell(1.5, 13.5) == 254    # own supply stays drivable
    assert cell(14.0, 7.5) == 0      # rune pedestal (physical obstacle)
