"""Field semantic files: schema, mirroring and the validation command."""

import json
from pathlib import Path

import pytest
import yaml

from conftest import ROOT

CONFIG = ROOT / "src/sentry_bringup/config"


def symmetric_field(**changes):
    field = {
        "field_version": 1, "season": "unit", "mirror": "auto", "bounds": [0, 0, 20, 12],
        "obstacles": [{"id": "west", "rect": [5, 2, 6, 4]}, {"id": "east", "rect": [14, 8, 15, 10]}],
        "keepouts": [{"id": "enemy_supply", "polygon": [[17, 9], [20, 9], [20, 12], [17, 12]]}],
        "structures": {"own_base": {"xy": [2, 6]}, "own_outpost": {"xy": [6, 9]},
                       "enemy_base": {"xy": [18, 6], "yaw": 3.14159265},
                       "enemy_outpost": {"xy": [14, 3]}},
        "supply": {"id": "own_supply", "polygon": [[0, 0], [3, 0], [3, 3], [0, 3]]},
        "zones": [
            {"id": "own_base", "structure": "own_base", "center": [2, 6], "radius": 3,
             "intent": "DEFEND_BASE", "importance": 1.0},
            {"id": "enemy_outpost", "structure": "enemy_outpost", "center": [14, 3], "radius": 3,
             "intent": "ATTACK_OUTPOST", "importance": 0.9},
            {"id": "middle", "polygon": [[8, 5], [12, 5], [12, 7], [8, 7]],
             "intent": "CONTEST_AREA", "importance": 0.6},
        ],
        "points": [{"id": "watch", "zone": "own_base", "xy": [4, 7], "yaw": 0.0,
                    "tasks": ["DEFEND"]}],
    }
    field.update(changes)
    return field


def write_tactics(tmp_path: Path, field: dict) -> Path:
    tactics = yaml.safe_load((CONFIG / "tactics.yaml").read_text())
    tactics["field"] = "field.yaml"
    (tmp_path / "field.yaml").write_text(yaml.safe_dump(field))
    path = tmp_path / "tactics.yaml"
    path.write_text(yaml.safe_dump(tactics))
    return path


def test_demo_tactics_reference_the_demo_field(config):
    from sentry_core.config import Area

    assert config.season == "demo" and config.mirror == "none" and not config.mirrored
    assert [z.name for z in config.zones] == ["own_base", "enemy_outpost", "center", "recovery"]
    assert len(config.points) == 8 and isinstance(config.obstacles[0], Area)
    assert {s.name for s in config.structures} == {
        "own_base", "own_outpost", "enemy_base", "enemy_outpost"}
    assert config.zones[0].structure == "own_base"


def test_schema_one_files_still_load(tmp_path, world_factory):
    from sentry_core.config import load_config
    from sentry_core.policy import DecisionEngine

    legacy = {
        "schema_version": 1, "site": "legacy", "frame": "map", "self_id": 7,
        "bounds": [0, 0, 20, 12], "robot_radius": 0.32, "obstacles": [[9, 4.5, 11, 7.5]],
        "weights": [0.22, 0.24, 0.28, 0.12, 0.10, 0.04],
        "zones": [
            {"id": "own_base", "center": [2, 6], "radius": 3, "intent": "DEFEND_BASE",
             "importance": 1.0},
            {"id": "enemy_outpost", "center": [18, 6], "radius": 3.5,
             "intent": "ATTACK_OUTPOST", "importance": 0.9},
            {"id": "center", "center": [10, 2], "radius": 2.5, "intent": "CONTEST_AREA",
             "importance": 0.65},
            {"id": "recovery", "center": [2, 2], "radius": 1.4, "intent": "RETREAT",
             "importance": 0.7},
        ],
        "points": [{"id": "defense_south", "zone": "own_base", "xy": [4, 4],
                    "tasks": ["DEFEND"]}],
    }
    path = tmp_path / "legacy.yaml"
    path.write_text(yaml.safe_dump(legacy))
    config = load_config(path)
    assert config.structures == () and config.obstacles == ((9, 4.5, 11, 7.5),)
    assert DecisionEngine(config).step(world_factory()).task.goal is not None


def test_polygon_zones_and_supply_become_zones(tmp_path):
    from sentry_core.config import load_config
    from sentry_core.model import IntentType, Vec2

    config = load_config(write_tactics(tmp_path, symmetric_field()))
    middle = next(z for z in config.zones if z.name == "middle")
    assert middle.center == Vec2(10, 6) and middle.contains(Vec2(11.9, 6.9))
    assert not middle.contains(Vec2(12.1, 6))
    supply = config.zones[-1]
    assert supply.name == "own_supply" and supply.intent == IntentType.RETREAT


def test_mirror_auto_reflects_the_field_for_blue(tmp_path):
    from sentry_core.config import load_config
    from sentry_core.model import Vec2
    from sentry_core.shapes import mirror_point

    path = write_tactics(tmp_path, symmetric_field())
    red, blue = load_config(path), load_config(path, own_team="BLUE")
    assert not red.mirrored and blue.mirrored and blue.own_team == "BLUE"
    assert blue.structure("own_base").position == Vec2(18, 6)
    assert blue.structure("enemy_base").position == Vec2(2, 6)
    assert blue.points[0].position == mirror_point(red.points[0].position, red.bounds)
    assert abs(abs(blue.points[0].yaw) - 3.14159265) < 1e-6
    assert all(blue.keepouts[0].polygon[i] == mirror_point(p, red.bounds)
               for i, p in enumerate(red.keepouts[0].polygon))
    assert {a.name for a in blue.obstacles} == {"west", "east"}


def test_goal_prior_mirrors_features_for_a_mirrored_field(tmp_path):
    from sentry_core.config import load_config
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    path = write_tactics(tmp_path, symmetric_field())
    actor = actor_file(tmp_path)
    assert not GoalPrior.load(actor, load_config(path)).mirror
    assert GoalPrior.load(actor, load_config(path, own_team="BLUE")).mirror
    assert not GoalPrior.load(actor, load_config(path, own_team="BLUE"), mirror=False).mirror


def test_field_problems_are_reported(tmp_path):
    from sentry_core.config import load_config

    lopsided = symmetric_field(obstacles=[{"id": "west", "rect": [5, 2, 6, 4]}])
    with pytest.raises(ValueError, match="mirror image"):
        load_config(write_tactics(tmp_path, lopsided))
    moved = symmetric_field()
    moved["structures"]["enemy_base"] = {"xy": [17, 6]}
    with pytest.raises(ValueError, match="enemy_base is not the mirror image"):
        load_config(write_tactics(tmp_path, moved))
    missing = symmetric_field()
    del missing["structures"]["own_outpost"]
    with pytest.raises(ValueError, match="missing structure own_outpost"):
        load_config(write_tactics(tmp_path, missing))
    blocked = symmetric_field(points=[{"id": "bad", "zone": "own_base", "xy": [5.5, 3],
                                       "tasks": ["DEFEND"]}])
    with pytest.raises(ValueError, match="lies in an obstacle"):
        load_config(write_tactics(tmp_path, blocked))
    with pytest.raises(ValueError, match="unknown field keys"):
        load_config(write_tactics(tmp_path, symmetric_field(goals=[])))


def test_field_check_command(tmp_path, capsys):
    from sentry_core.fieldcheck import main

    assert main([str(CONFIG / "tactics.yaml")]) == 0
    assert json.loads(capsys.readouterr().out)["ok"]
    field = tmp_path / "field.yaml"
    field.write_text(yaml.safe_dump(symmetric_field(obstacles=[{"id": "west",
                                                               "rect": [5, 2, 6, 4]}])))
    assert main([str(field), "--own-team", "BLUE"]) == 1
    report = json.loads(capsys.readouterr().out)
    assert any("mirror image" in problem for problem in report["problems"])


def test_field_check_reports_malformed_and_missing_files(tmp_path):
    from sentry_core.fieldcheck import check_file

    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(symmetric_field(structures=["own_base"])))
    broken = tmp_path / "broken.yaml"
    broken.write_text("field_version: [1\n")
    for path in (bad, broken, tmp_path / "missing.yaml"):
        report = check_file(path)
        assert not report["ok"] and len(report["problems"]) == 1


def test_closed_rings_and_repeated_vertices_are_dropped(tmp_path):
    from sentry_core.config import load_config
    from sentry_core.model import Vec2

    ring = [[5, 2], [6, 2], [6, 2], [6, 4], [5, 4], [5, 2]]
    field = symmetric_field(obstacles=[{"id": "west", "polygon": ring},
                                       {"id": "east", "rect": [14, 8, 15, 10]}])
    field["keepouts"][0]["polygon"].append([17, 9])
    field["supply"]["polygon"].append([0, 0])
    field["zones"][2]["polygon"] = [[8, 5], [12, 5], [12, 7], [12, 7], [8, 7], [8, 5]]
    path = write_tactics(tmp_path, field)
    for team in ("RED", "BLUE"):
        config = load_config(path, own_team=team)
        assert [len(a.polygon) for a in (*config.obstacles, *config.keepouts)] == [4, 4, 4]
        assert [len(z.polygon) for z in config.zones if z.polygon] == [4, 4]
    assert load_config(path).obstacles[0].polygon[0] == Vec2(5, 2)


def test_zero_length_edges_are_rejected_with_a_clear_error(config):
    from dataclasses import replace
    from sentry_core.config import Area, validate_config
    from sentry_core.model import Vec2
    from sentry_core.shapes import is_convex

    a, b, c = Vec2(14, 1), Vec2(18, 1), Vec2(16, 4)
    assert is_convex((a, b, c))
    for polygon in ((a, b, b, c), (a, b, c, a)):
        assert not is_convex(polygon)
        with pytest.raises(ValueError, match="distinct vertices"):
            validate_config(replace(config, keepouts=(Area("repeated", polygon),)))
        with pytest.raises(ValueError, match="distinct vertices"):
            validate_config(replace(config, obstacles=(Area("repeated", polygon),)))


def test_self_id_follows_own_team_for_schema_two_and_the_field_check(tmp_path):
    from sentry_core.config import load_config
    from sentry_core.fieldcheck import check_file

    path = write_tactics(tmp_path, symmetric_field())
    raw = yaml.safe_load(path.read_text())
    assert "self_id" not in raw
    assert [load_config(path, own_team=team).self_id for team in ("RED", "BLUE")] == [7, 107]
    assert [check_file(tmp_path / "field.yaml", team)["self_id"]
            for team in ("RED", "BLUE")] == [7, 107]
    assert check_file(path, "BLUE")["self_id"] == 107
    path.write_text(yaml.safe_dump({**raw, "self_id": 7}))
    assert load_config(path, own_team="BLUE").self_id == 7
