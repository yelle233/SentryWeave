"""Typed game/robot/structure inputs: validity, expiry, precedence and replay."""

from dataclasses import replace
import json


def game(now, phase=None, own_team="RED", remaining=300.0, supply=True):
    from sentry_core.model import GameState, Phase

    return GameState(now, "rmuc2027", own_team, Phase.RUNNING if phase is None else phase,
                     remaining, supply)


def test_world_without_typed_inputs_keeps_working(config):
    from sentry_core.model import Observation, Phase, Referee, Team, TaskType, Vec2, World
    from sentry_core.policy import DecisionEngine
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    model.observe(Observation(7, Team.ALLY, Vec2(3, 6), 10.0), 10.0)
    track = model.snapshot(10.0).self_track
    world = World(1, 10.0, track, referee=Referee(10.0, Phase.RUNNING, 0.9, 0.9, True, True, True))
    assert world.game is None and world.robots == {} and world.structures == {}
    assert DecisionEngine(config).step(world).task.kind != TaskType.HOLD


def test_game_status_alone_enables_decisions(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    world = replace(world_factory(), referee=None, game=game(10.0))
    assert DecisionEngine(config).step(world).task.kind == TaskType.DEFEND


def test_typed_interface_wins_over_legacy_referee(config, world_factory):
    from sentry_core.model import Phase, RobotStatus, Team, TaskType
    from sentry_core.policy import DecisionEngine

    legacy_ready = world_factory(phase=Phase.READY)
    assert DecisionEngine(config).step(legacy_ready).task.kind == TaskType.HOLD
    running = replace(legacy_ready, game=game(10.0))
    assert DecisionEngine(config).step(running).task.kind == TaskType.DEFEND
    finished = replace(world_factory(), game=game(10.0, phase=Phase.FINISHED))
    assert DecisionEngine(config).step(finished).task.kind == TaskType.HOLD
    low = RobotStatus(7, Team.ALLY, 10.0, hp=40.0, max_hp=400.0, ammo=0.9)
    hurt = replace(world_factory(hp=0.9, supply=True), robots={7: low})
    assert DecisionEngine(config).step(hurt).task.kind == TaskType.RECOVER


def test_expired_typed_inputs_are_unknown(config, world_factory):
    from sentry_core.inputs import view_inputs
    from sentry_core.model import Phase, RobotStatus, StructureState, Team, TaskType
    from sentry_core.policy import DecisionEngine

    world = replace(world_factory(), referee=None, game=game(10.0 - config.game_timeout - 0.1))
    assert DecisionEngine(config).step(world).task.kind == TaskType.HOLD
    old = RobotStatus(7, Team.ALLY, 10.0 - config.robot_status_timeout - 0.1, hp=1.0, max_hp=400.)
    gone = StructureState("enemy_outpost", 10.0 - config.structure_timeout - 0.1, alive=False)
    view = view_inputs(replace(world_factory(hp=0.9), robots={7: old},
                               structures={"enemy_outpost": gone}), config)
    assert view.self_hp == 0.9  # Falls back to the fresh legacy value.
    assert view.structures["enemy_outpost"] is True
    assert view.phase == Phase.RUNNING


def test_team_mismatch_holds_with_reason(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    world = replace(world_factory(), game=game(10.0, own_team="BLUE"))
    decision = DecisionEngine(config).step(world)
    assert decision.task.kind == TaskType.HOLD
    assert decision.task.reason == "TEAM_MISMATCH"


def test_unknown_is_neither_zero_nor_full(config, world_factory):
    from sentry_core.inputs import view_inputs
    from sentry_core.model import RobotStatus, StructureState, Team

    world = replace(world_factory(hp=None, enemy_outpost=None), robots={
        7: RobotStatus(7, Team.ALLY, 10.0, alive=True),
        3: RobotStatus(3, Team.ENEMY, 10.0, hp=0.0, max_hp=200.0),
    }, structures={"own_base": StructureState("own_base", 10.0)})
    view = view_inputs(world, config)
    assert view.self_hp is None and view.self_alive is True
    assert view.robot(Team.ENEMY, 3).known_alive is False
    assert view.robot(Team.ALLY, 3) is None
    assert view.structures["own_base"] is None
    assert view.structures["enemy_outpost"] is None


def test_world_model_merges_fields_from_several_sources(config):
    from sentry_core.model import SOURCE_RADAR, SOURCE_REFEREE, RobotStatus, Team
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    assert model.update_robot(RobotStatus(103, Team.ENEMY, 10.0, hp=150.0, max_hp=200.0,
                                          source=SOURCE_REFEREE), 10.0)
    assert model.update_robot(RobotStatus(103, Team.ENEMY, 10.4, alive=True,
                                          source=SOURCE_RADAR), 10.4)
    merged = model.snapshot(10.5).robots[103]
    assert (merged.hp, merged.max_hp, merged.alive) == (150.0, 200.0, True)
    assert merged.source == SOURCE_REFEREE | SOURCE_RADAR and merged.stamp == 10.4
    model.update_robot(RobotStatus(103, Team.ENEMY, 12.3, alive=True, source=SOURCE_RADAR), 12.3)
    later = model.snapshot(12.3).robots[103]
    assert later.hp is None and later.alive is True and later.source == SOURCE_RADAR


def test_world_model_rejects_invalid_typed_inputs(config):
    from sentry_core.model import GameState, RobotStatus, StructureState, Team
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    for status in (RobotStatus(103, Team.ENEMY, 11.0, hp=1.0),
                   RobotStatus(103, Team.ENEMY, 10.0, hp=-1.0),
                   RobotStatus(103, Team.ENEMY, 10.0, ammo=1.5),
                   RobotStatus(103, Team.UNKNOWN, 10.0, hp=1.0),
                   RobotStatus(0, Team.ENEMY, 10.0, hp=1.0),
                   RobotStatus(103, Team.ENEMY, 10.0 - config.robot_status_timeout - 1, hp=1.)):
        assert not model.update_robot(status, 10.0)
    assert model.update_robot(RobotStatus(103, Team.ENEMY, 10.0, hp=1.0), 10.0)
    assert not model.update_robot(RobotStatus(103, Team.ALLY, 10.1, hp=1.0), 10.1)
    assert not model.update_game(GameState(10.1, own_team="GREEN"), 10.1)
    assert not model.update_game(GameState(10.1, remaining=float("nan")), 10.1)
    assert not model.update_structure(StructureState("", 10.1, alive=True), 10.1)
    assert not model.update_structure(StructureState("own_base", 10.1, max_hp=0.0), 10.1)


def test_clock_rewind_clears_typed_inputs(config):
    from sentry_core.model import RobotStatus, StructureState, Team
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    model.update_game(game(10.0), 10.0)
    model.update_robot(RobotStatus(1, Team.ALLY, 10.0, hp=100.0, max_hp=200.0), 10.0)
    model.update_structure(StructureState("own_base", 10.0, alive=True), 10.0)
    assert model.snapshot(10.0).game is not None
    rewound = model.snapshot(1.0)
    assert rewound.game is None and rewound.robots == {} and rewound.structures == {}


def test_trace_round_trip_preserves_typed_inputs(config, world_factory):
    from sentry_core.model import RobotStatus, StructureState, Team
    from sentry_core.serde import json_line, world_from_dict, world_to_dict

    world = replace(world_factory(), game=game(10.0), robots={
        7: RobotStatus(7, Team.ALLY, 10.0, hp=300.0, max_hp=400.0, alive=True, level=2, ammo=0.5,
                       source=1),
        103: RobotStatus(103, Team.ENEMY, 9.9, alive=False, source=2),
    }, structures={"enemy_outpost": StructureState("enemy_outpost", 9.8, 600.0, 1500.0, True)})
    restored = world_from_dict(json.loads(json_line(world_to_dict(world))))
    assert restored == world
    old = world_to_dict(world_factory())
    for key in ("game", "robots", "structures"):
        old.pop(key)
    assert world_from_dict(old).robots == {}


def test_self_id_follows_own_team_unless_set_explicitly(tmp_path):
    import pytest
    import yaml

    from conftest import ROOT
    from sentry_core.config import load_config

    # No bundled tactics file sets self_id, including tactics_rmuc2027.yaml, the launch default:
    # own_team BLUE must give the blue sentry id there, not an explicit 7.
    for name in ("tactics.yaml", "tactics_rmuc2027.yaml", "tactics_rmuc2026.yaml"):
        bundled = ROOT / "src/sentry_bringup/config" / name
        assert "self_id" not in yaml.safe_load(bundled.read_text(encoding="utf-8")), name
        assert load_config(bundled, own_team="BLUE").self_id == 107, name
    tactics = ROOT / "src/sentry_bringup/config/tactics.yaml"
    raw = yaml.safe_load(tactics.read_text(encoding="utf-8"))
    # Keep the demo tactics' relative field file reachable from tmp_path.
    raw["field"] = str(tactics.parent / raw["field"])
    path = tmp_path / "tactics.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    assert load_config(path).self_id == 7
    assert load_config(path, own_team="blue").self_id == 107
    path.write_text(yaml.safe_dump({**raw, "own_team": "BLUE"}), encoding="utf-8")
    assert load_config(path).self_id == 107
    assert load_config(path, own_team="RED").self_id == 7
    with pytest.raises(ValueError, match="own_team"):
        load_config(path, own_team="GREEN")
    path.write_text(yaml.safe_dump({**raw, "self_id": 7}), encoding="utf-8")
    explicit = load_config(path, own_team="BLUE")
    assert (explicit.own_team, explicit.self_id) == ("BLUE", 7)
