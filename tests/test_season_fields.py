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


def _rmuc2027_world(config, self_position, *, allies=(), enemies=(), now=10.0):
    from sentry_core.model import GameState, Observation, Phase, RobotStatus, StructureState, Team, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    for stamp in (now - 1.0, now - 0.5, now):
        back = now - stamp
        if config.bounds[0] <= self_position[0] <= config.bounds[2] and (
                config.bounds[1] <= self_position[1] <= config.bounds[3]):
            model.observe(Observation(config.self_id, Team.ALLY, Vec2(*self_position), stamp), stamp)
        else:
            model.observe(Observation(config.self_id, Team.ALLY, Vec2(6.571, 7.5), stamp), stamp)
        for robot_id, x, y, vx, vy in allies:
            model.observe(Observation(robot_id, Team.ALLY, Vec2(x - vx * back, y - vy * back),
                                      stamp, velocity=Vec2(vx, vy)), stamp)
        for robot_id, x, y, vx, vy in enemies:
            model.observe(Observation(robot_id, Team.ENEMY, Vec2(x - vx * back, y - vy * back),
                                      stamp, velocity=Vec2(vx, vy)), stamp)
    for robot_id, *_ in allies:
        model.update_robot(RobotStatus(robot_id, Team.ALLY, now, 200.0, 200.0, True, source=1), now)
    for robot_id, *_ in enemies:
        model.update_robot(RobotStatus(robot_id, Team.ENEMY, now, 200.0, 200.0, True, source=2), now)
    model.update_game(GameState(now, config.season, config.own_team, Phase.RUNNING, 300.0, True), now)
    model.update_robot(RobotStatus(config.self_id, Team.ALLY, now, 400.0, 400.0, True, ammo=0.8,
                                   source=1), now)
    for name in ("own_base", "own_outpost", "enemy_base", "enemy_outpost"):
        model.update_structure(StructureState(name, now, alive=True, source=1), now)
    world = model.snapshot(now)
    if not (config.bounds[0] <= self_position[0] <= config.bounds[2]
            and config.bounds[1] <= self_position[1] <= config.bounds[3]):
        world = replace(world, self_track=replace(world.self_track, position=Vec2(*self_position)))
    return world


def test_rmuc2027_kerb_margin_pose_decides_and_ranks_with_rl_instead_of_holding(tmp_path):
    from sentry_core.config import load_config
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import TaskType, Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    config = load_config(CONFIG / "tactics_rmuc2027.yaml")
    arena = TacticalMap(config)
    kerb_pose = (4.6224, 10.7291)  # 0.33 m from red_kerb_2, inside the 0.35 m inflation margin
    assert not arena.legal(Vec2(*kerb_pose))
    allies = ((1, 6.0, 5.5, 0.0, 0.0), (2, 7.5, 5.5, 0.0, 0.0), (3, 7.5, 9.5, 0.0, 0.0))
    enemies = ((101, 22.0, 6.0, 0.0, 0.0), (102, 22.0, 9.0, 0.0, 0.0),
               (103, 23.5, 5.5, 0.0, 0.0), (104, 23.5, 9.5, 0.0, 0.0))
    world = _rmuc2027_world(config, kerb_pose, allies=allies, enemies=enemies)

    rule_decision = DecisionEngine(config, session_id="kerb-rule").step(world)
    assert rule_decision.task.kind != TaskType.HOLD
    assert rule_decision.task.reason != "LOCALIZATION_INVALID"
    origin = arena.origin(Vec2(*kerb_pose))
    assert origin is not None and arena.legal(origin)
    assert rule_decision.task.goal is not None and arena.legal(rule_decision.task.goal)
    assert arena.reachable(origin, rule_decision.task.goal)
    assert all(arena.legal(c.goal) for c in rule_decision.candidates if c.goal is not None)

    actor = actor_file(tmp_path, action=(0.06, -0.2), width=28.0, height=15.0)
    prior = GoalPrior.load(actor, config, blend=1.0)
    rl_decision = DecisionEngine(config, session_id="kerb-rl", goal_prior=prior).step(world)
    assert rl_decision.task.kind != TaskType.HOLD
    assert rl_decision.policy_info["status"] == "READY"
    assert rl_decision.policy_info["used"] is True
    assert rl_decision.policy_info["subgoal_refined"] is True
    assert arena.legal(rl_decision.task.goal) and arena.reachable(origin, rl_decision.task.goal)

    blocked = TacticalMap(replace(config, obstacles=((0.0, 0.0, 28.0, 15.0),), points=()))
    no_proj_prior = GoalPrior.load(actor, config, blend=1.0)
    unchanged = no_proj_prior.rank(world, list(rule_decision.candidates), blocked)
    assert unchanged == list(rule_decision.candidates)
    assert no_proj_prior.info["status"] == "NO_LEGAL_PROJECTION"


@pytest.mark.parametrize("out_of_bounds", [(-0.5, 7.5), (28.5, 7.5), (14.0, -0.2), (14.0, 15.2)])
def test_rmuc2027_out_of_bounds_pose_still_holds_with_localization_invalid(out_of_bounds):
    from sentry_core.config import load_config
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    config = load_config(CONFIG / "tactics_rmuc2027.yaml")
    world = _rmuc2027_world(config, out_of_bounds)
    assert world.self_track is not None
    decision = DecisionEngine(config, session_id="oob").step(world)
    assert decision.task.kind == TaskType.HOLD
    assert decision.task.reason == "LOCALIZATION_INVALID"


def test_rmuc2027_ally_in_obstacle_margin_keeps_attack_intent_and_support():
    from sentry_core.config import load_config
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import IntentType, TaskType
    from sentry_core.policy import DecisionEngine

    config = load_config(CONFIG / "tactics_rmuc2027.yaml")
    arena = TacticalMap(config)

    # Ally pushing enemy_outpost while 0.33 m north of blue_near_tunnel_side (y=2.08 -> 2.41).
    push_world = _rmuc2027_world(config, (6.571, 7.5), allies=((1, 14.5, 2.41, 1.4, 0.0),))
    assert not arena.legal(push_world.allies[0].position)
    push_decision = DecisionEngine(config, session_id="ally-margin").step(push_world)
    intent = push_decision.intents[0]
    assert intent.kind == IntentType.ATTACK_OUTPOST and intent.valid
    assert any(c.kind == TaskType.SUPPORT and c.zone_id == "enemy_outpost" and arena.legal(c.goal)
               for c in push_decision.candidates)
    assert push_decision.task.kind == TaskType.SUPPORT
    assert arena.legal(push_decision.task.goal)


@pytest.mark.parametrize("enemy_spec", [
    ((101, 3.2, 6.32, 1.0, 0.0),),      # 0.33 m south of red_base_body (y=6.65 -> 6.32)
    ((101, 5.4, 10.7291, 1.2, 0.0),),   # 0.33 m south of red_kerb_2 (y=11.059 -> 10.7291)
    ((101, 1.5, 4.67, 0.0, 1.0),),      # standing on red_high_point_south_step_2
])
def test_rmuc2027_enemy_in_kerb_margin_keeps_base_threat_and_intercept(enemy_spec):
    from sentry_core.config import load_config
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    config = load_config(CONFIG / "tactics_rmuc2027.yaml")
    arena = TacticalMap(config)
    raid_world = _rmuc2027_world(config, (6.571, 7.5), enemies=enemy_spec)
    assert not arena.legal(raid_world.enemies[0].position)
    raid_decision = DecisionEngine(config, session_id="enemy-margin").step(raid_world)
    base_threat = next(t for t in raid_decision.threats if t.zone_id == "own_base")
    assert base_threat.score > 0.2 and base_threat.enemy_id == 101
    assert any(c.kind == TaskType.INTERCEPT and c.zone_id == "own_base" and arena.legal(c.goal)
               for c in raid_decision.candidates)
    assert raid_decision.task.kind == TaskType.INTERCEPT
    assert arena.legal(raid_decision.task.goal)
