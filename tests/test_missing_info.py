"""Missing-information matrix: each input alone and in combination never becomes 0 or full."""

from dataclasses import asdict, replace
import json

import pytest

NOW = 10.0
FAR_ENEMY = ((11, 16.0, 9.0, 0.0, 0.0),)


def build(config, *, ally=True, enemies=FAR_ENEMY, stale=(), phase=True, remaining=True,
          self_status=True, ally_alive=True, enemy_hp=150.0, structures=True, outpost=True):
    from sentry_core.model import (
        GameState, Observation, Phase, RobotStatus, StructureState, Team, Vec2,
    )
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    for stamp in (NOW - 2.5, NOW - 2.0, NOW - 1.5):
        for robot_id, x, y in stale:
            model.observe(Observation(robot_id, Team.ENEMY, Vec2(x, y), stamp), stamp)
    for stamp in (NOW - 1.0, NOW - 0.5, NOW):
        back = NOW - stamp
        model.observe(Observation(7, Team.ALLY, Vec2(3, 6), stamp), stamp)
        if ally:
            model.observe(Observation(1, Team.ALLY, Vec2(14 - 1.4 * back, 4 - 0.4 * back), stamp,
                                      velocity=Vec2(1.4, 0.4)), stamp)
        for robot_id, x, y, vx, vy in enemies:
            model.observe(Observation(robot_id, Team.ENEMY, Vec2(x - vx * back, y - vy * back),
                                      stamp, velocity=Vec2(vx, vy)), stamp)
    if phase:
        model.update_game(GameState(NOW, "rmuc2027", "RED", Phase.RUNNING,
                                    300.0 if remaining else None, True), NOW)
    if self_status:
        model.update_robot(RobotStatus(7, Team.ALLY, NOW, 300.0, 400.0, True, 1, 0.8, 1), NOW)
    model.update_robot(RobotStatus(1, Team.ALLY, NOW, 200.0, 400.0, ally_alive, source=1), NOW)
    if enemy_hp is not None:
        for robot_id, *_ in enemies:
            model.update_robot(RobotStatus(robot_id, Team.ENEMY, NOW, enemy_hp, 200.0, source=2),
                               NOW)
    if structures:
        for name in ("own_base", "own_outpost", "enemy_base"):
            model.update_structure(StructureState(name, NOW, alive=True, source=1), NOW)
        if outpost is not None:
            model.update_structure(StructureState("enemy_outpost", NOW, alive=outpost, source=1),
                                   NOW)
    return model.snapshot(NOW)


@pytest.fixture
def expected(config):
    return replace(config, expected_allies=1, expected_enemies=1)


def decide(config, world, **kwargs):
    from sentry_core.policy import DecisionEngine

    return DecisionEngine(config, session_id="test", **kwargs).step(world)


def candidate(decision, prefix):
    return next(c for c in decision.candidates if c.goal_id.startswith(prefix))


def test_full_information_supports_the_teammate(expected):
    from sentry_core.model import TaskType

    decision = decide(expected, build(expected))
    assert decision.task.kind == TaskType.SUPPORT
    assert decision.missing_info["completeness"] == {
        "game": 1.0, "self": 1.0, "allies": 1.0, "enemies": 1.0, "robot_status": 1.0,
        "structures": 1.0}
    assert decision.missing_info["missing"] == []


@pytest.mark.parametrize("changes, missing, kind", [
    ({"remaining": False}, "game.remaining_time", "SUPPORT"),
    ({"self_status": False}, "self.hp", "DEFEND"),
    ({"ally": False}, "allies.positions 0/1", "DEFEND"),
    ({"enemies": ()}, "enemies.positions 0/1", "SUPPORT"),
    ({"enemy_hp": None}, "robots.hp 1/2", "SUPPORT"),
    ({"outpost": None}, "structure.enemy_outpost", "SUPPORT"),
    ({"outpost": False}, None, "DEFEND"),
    ({"ally_alive": False}, None, "DEFEND"),
])
def test_single_missing_item_degrades_without_stopping(expected, changes, missing, kind):
    decision = decide(expected, build(expected, **changes))
    assert decision.task.kind.name == kind
    if missing:
        assert missing in decision.missing_info["missing"]


def test_missing_game_phase_is_the_only_input_that_stops_motion(expected):
    from sentry_core.model import TaskType

    decision = decide(expected, build(expected, phase=False))
    assert decision.task.kind == TaskType.HOLD
    assert decision.task.reason == "GAME_STATE_UNKNOWN"
    assert "game.phase" in decision.missing_info["missing"]


def test_missing_enemy_positions_raise_key_area_defense(expected):
    seen = decide(expected, build(expected))
    full = candidate(seen, "defense:own_base:0")
    assert full.features[2] == pytest.approx(0.15 + 0.8 * seen.threats[0].score)
    blind = decide(expected, build(expected, enemies=()))
    boosted = candidate(blind, "defense:own_base:0")
    assert blind.threats[0].score == 0
    assert boosted.features[2] == pytest.approx(0.15 + expected.missing_enemy_defense_boost)
    assert any(d.startswith("ENEMY_POSITIONS_MISSING") for d in blind.missing_info["degraded"])


def test_stale_enemy_keeps_a_fading_threat_at_its_last_position(expected):
    fresh = decide(expected, build(expected, enemies=((11, 4.5, 8.0, 0.0, 0.0),)))
    stale = decide(expected, build(expected, enemies=(), stale=((11, 4.5, 8.0),)))
    assert fresh.threats[0].enemy_id == stale.threats[0].enemy_id == 11
    assert 0 < stale.threats[0].score < fresh.threats[0].score
    assert "STALE_ENEMY:11" in stale.missing_info["degraded"]


def test_enemy_health_orders_targets_only_when_known(expected):
    from sentry_core.model import RobotStatus, Team

    pair = ((11, 5.0, 6.0, -1.0, 0.0), (12, 2.0, 9.0, 0.0, -1.0))
    unknown = build(expected, enemies=pair, enemy_hp=None)
    assert decide(expected, unknown).threats[0].enemy_id == 11
    weak = replace(unknown, robots={**unknown.robots,
                                    12: RobotStatus(12, Team.ENEMY, NOW, 40.0, 200.0)})
    assert decide(expected, weak).threats[0].enemy_id == 12
    healthy = replace(unknown, robots={**unknown.robots,
                                       11: RobotStatus(11, Team.ENEMY, NOW, 200.0, 200.0)})
    assert decide(expected, healthy).threats[0].enemy_id == 11
    assert "ENEMY_HP_UNKNOWN:11" in decide(expected, unknown).missing_info["degraded"]


def test_unconfirmed_outpost_is_alive_with_less_attack_weight(expected):
    confirmed = candidate(decide(expected, build(expected)), "support_north")
    unconfirmed = decide(expected, build(expected, outpost=None))
    lowered = candidate(unconfirmed, "support_north")
    assert lowered.features[0] == pytest.approx(
        confirmed.features[0] * expected.unconfirmed_attack_factor)
    assert "STRUCTURE_UNCONFIRMED:enemy_outpost" in unconfirmed.missing_info["degraded"]
    destroyed = decide(expected, build(expected, outpost=False))
    assert any(r.endswith("TARGET_DESTROYED") for r in destroyed.rejected)


def test_dead_teammate_is_not_followed(expected):
    decision = decide(expected, build(expected, ally_alive=False))
    assert any(r.endswith("ALLY_DOWN") for r in decision.rejected)
    assert decision.missing_info["completeness"]["allies"] == 1.0


def test_radar_loss_defends_and_suppresses_the_rl_blend(expected, tmp_path):
    from sentry_core.model import TaskType
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    world = build(expected, ally=False, enemies=(), enemy_hp=None)
    decision = decide(expected, world, goal_prior=GoalPrior.load(actor_file(tmp_path), expected))
    assert decision.task.kind == TaskType.DEFEND
    assert decision.missing_info["rl_blend"] == {
        "configured": 0.25, "effective": 0.0, "reason": "INFO_INCOMPLETE:allies,enemies"}
    assert decision.policy_info["status"] == "SHADOW"
    assert "RL_BLEND_REDUCED:INFO_INCOMPLETE:allies,enemies" in decision.missing_info["degraded"]


def test_partial_enemy_picture_lowers_but_keeps_the_rl_blend(expected, tmp_path):
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    two = replace(expected, expected_enemies=2)
    prior = GoalPrior.load(actor_file(tmp_path), two)
    decision = decide(two, build(two), goal_prior=prior)
    blend = decision.missing_info["rl_blend"]
    assert blend["effective"] == pytest.approx(0.25 * 0.5 / two.completeness_threshold)
    assert decision.policy_info["effective_blend"] == pytest.approx(blend["effective"])
    assert blend["reason"] == "INFO_INCOMPLETE:enemies"


def test_only_game_phase_and_pose_still_defend(expected):
    from sentry_core.model import TaskType

    decision = decide(expected, build(expected, remaining=False, self_status=False,
                                      structures=False, ally=False, enemies=(), enemy_hp=None))
    assert decision.task.kind == TaskType.DEFEND
    missing = decision.missing_info["missing"]
    for item in ("game.remaining_time", "self.hp", "self.ammo", "allies.positions 0/1",
                 "enemies.positions 0/1", "structure.own_base", "structure.enemy_outpost"):
        assert item in missing


def test_nothing_but_localization_holds(expected):
    from sentry_core.model import TaskType

    decision = decide(expected, build(expected, phase=False, self_status=False, structures=False,
                                      ally=False, enemies=(), enemy_hp=None))
    assert decision.task.kind == TaskType.HOLD


def test_missing_info_is_part_of_the_trace_record(expected):
    from sentry_core.serde import json_line

    record = json.loads(json_line(asdict(decide(expected, build(expected, enemies=())))))
    assert set(record["missing_info"]) == {"completeness", "missing", "degraded"}
    assert record["missing_info"]["completeness"]["enemies"] == 0.0


def test_stale_enemy_weight_fades_linearly_until_the_track_is_dropped(expected):
    world = build(expected, enemies=(), stale=((11, 4.5, 8.0),))
    track = world.enemies[0]

    def threat(age):
        return decide(expected, replace(world, enemies=(replace(track, age=age),))).threats[0]

    def fade(age):
        return 1 - (age - expected.predicted_age) / (expected.retention - expected.predicted_age)

    recent, older, dropped = threat(track.age), threat(3.0), threat(expected.retention)
    assert older.score == pytest.approx(recent.score * fade(3.0) / fade(track.age))
    assert 0 < older.score < recent.score
    assert dropped.score == 0 and dropped.enemy_id is None


def test_missing_teammates_lower_support_cooperation(expected):
    full = candidate(decide(expected, build(expected)), "support_north")
    two = replace(expected, expected_allies=2)
    half = decide(two, build(two))
    lowered = candidate(half, "support_north")
    assert half.missing_info["completeness"]["allies"] == 0.5
    assert lowered.features[1] == pytest.approx(full.features[1] * 0.75)
    assert "ALLIES_INCOMPLETE:support*0.75" in half.missing_info["degraded"]


def test_destroyed_enemy_is_no_threat(expected):
    from sentry_core.model import RobotStatus, Team, TaskType

    close = ((11, 5.0, 6.0, -1.0, 0.0),)
    world = build(expected, enemies=close)
    down = replace(world, robots={**world.robots,
                                  11: RobotStatus(11, Team.ENEMY, NOW, 0.0, 200.0)})
    alive, destroyed = decide(expected, world), decide(expected, down)
    assert alive.threats[0].enemy_id == 11 and alive.task.kind == TaskType.INTERCEPT
    assert destroyed.threats[0].enemy_id is None and destroyed.task.kind != TaskType.INTERCEPT
    # A robot known to be down is not expected on the field: nothing is missing.
    assert destroyed.missing_info["completeness"]["enemies"] == 1.0


def test_reduced_blend_is_the_one_used_for_scoring(expected, tmp_path):
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    two = replace(expected, expected_enemies=2)
    prior = GoalPrior.load(actor_file(tmp_path, (0.1, 0.)), two)
    decision = decide(two, build(two), goal_prior=prior)
    effective = decision.policy_info["effective_blend"]
    blended = [c for c in decision.candidates if c.learned_utility is not None]
    assert 0 < effective < prior.blend and blended and decision.policy_info["used"]
    for c in blended:
        assert c.utility == pytest.approx(
            (1 - effective) * c.rule_utility + effective * c.learned_utility)


def test_blind_cycle_suspends_subgoal_refinement_and_logs_why(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    # One enemy is expected. While it is seen, the 1.2 m hint refines the defense task 3 m
    # from its rule goal. A cycle without it lowers the blend to 0 (SHADOW): the sub-goal
    # does not refine the task, which returns to its rule goal, and the log says why.
    one = replace(config, expected_enemies=1)
    engine = DecisionEngine(one, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0., 0.1)), one))
    first, blind, back = (
        engine.step(world_factory(now=now, enemies=enemies, self_position=(1.0, 7.3)))
        for now, enemies in ((10.0, FAR_ENEMY), (10.5, ()), (11.0, FAR_ENEMY)))
    rule = next(c.goal for c in blind.candidates if c.key == blind.task.key)
    assert first.policy_info["subgoal_refined"]
    assert first.task.goal.distance(rule) > one.goal_update_distance
    assert blind.policy_info["status"] == "SHADOW" and blind.policy_info["effective_blend"] == 0
    assert not blind.policy_info["subgoal_refined"]
    assert blind.policy_info["subgoal_reason"] == "SHADOW"
    assert blind.policy_info["blend_reason"] == "INFO_INCOMPLETE:enemies"
    assert "RL_BLEND_REDUCED:INFO_INCOMPLETE:enemies" in blind.missing_info["degraded"]
    assert blind.task.key == first.task.key and blind.task.goal == rule
    assert blind.arbitration == "GOAL_MOVED"
    # With the enemy back, the configured blend returns and refinement resumes.
    assert back.policy_info["effective_blend"] == engine.goal_prior.blend
    assert "blend_reason" not in back.policy_info
    assert back.policy_info["subgoal_refined"] and back.task.goal == first.task.goal


def test_missing_info_parameters_are_validated(config):
    from sentry_core.config import validate_config

    for change in ({"completeness_threshold": 1.5}, {"stale_enemy_weight": float("nan")},
                   {"missing_enemy_defense_boost": -0.1}, {"unconfirmed_attack_factor": 2.0},
                   {"enemy_low_hp_bonus": float("inf")}, {"expected_allies": -1},
                   {"expected_enemies": 33}, {"expected_allies": 1.5},
                   {"expected_enemies": True}):
        with pytest.raises(ValueError):
            validate_config(replace(config, **change))
    valid = validate_config(replace(config, expected_allies=3, expected_enemies=4))
    assert (valid.expected_allies, valid.expected_enemies) == (3, 4)
