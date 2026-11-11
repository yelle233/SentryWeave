"""Dynamic candidate goals: legality, geometry, and the RL sub-goal contract."""

from dataclasses import replace

import pytest


def test_generated_goals_are_legal_reachable_and_cover_every_source(config, world_factory):
    from sentry_core.candidates import GoalGenerator
    from sentry_core.geometry import TacticalMap
    from sentry_core.policy import DecisionEngine

    world = world_factory(allies=((1, 14, 4, 1.4, 0.4),), enemies=((11, 6, 9, -1, -0.5),))
    engine = DecisionEngine(config)
    intents, threats = engine.estimator.estimate(world), engine._threats(world)
    arena = TacticalMap(config)
    own = world.self_track.position
    bare = GoalGenerator(replace(config, points=()), arena).generate(world, intents, threats)
    assert {g.source for g in bare} == {"intercept", "defense", "support", "contest", "recovery"}
    goals = GoalGenerator(config, arena).generate(world, intents, threats)
    assert {p.name for p in config.points} <= {g.goal_id for g in goals}
    # A generated goal that repeats a preferred point of the same task is dropped.
    assert not any(g.goal_id == "recovery:recovery" for g in goals)
    for goal in (*bare, *goals):
        assert arena.legal(goal.position) and arena.reachable(own, goal.position), goal


def test_field_without_preferred_points_still_decides(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    bare = replace(config, points=())
    defend = DecisionEngine(bare).step(world_factory())
    assert defend.task.kind == TaskType.DEFEND and defend.task.goal_id.startswith("defense:")
    support = DecisionEngine(bare).step(world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    assert support.task.kind == TaskType.SUPPORT and support.task.target_id == 1
    assert support.task.goal_id.startswith("support:enemy_outpost:1:")
    recover = DecisionEngine(bare).step(world_factory(hp=0.1, supply=True))
    assert recover.task.kind == TaskType.RECOVER
    assert recover.task.goal_id == "recovery:recovery"


def test_intercept_point_lies_between_structure_and_threat(config, world_factory):
    from sentry_core.model import TaskType, Vec2
    from sentry_core.policy import DecisionEngine

    result = DecisionEngine(config).step(world_factory(enemies=((11, 5, 6, -1, 0),)))
    intercept = next(c for c in result.candidates if c.goal_id == "intercept:own_base:11")
    assert intercept.kind == TaskType.INTERCEPT and intercept.target_id == 11
    assert intercept.goal.distance(Vec2(2 + 0.6 * 3, 6)) < 0.05
    assert result.task.kind == TaskType.INTERCEPT and result.task.urgent


def test_structure_inside_a_keepout_gets_legal_defense_points(config, world_factory):
    from sentry_core.config import Area
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2
    from sentry_core.policy import DecisionEngine

    body = Area("own_base_body", (Vec2(1, 5), Vec2(3, 5), Vec2(3, 7), Vec2(1, 7)))
    sealed = replace(config, keepouts=(body,))
    result = DecisionEngine(sealed).step(world_factory(self_position=(5.0, 6.0)))
    arena = TacticalMap(sealed)
    defense = [c for c in result.candidates if c.goal_id.startswith("defense:")]
    assert len(defense) == 3
    for candidate in defense:
        assert arena.legal(candidate.goal)
        assert candidate.goal.distance(Vec2(2, 6)) <= config.defense_radius + 0.2


def test_moved_dynamic_goal_gets_a_new_task_identity(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    first = engine.step(world_factory(enemies=((11, 5, 6, -1, 0),)))
    second = engine.step(world_factory(now=10.5, enemies=((11, 2, 9, 0, -1),)))
    assert first.task.kind == second.task.kind == TaskType.INTERCEPT
    assert first.task.key == second.task.key
    assert second.task.task_id > first.task.task_id
    assert second.arbitration == "GOAL_MOVED"
    assert second.task.goal.distance(first.task.goal) > config.goal_update_distance


def test_fixed_baseline_only_uses_preferred_points(config, world_factory):
    from sentry_core.policy import DecisionEngine

    result = DecisionEngine(config, baseline=True).step(
        world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    names = {p.name for p in config.points}
    assert all(c.goal_id in names for c in result.candidates if c.goal is not None)


def test_rl_subgoal_respects_urgency_and_failure_cooldown(config, world_factory, tmp_path):
    from sentry_core.model import Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    urgent = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path), config,
                                                              blend=1.))
    threat = urgent.step(world_factory(enemies=((11, 5, 6, -1, 0),)))
    assert not any(c.goal_id == "rl_subgoal" for c in threat.candidates)
    # A 2 m forward hint lands in open space, so the projected sub-goal is the hint itself.
    engine = DecisionEngine(config, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0.1, 0.)), config, blend=1.))
    first = engine.step(world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    assert first.task.goal_id == "rl_subgoal"
    assert first.task.goal.distance(Vec2(5, 6)) < 1e-3
    engine.feedback("test", first.task.task_id, "FAILED", 10.1)
    later = engine.step(world_factory(now=10.5, allies=((1, 14.7, 4.2, 1.4, 0.4),)))
    assert "rl_subgoal:FAILURE_COOLDOWN" in later.rejected
    assert later.task.goal_id != "rl_subgoal"


def test_candidate_parameters_are_validated(config):
    from sentry_core.config import validate_config

    for change in ({"defense_radius": 0.0}, {"contest_samples": 17}, {"defense_spread": 4.0},
                   {"goal_update_distance": float("nan")}):
        with pytest.raises(ValueError):
            validate_config(replace(config, **change))
