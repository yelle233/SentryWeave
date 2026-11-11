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


def test_moved_goal_keeps_counting_the_minimum_hold(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    # Without preferred base points the defense goals face the far raider and rotate
    # when it changes side; the support challenger appears at 10.5 s.
    dynamic = replace(config, points=tuple(p for p in config.points if p.zone != "own_base"))
    engine = DecisionEngine(dynamic, session_id="test")
    ally, north, south = ((1, 14, 4, 1.4, 0.4),), ((21, 14, 11.5, 0, 0),), ((21, 14, 0.5, 0, 0),)
    steps = [engine.step(world_factory(enemies=north)),
             engine.step(world_factory(now=10.5, allies=ally, enemies=north))]
    steps += [engine.step(world_factory(now=now, allies=ally, enemies=south))
              for now in (11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0)]
    assert steps[2].arbitration == "GOAL_MOVED" and steps[2].task.key == steps[0].task.key
    assert steps[2].task.task_id > steps[0].task.task_id
    # The re-issued task was first held at 10 s, so the hold ends at 13 s, not 14 s.
    assert [d.arbitration for d in steps] == [
        "INITIAL", "MIN_HOLD", "GOAL_MOVED", "MIN_HOLD", "MIN_HOLD", "MIN_HOLD",
        "CONFIRMING", "CONFIRMING", "CONFIRMED"]
    assert steps[0].task.kind == TaskType.DEFEND and steps[-1].task.kind == TaskType.SUPPORT


def test_point_with_several_tasks_is_rejected_once(config, world_factory):
    from sentry_core.policy import DecisionEngine

    # defense_south offers DEFEND and INTERCEPT; a teammate stands on it.
    result = DecisionEngine(config).step(world_factory(allies=((1, 4, 4, 0, 0),)))
    assert result.rejected.count("defense_south:ALLY_OCCUPIED") == 1


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
    # Default blend: the sub-goal carries its base's learned value, so it wins on its
    # closer position (rule part); at blend 1 it could only tie that base.
    engine = DecisionEngine(config, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0.1, 0.)), config))
    first = engine.step(world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    assert first.task.goal_id == "rl_subgoal"
    assert first.task.goal.distance(Vec2(5, 6)) < 1e-3
    engine.feedback("test", first.task.task_id, "FAILED", 10.1)
    later = engine.step(world_factory(now=10.5, allies=((1, 14.7, 4.2, 1.4, 0.4),)))
    assert "rl_subgoal:FAILURE_COOLDOWN" in later.rejected
    assert later.task.goal_id != "rl_subgoal"


def test_non_urgent_intercept_with_better_rule_utility_beats_the_subgoal(config, world_factory,
                                                                        tmp_path):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    # Default blend; a 1 m hint back towards the base while a raider is a moderate threat.
    prior = GoalPrior.load(actor_file(tmp_path, (-0.05, 0.)), config)
    result = DecisionEngine(config, goal_prior=prior).step(
        world_factory(enemies=((11, 8, 10, -1, -0.6),)))
    subgoal = next(c for c in result.candidates if c.goal_id == "rl_subgoal")
    chosen = next(c for c in result.candidates if c.key == result.task.key)
    assert result.task.kind == TaskType.INTERCEPT and not result.task.urgent
    assert chosen.rule_utility > subgoal.rule_utility + 0.2
    # The sub-goal lies on the proposal, so its own route would agree by construction.
    # Its learned part is the agreement of the rule route whose task it inherits.
    base = next(c for c in result.candidates if c.goal_id == result.policy_info["subgoal_base"]
                and c.kind == subgoal.kind)
    assert subgoal.learned_utility == pytest.approx(base.learned_utility)
    assert subgoal.learned_utility < 0.5


def test_reached_subgoal_stays_under_normal_hysteresis(config, world_factory, tmp_path):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    # The moderate raid below leads the reached sub-goal by ~0.06, so use a 0.05 margin.
    config = replace(config, switch_margin=0.05)
    engine = DecisionEngine(config, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0.1, 0.)), config))
    ally, raider = ((1, 14, 4, 1.4, 0.4),), ((11, 8, 10, -1, -0.6),)
    first = engine.step(world_factory(allies=ally))
    assert first.task.goal_id == "rl_subgoal"
    # The robot now stands on the sub-goal: the proposal is reached until the next
    # inference. From 11 s a moderate (non-urgent) raid makes an intercept better.
    steps = [engine.step(world_factory(now=10.5, allies=ally, self_position=(5.0, 6.0)))]
    steps += [engine.step(world_factory(now=now, allies=ally, enemies=raider,
                                        self_position=(5.0, 6.0)))
              for now in (11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0)]
    assert all(d.policy_info["status"] == "PROPOSAL_REACHED" for d in steps)
    assert all(d.task.identity == first.task.identity for d in steps[:-1])
    assert [d.arbitration for d in steps] == [
        "KEEP_CURRENT", "MIN_HOLD", "MIN_HOLD", "MIN_HOLD", "MIN_HOLD",
        "CONFIRMING", "CONFIRMING", "CONFIRMED"]
    assert steps[-1].task.kind == TaskType.INTERCEPT and not steps[-1].task.urgent


def test_screened_out_subgoal_is_not_reported_as_offered(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    # A teammate stands exactly where the 2 m forward hint lands.
    prior = GoalPrior.load(actor_file(tmp_path, (0.1, 0.)), config)
    result = DecisionEngine(config, goal_prior=prior).step(world_factory(allies=((2, 5, 6, 0, 0),)))
    assert result.policy_info["projected_goal"] == pytest.approx([5, 6])
    assert "rl_subgoal:ALLY_OCCUPIED" in result.rejected
    assert not any(c.goal_id == "rl_subgoal" for c in result.candidates)
    assert "subgoal" not in result.policy_info and "subgoal_base" not in result.policy_info


def test_candidate_parameters_are_validated(config):
    from sentry_core.config import validate_config

    for change in ({"defense_radius": 0.0}, {"contest_samples": 17}, {"defense_spread": 4.0},
                   {"goal_update_distance": float("nan")}):
        with pytest.raises(ValueError):
            validate_config(replace(config, **change))


def test_contest_samples_must_be_an_integer(config):
    from sentry_core.config import validate_config

    # 4.0 from a YAML file used to pass validation, then fail every decision in range().
    for value in (4.0, True, "4"):
        with pytest.raises(ValueError, match="contest_samples must be an integer"):
            validate_config(replace(config, contest_samples=value))
    assert validate_config(replace(config, contest_samples=16)).contest_samples == 16
