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
    intercept = next(c for c in threat.candidates if c.key == threat.task.key)
    assert threat.task.urgent and threat.task.goal == intercept.goal
    assert threat.policy_info["subgoal_reason"] == "URGENT_TASK"
    # A 1.4 m forward hint lands in open space inside the base zone, so the projected
    # sub-goal is the hint itself and becomes the goal of the defense task the rules chose.
    engine = DecisionEngine(config, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0.07, 0.)), config))
    first = engine.step(world_factory())
    assert first.task.goal.distance(Vec2(4.4, 6)) < 1e-3
    engine.feedback("test", first.task.task_id, "FAILED", 10.1)
    later = engine.step(world_factory(now=10.5))
    assert "rl_subgoal:FAILURE_COOLDOWN" in later.rejected
    # Only the sub-goal cools down: the rule task resumes at its rule goal as a new task.
    rule = next(c for c in later.candidates if c.key == later.task.key)
    assert later.task.key == first.task.key and later.task.goal == rule.goal
    assert later.task.task_id > first.task.task_id


def test_closer_subgoal_never_changes_which_task_the_rules_chose(config, world_factory,
                                                                tmp_path):
    from sentry_core.model import TaskType, Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    # A raider makes a non-urgent intercept the best (blended) rule task. The 0.8 m hint
    # lands closer than every rule goal: as a competing candidate with the task of the
    # pushing teammate it won on its short travel and turned the decision into SUPPORT.
    world = world_factory(self_position=(1.0, 1.0), allies=((1, 14, 4, 1.4, 0.4),),
                          enemies=((11, 12, 9, -1, 0),))
    prior = GoalPrior.load(actor_file(tmp_path, (0.04, 0.)), config)
    decision = DecisionEngine(config, goal_prior=prior).step(world)
    best = max(decision.candidates, key=lambda c: (c.utility, c.goal_id))
    assert decision.task.kind == TaskType.INTERCEPT and not decision.task.urgent
    assert (decision.task.key, decision.task.goal) == (best.key, best.goal)
    own, subgoal = world.self_track.position, Vec2(*decision.policy_info["projected_goal"])
    assert all(own.distance(subgoal) < own.distance(c.goal) for c in decision.candidates
               if c.goal is not None)
    assert not decision.policy_info["subgoal_refined"]
    assert decision.policy_info["subgoal_reason"] == "NOT_REFINABLE:INTERCEPT"


def test_subgoal_moves_only_the_goal_of_the_chosen_support_or_defend_task(config, world_factory,
                                                                         tmp_path):
    from sentry_core.model import TaskType, Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    # A pushing teammate makes support_north (15, 8) the rule task. The sub-goal refines
    # it inside the outpost zone (2.8 m away) or within goal_update_distance of the rule
    # goal (0.6 m, outside the zone), but not next to the own base, 11 m away.
    ally = ((1, 14, 4, 1.4, 0.4),)
    for position, hint, refined in (((15.0, 4.0), (0., 0.1), True),
                                    ((13.0, 8.0), (0.07, 0.), True),
                                    ((3.0, 6.0), (0.04, 0.), False)):
        prior = GoalPrior.load(actor_file(tmp_path, hint), config)
        decision = DecisionEngine(config, goal_prior=prior).step(
            world_factory(self_position=position, allies=ally))
        best = max(decision.candidates, key=lambda c: (c.utility, c.goal_id))
        subgoal = Vec2(*decision.policy_info["projected_goal"])
        assert best.kind == TaskType.SUPPORT and best.goal_id == "support_north"
        assert decision.task.key == best.key
        assert decision.task.goal == (subgoal if refined else best.goal)
        assert decision.policy_info["subgoal_refined"] == refined
        assert decision.policy_info["subgoal_reason"] == (
            "REFINED" if refined else "OUTSIDE_TASK_AREA")
    # A refined defense goal follows the identity rule for moved goals: the next sub-goal,
    # 0.5 m away, keeps the task and its goal; the one after, 2 m away, re-issues the task.
    engine = DecisionEngine(config, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0.07, 0.)), config))
    first = engine.step(world_factory())
    near = engine.step(world_factory(now=15.0, self_position=(3.5, 6.0)))
    moved = engine.step(world_factory(now=20.0, self_position=(1.0, 6.0)))
    assert first.task.kind == TaskType.DEFEND and first.policy_info["subgoal_refined"]
    assert first.task.goal == Vec2(*first.policy_info["subgoal"])
    assert first.policy_info["subgoal_base"] == first.task.goal_id
    assert first.task.goal.distance(Vec2(4.4, 6.0)) < 1e-6
    assert near.policy_info["subgoal_reason"] == "REFINED"
    assert 0.4 < first.task.goal.distance(Vec2(*near.policy_info["subgoal"])) < 0.6
    assert near.task.identity == first.task.identity and near.task.goal == first.task.goal
    assert moved.arbitration == "GOAL_MOVED" and moved.task.key == first.task.key
    assert moved.task.task_id == first.task.task_id + 1
    assert moved.task.goal == Vec2(*moved.policy_info["subgoal"])
    assert moved.task.goal.distance(first.task.goal) > config.goal_update_distance


def test_urgent_hold_and_recover_tasks_keep_their_rule_goals(config, world_factory, tmp_path):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    def decide(world, hint=(0.04, 0.)):
        prior = GoalPrior.load(actor_file(tmp_path, hint), config)
        return DecisionEngine(config, goal_prior=prior).step(world)

    # At 50 % HP, RECOVER is a non-urgent rule task. The hint lands inside the supply
    # zone; as a competing SUPPORT candidate it used to win there.
    recover = decide(world_factory(self_position=(1.0, 1.0), allies=((1, 14, 4, 1.4, 0.4),),
                                   hp=0.5), (0., 0.1))
    urgent = decide(world_factory(enemies=((11, 5, 6, -1, 0),)))
    recovering = decide(world_factory(hp=0.1, supply=True))
    hold = decide(replace(world_factory(), self_track=None))
    for decision, kind, reason in ((recover, TaskType.RECOVER, "NOT_REFINABLE:RECOVER"),
                                   (urgent, TaskType.INTERCEPT, "URGENT_TASK"),
                                   (recovering, TaskType.RECOVER, "URGENT_TASK"),
                                   (hold, TaskType.HOLD, "NO_RULE_TASK")):
        chosen = next(c for c in decision.candidates if c.key == decision.task.key)
        assert decision.task.kind == kind and decision.task.goal == chosen.goal
        assert not decision.policy_info["subgoal_refined"]
        assert decision.policy_info["subgoal_reason"] == reason
        assert "subgoal" not in decision.policy_info
    assert not recover.task.urgent and urgent.task.urgent and recovering.task.urgent


def test_reached_subgoal_keeps_refining_its_task_under_normal_hysteresis(config, world_factory,
                                                                        tmp_path):
    from sentry_core.model import TaskType, Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior
    from test_rl_policy import actor_file

    engine = DecisionEngine(config, session_id="test", goal_prior=GoalPrior.load(
        actor_file(tmp_path, (0., 0.1)), config))
    raider = ((11, 8, 10, -1, -0.6),)
    first = engine.step(world_factory(self_position=(1.0, 7.3)))
    subgoal = Vec2(*first.policy_info["subgoal"])
    rule = next(c for c in first.candidates if c.key == first.task.key)
    # The 1.2 m hint refines the defense task inside the base zone, 3 m from its rule goal.
    assert first.task.kind == TaskType.DEFEND and first.task.goal == subgoal
    assert subgoal.distance(rule.goal) > config.goal_update_distance
    # The robot now stands on the sub-goal: the proposal is reached until the next
    # inference. From 11 s a moderate (non-urgent) raid makes an intercept better.
    steps = [engine.step(world_factory(now=10.5, self_position=(1.0, 8.5)))]
    steps += [engine.step(world_factory(now=now, enemies=raider, self_position=(1.0, 8.5)))
              for now in (11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0)]
    assert all(d.policy_info["status"] == "PROPOSAL_REACHED" for d in steps)
    # The task keeps its sub-goal (no jump back to the rule goal) until the rules switch.
    assert all(d.task.identity == first.task.identity and d.task.goal == subgoal
               and d.policy_info["subgoal_reason"] == "KEPT:PROPOSAL_REACHED"
               for d in steps[:-1])
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
