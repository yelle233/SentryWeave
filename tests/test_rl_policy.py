from dataclasses import replace
import json

import numpy as np
import pytest


def actor_file(tmp_path, action=(0.4, 0.), width=20., height=12.):
    from sentry_core.navigation_features import FEATURE_NAMES, FEATURE_VERSION, NavigationSpec

    parameters = {"log_std": [-2., -2.]}
    for layer, input_size, output_size in ((0, 73, 8), (2, 8, 8), (4, 8, 2)):
        parameters[f"layers.{layer}.weight"] = np.zeros((output_size, input_size)).tolist()
        parameters[f"layers.{layer}.bias"] = np.zeros(output_size).tolist()
    parameters["layers.4.bias"] = np.arctanh(action).tolist()
    payload = {"artifact_type": "sentry_navigation_policy", "schema_version": 1,
               "feature_version": FEATURE_VERSION, "feature_names": list(FEATURE_NAMES),
               "algorithm": "iql", "spec": NavigationSpec(width=width, height=height).to_dict(),
               "architecture": {"type": "mlp", "hidden": 8}, "parameters": parameters,
               "target_return": 0., "return_scale": 1., "provenance": {"dataset_id": "test"}}
    path = tmp_path / "actor.json"
    path.write_text(json.dumps(payload))
    return path


def test_rl_rescores_rule_candidates_and_projects_one_legal_subgoal(config, world_factory,
                                                                    tmp_path):
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    world = world_factory(allies=((1, 14, 4, 1.4, 0.4),))
    baseline = DecisionEngine(config).step(world)
    prior = GoalPrior.load(actor_file(tmp_path), config, blend=1.)
    learned = DecisionEngine(config, goal_prior=prior).step(world)
    # Rule candidates are re-scored, never added or moved; the sub-goal is no candidate.
    assert {c.key: c.goal for c in baseline.candidates} == {
        c.key: c.goal for c in learned.candidates}
    assert any(a.utility != b.utility for a, b in zip(baseline.candidates, learned.candidates))
    subgoal = Vec2(*learned.policy_info["projected_goal"])
    arena = TacticalMap(config)
    assert arena.legal(subgoal) and arena.reachable(world.self_track.position, subgoal)
    assert learned.policy_info["used"]
    chosen = next(c for c in learned.candidates if c.key == learned.task.key)
    assert learned.task.goal in (chosen.goal, subgoal)


def test_rl_cannot_override_resource_or_localization_guards(config, world_factory, tmp_path):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    engine = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path), config, blend=1.))
    assert engine.step(world_factory(hp=0.1, supply=True)).task.kind == TaskType.RECOVER
    unsafe = replace(world_factory(now=10.5), self_track=None)
    assert engine.step(unsafe).task.kind == TaskType.HOLD


def test_out_of_field_prediction_falls_back_to_existing_policy(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    world = world_factory()
    engine = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path, (0.99, 0.)), config))
    result = engine.step(world)
    assert result.task.key == DecisionEngine(config).step(world).task.key
    assert result.policy_info["status"] == "OUT_OF_FIELD"


def test_domain_transfer_is_explicit_and_recorded(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    path = actor_file(tmp_path, width=28., height=15.)
    with pytest.raises(ValueError, match="dimensions"):
        GoalPrior.load(path, config)
    engine = DecisionEngine(config, goal_prior=GoalPrior.load(path, config, allow_transfer=True))
    assert engine.step(world_factory()).policy_info["domain_transfer"]


def test_macro_goal_stays_fixed_until_next_inference(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    engine = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path), config))
    first = engine.step(world_factory(now=10)).policy_info
    cached = engine.step(world_factory(now=11, self_position=(4, 6))).policy_info
    later = engine.step(world_factory(now=15, self_position=(5, 6))).policy_info
    assert cached["proposal"] == first["proposal"]
    assert cached["cache_age"] == 1
    assert later["proposal"] != first["proposal"]


def test_world_epoch_change_clears_rl_context_without_waiting_for_horizon(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    engine = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path), config))
    first = engine.step(world_factory(now=10)).policy_info
    reset = replace(world_factory(now=11, self_position=(4, 6)), epoch=1)
    second = engine.step(reset).policy_info
    assert second["cache_age"] == 0
    assert second["proposal"] != first["proposal"]


def test_rl_closed_loop_trace_replays_the_same_goal_decisions(config, tmp_path):
    from sentry_core.rl_policy import GoalPrior
    from sentry_training.evaluate import replay
    from sentry_training.simulation import rollout

    model = actor_file(tmp_path)
    trace = tmp_path / "episode.jsonl"
    actual = rollout(config, "team_push", 42, duration=12, policy="rl", trace_path=trace,
                     goal_prior=GoalPrior.load(model, config))
    replayed = replay(config, trace, policy="rl", goal_prior=GoalPrior.load(model, config))
    assert actual["task_sequence"] == replayed["task_sequence"]
    assert actual["rl_used_steps"] > 0


def test_short_forward_hint_prefers_aligned_route_over_nearby_side_point(config, world_factory, tmp_path):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    prior = GoalPrior.load(actor_file(tmp_path, (0.1, 0.)), config, blend=1.)
    result = DecisionEngine(config, goal_prior=prior).step(
        world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    # A 2 m forward suggestion is directional evidence for a distant support route.
    # Comparing it to a forced 7.5 m waypoint incorrectly favors a nearby defense point.
    assert result.task.kind == TaskType.SUPPORT


def test_consumed_macro_hint_cannot_pull_robot_backwards(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    engine = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path, (0.1, 0.)), config))
    engine.step(world_factory(now=10))
    reached = engine.step(world_factory(now=11, self_position=(5.4, 6)))
    passed = engine.step(world_factory(now=12, self_position=(6.2, 6)))
    assert reached.policy_info["status"] == "PROPOSAL_REACHED"
    assert passed.policy_info["status"] == "PROPOSAL_REACHED"
    assert not passed.policy_info["used"]


def test_zero_blend_logs_shadow_proposal_without_changing_decision(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    world = world_factory()
    baseline = DecisionEngine(config).step(world)
    shadow = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path), config, blend=0.)).step(world)
    assert shadow.task.key == baseline.task.key
    assert shadow.policy_info["status"] == "SHADOW"
    assert "proposal" in shadow.policy_info
    assert not shadow.policy_info["used"]


def test_zero_blend_never_refines_a_task_even_when_the_subgoal_would(config, world_factory,
                                                                     tmp_path):
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    path = actor_file(tmp_path, (0.04, 0.))
    worlds = [world_factory(now=now, self_position=(x, 6.0))
              for now, x in ((10.0, 3.0), (10.5, 3.4), (11.0, 3.8))]
    # The 0.8 m hint is legal (its own projection), closer than every rule goal, and at
    # the default blend it becomes the goal of the defense task the rules choose.
    active = DecisionEngine(config, goal_prior=GoalPrior.load(path, config)).step(worlds[0])
    first = DecisionEngine(config).step(worlds[0])
    assert active.task.key == first.task.key and active.policy_info["subgoal_refined"]
    assert active.task.goal == Vec2(*active.policy_info["projected_goal"]) != first.task.goal
    own, proposal = worlds[0].self_track.position, Vec2(*active.policy_info["proposal"])
    assert TacticalMap(config).legal(proposal)
    assert all(own.distance(proposal) < own.distance(c.goal) for c in active.candidates
               if c.goal is not None)
    shadow = DecisionEngine(config, session_id="test",
                            goal_prior=GoalPrior.load(path, config, blend=0.))
    rule = DecisionEngine(config, session_id="test")
    for world in worlds:
        decision, expected = shadow.step(world), rule.step(world)
        assert (decision.task.key, decision.task.goal) == (expected.task.key, expected.task.goal)
        assert decision.task.identity == expected.task.identity
        assert decision.arbitration == expected.arbitration
        assert "subgoal" not in decision.policy_info
        assert not decision.policy_info["subgoal_refined"]
        assert decision.policy_info["subgoal_reason"] == "SHADOW"
        assert decision.policy_info["status"] == "SHADOW" and not decision.policy_info["used"]


def test_reach_limit_applies_to_both_proposal_and_candidate_waypoints(config, world_factory, tmp_path):
    from sentry_core.policy import DecisionEngine
    from sentry_core.rl_policy import GoalPrior

    engine = DecisionEngine(config, goal_prior=GoalPrior.load(actor_file(tmp_path, (0.7, 0.)), config, blend=1.))
    engine.step(world_factory(now=10, allies=((1, 14, 4, 1.4, 0.4),)))
    late = engine.step(world_factory(now=14.5, allies=((1, 14, 4, 1.4, 0.4),)))
    # The suggestion is 14 m away but only 0.75 m of this macro remains reachable.
    # Both sides of the comparison must use that distance, not just the candidates.
    assert max(c.learned_utility or 0. for c in late.candidates) > 0.5
