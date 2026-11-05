from dataclasses import replace
import pytest


def test_infers_support_for_a_teammate_approaching_outpost(config, world_factory):
    from sentry_core.model import IntentType, TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    world = world_factory(allies=((1, 14, 4, 1.4, 0.4),))
    decision = engine.step(world)
    intent = decision.intents[0]
    assert intent.kind == IntentType.ATTACK_OUTPOST
    assert sum(intent.probabilities) == pytest.approx(1.0)
    assert decision.task.kind == TaskType.SUPPORT
    assert decision.task.goal.distance(world.allies[0].position) > 0.8


def test_imminent_base_threat_overrides_support_hold(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    before = engine.step(world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    after = engine.step(world_factory(now=10.5, allies=((1, 14.5, 4, 1.4, 0.4),),
                                      enemies=((11, 3.5, 6, -1, 0),)))
    assert before.task.kind == TaskType.SUPPORT
    assert after.task.kind == TaskType.INTERCEPT
    assert after.task.urgent
    assert after.task.task_id > before.task.task_id


def test_critical_resources_require_real_supply_availability(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    for supply, expected in ((True, TaskType.RECOVER), (False, TaskType.DEFEND),
                             (None, TaskType.DEFEND)):
        result = DecisionEngine(config).step(world_factory(hp=0.1, supply=supply))
        assert result.task.kind == expected


def test_missing_referee_and_localization_stop_but_external_loss_defends(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    world = world_factory()
    assert DecisionEngine(config).step(world).task.kind == TaskType.DEFEND
    for invalid in (replace(world, referee=None),
                    replace(world, self_track=None, mode="SAFE_STOP")):
        assert DecisionEngine(config).step(invalid).task.kind == TaskType.HOLD


def test_same_task_renews_lease_without_issuing_new_identity(config, world_factory):
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    first = engine.step(world_factory())
    second = engine.step(world_factory(now=10.5))
    assert second.task.identity == first.task.identity
    assert second.task.valid_until > first.task.valid_until
    assert second.task.created_at == first.task.created_at


def test_invalid_target_bypasses_minimum_hold(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config)
    before = engine.step(world_factory(allies=((1, 14, 4, 1.4, 0.4),)))
    after = engine.step(world_factory(now=10.5, enemy_outpost=False,
                                      allies=((1, 14.5, 4, 1.4, 0.4),)))
    assert before.task.kind == TaskType.SUPPORT
    assert after.task.kind == TaskType.DEFEND


def test_failed_point_cools_down_across_tasks(config, world_factory):
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    first = engine.step(world_factory()).task
    engine.feedback("old-session", first.task_id, "FAILED", 10.1)
    assert engine.step(world_factory(now=10.2)).task.goal_id == first.goal_id
    engine.feedback("test", first.task_id, "FAILED", 10.3)
    second = engine.step(world_factory(now=10.5)).task
    assert second.goal_id != first.goal_id
    assert second.task_id > first.task_id


def test_stale_track_cannot_keep_support_task_alive(config, world_factory):
    from sentry_core.model import TaskType, TrackStatus
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config)
    world = world_factory(allies=((1, 14, 4, 1.4, 0.4),))
    assert engine.step(world).task.kind == TaskType.SUPPORT
    stale = replace(world.allies[0], age=3.0, confidence=0.01, status=TrackStatus.STALE)
    result = engine.step(replace(world_factory(now=11), allies=(stale,), mode="LOCAL_ONLY"))
    assert result.task.kind == TaskType.DEFEND
    assert not result.intents[0].valid


def test_stationary_robot_away_from_zones_is_unknown(config, world_factory):
    from sentry_core.model import IntentType
    from sentry_core.policy import DecisionEngine

    result = DecisionEngine(config).step(world_factory(allies=((1, 11, 10, 0, 0),)))
    assert result.intents[0].kind == IntentType.UNKNOWN


def test_normal_challenger_requires_hold_margin_and_confirmation(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config)
    assert engine.step(world_factory()).task.kind == TaskType.DEFEND
    for now in (10.5, 11.0, 11.5, 12.0, 12.5):
        result = engine.step(world_factory(now=now, allies=((1, 14, 4, 1.4, 0.4),)))
        assert result.task.kind == TaskType.DEFEND
    for now in (13.0, 13.5, 14.0):
        result = engine.step(world_factory(now=now, allies=((1, 14, 4, 1.4, 0.4),)))
    assert result.task.kind == TaskType.SUPPORT


def test_current_task_canceled_by_watchdog_gets_a_fresh_identity(config, world_factory):
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    first = engine.step(world_factory()).task
    engine.feedback("test", first.task_id, "CANCELED", 10.2)
    resumed = engine.step(world_factory(now=10.5)).task
    assert resumed.key == first.key
    assert resumed.task_id > first.task_id


def test_unknown_resource_values_alone_do_not_trigger_recovery(config, world_factory):
    from sentry_core.model import TaskType
    from sentry_core.policy import DecisionEngine

    world = world_factory(hp=None, supply=True)
    assert DecisionEngine(config).step(world).task.kind == TaskType.DEFEND
    world = replace(world, referee=replace(world.referee, ammo=None))
    assert DecisionEngine(config).step(world).task.kind == TaskType.DEFEND
