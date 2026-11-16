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


@pytest.mark.parametrize("phase, reason", [("UNKNOWN", "GAME_STATE_UNKNOWN"),
                                           ("READY", "GAME_NOT_RUNNING")])
def test_match_start_moves_on_the_first_running_cycle(config, world_factory, phase, reason):
    from sentry_core.model import Phase, TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    hold = engine.step(world_factory(now=10.0, phase=Phase[phase]))
    assert hold.task.kind == TaskType.HOLD and hold.task.reason == reason
    start = engine.step(world_factory(now=10.1))
    assert start.arbitration == "SAFETY_RECOVERED"
    assert start.task.kind == TaskType.DEFEND and start.task.goal is not None
    assert start.task.task_id == hold.task.task_id + 1 and start.task.created_at == 10.1


@pytest.mark.parametrize("dead_for", [1.0, 10.0])
def test_respawn_moves_on_the_first_alive_cycle(config, world_factory, dead_for):
    from sentry_core.model import RobotStatus, Team, TaskType
    from sentry_core.policy import DecisionEngine

    def world(now, alive):
        status = RobotStatus(config.self_id, Team.ALLY, now, 400.0 if alive else 0.0, 400.0,
                             alive, source=1)
        return replace(world_factory(now=now), robots={config.self_id: status})

    engine = DecisionEngine(config, session_id="test")
    assert engine.step(world(10.0, True)).task.kind == TaskType.DEFEND
    dead = [engine.step(world(10.5 + 0.5 * k, False)) for k in range(int(2 * dead_for))]
    assert all(d.task.kind == TaskType.HOLD and d.task.reason == "ROBOT_DISABLED" for d in dead)
    alive = engine.step(world(10.5 + dead_for, True))
    assert alive.arbitration == "SAFETY_RECOVERED"
    assert alive.task.kind == TaskType.DEFEND and alive.task.goal is not None


@pytest.mark.parametrize("reason", ["FRAME_MISMATCH", "LOCALIZATION_INVALID", "GAME_STATE_UNKNOWN",
                                    "GAME_NOT_RUNNING", "TEAM_MISMATCH", "ROBOT_DISABLED"])
def test_every_safety_hold_ends_with_the_best_task_at_once(config, world_factory, reason):
    from sentry_core.model import GameState, Phase, TaskType
    from sentry_core.policy import DecisionEngine

    unsafe = {
        "FRAME_MISMATCH": lambda now: replace(world_factory(now=now), frame="odom"),
        "LOCALIZATION_INVALID": lambda now: replace(world_factory(now=now), self_track=None),
        "GAME_STATE_UNKNOWN": lambda now: world_factory(now=now, phase=Phase.UNKNOWN),
        "GAME_NOT_RUNNING": lambda now: world_factory(now=now, phase=Phase.READY),
        "TEAM_MISMATCH": lambda now: replace(world_factory(now=now), game=GameState(
            now, own_team="BLUE", phase=Phase.RUNNING)),
        "ROBOT_DISABLED": lambda now: world_factory(now=now, hp=0.0),
    }[reason]
    engine = DecisionEngine(config, session_id="test")
    assert engine.step(world_factory(now=10.0)).task.kind == TaskType.DEFEND
    held = engine.step(unsafe(10.5))
    assert held.task.kind == TaskType.HOLD and held.task.reason == reason
    safe = world_factory(now=11.0)
    resumed = engine.step(safe)
    assert resumed.arbitration == "SAFETY_RECOVERED"
    assert resumed.task.key == DecisionEngine(config).step(safe).task.key
    assert resumed.task.task_id == held.task.task_id + 1


def test_after_a_safety_hold_real_tasks_keep_hysteresis_and_urgent_preemption(config,
                                                                              world_factory):
    from sentry_core.model import Phase, TaskType
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="test")
    # Held since 5 s: a hold timer counted from the HOLD would already have expired.
    for now in (5.0, 7.5):
        engine.step(world_factory(now=now, phase=Phase.READY))
    start = engine.step(world_factory(now=10.0))
    assert start.task.kind == TaskType.DEFEND and start.arbitration == "SAFETY_RECOVERED"
    # The new task's minimum hold starts at 10 s; a better task still needs confirmation.
    ally = ((1, 14, 4, 1.4, 0.4),)
    steps = [engine.step(world_factory(now=now, allies=ally))
             for now in (10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0)]
    assert [d.arbitration for d in steps] == [
        "MIN_HOLD", "MIN_HOLD", "MIN_HOLD", "MIN_HOLD", "MIN_HOLD",
        "CONFIRMING", "CONFIRMING", "CONFIRMED"]
    assert all(d.task.identity == start.task.identity for d in steps[:-1])
    assert steps[-1].task.kind == TaskType.SUPPORT
    # An urgent raid on the first safe cycle is still an urgent preemption.
    raid = DecisionEngine(config, session_id="test")
    raid.step(world_factory(now=10.0, phase=Phase.READY))
    urgent = raid.step(world_factory(now=10.1, enemies=((11, 3.5, 6, -1, 0),)))
    assert urgent.arbitration == "URGENT_PREEMPT"
    assert urgent.task.kind == TaskType.INTERCEPT and urgent.task.urgent
