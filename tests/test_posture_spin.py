"""RMUC 2027 referee/radar boundary, HP-drop hit synthesis, PosturePolicy and SpinPolicy."""

from dataclasses import replace
import json

import pytest


def make_running_world(
    config,
    now=10.0,
    self_pos=(3.0, 6.0),
    enemies=(),
    allies=(),
    self_status=None,
    own_gold=250.0,
    enemy_gold=None,
    phase=None,
):
    from sentry_core.model import (
        SOURCE_ONBOARD,
        SOURCE_RADAR,
        SOURCE_REFEREE,
        GameState,
        Observation,
        Phase,
        RobotStatus,
        StructureState,
        Team,
        Vec2,
    )
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    for stamp in (now - 0.4, now - 0.2, now):
        model.observe(
            Observation(
                config.self_id,
                Team.ALLY,
                Vec2(*self_pos),
                stamp,
                source_mask=SOURCE_ONBOARD,
            ),
            stamp,
        )
        for rid, x, y, vx, vy in allies:
            model.observe(
                Observation(
                    rid,
                    Team.ALLY,
                    Vec2(x - vx * (now - stamp), y - vy * (now - stamp)),
                    stamp,
                    velocity=Vec2(vx, vy),
                    source_mask=SOURCE_REFEREE,
                ),
                stamp,
            )
        for rid, x, y, vx, vy in enemies:
            model.observe(
                Observation(
                    rid,
                    Team.ENEMY,
                    Vec2(x - vx * (now - stamp), y - vy * (now - stamp)),
                    stamp,
                    velocity=Vec2(vx, vy),
                    source_mask=SOURCE_RADAR,
                ),
                stamp,
            )
    model.update_game(
        GameState(
            now,
            "rmuc2027",
            config.own_team,
            Phase.RUNNING if phase is None else phase,
            300.0,
            True,
            SOURCE_REFEREE,
            own_gold=own_gold,
            enemy_gold=enemy_gold,
        ),
        now,
    )
    status = self_status or RobotStatus(
        config.self_id,
        Team.ALLY,
        now,
        hp=400.0,
        max_hp=400.0,
        alive=True,
        ammo=0.8,
        source=SOURCE_REFEREE,
    )
    model.update_robot(status, now)
    for name in ("own_base", "own_outpost", "enemy_base", "enemy_outpost"):
        model.update_structure(StructureState(name, now, alive=True, source=SOURCE_REFEREE), now)
    return model, model.snapshot(now)


def test_world_model_synthesizes_hit_on_self_hp_drop_and_ignores_sticky_old_hit(config):
    from sentry_core.inputs import view_inputs
    from sentry_core.model import SOURCE_REFEREE, RobotStatus, Team
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    assert model.update_robot(
        RobotStatus(7, Team.ALLY, 10.0, hp=400.0, max_hp=400.0, source=SOURCE_REFEREE), 10.0
    )
    assert model.snapshot(10.0).robots[7].last_hit_stamp is None

    # Self HP drops without an explicit 0x0206 last_hit_stamp -> synthesized at status.stamp!
    assert model.update_robot(
        RobotStatus(7, Team.ALLY, 10.4, hp=360.0, max_hp=400.0, source=SOURCE_REFEREE), 10.4
    )
    snap_hit = model.snapshot(10.4)
    assert snap_hit.robots[7].last_hit_stamp == pytest.approx(10.4)

    # 3.0 s later (> robot_status_timeout=2.0s, <= spin_hold_sec=3.5s <= hit_timeout=5.0s),
    # regular 10 Hz HP packets keep arriving without last_hit_stamp: hit memory stays alive!
    assert model.update_robot(
        RobotStatus(7, Team.ALLY, 13.2, hp=360.0, max_hp=400.0, source=SOURCE_REFEREE), 13.2
    )
    view_active = view_inputs(model.snapshot(13.2), config)
    assert view_active.self_last_hit_stamp == pytest.approx(10.4)

    # Past hit_timeout (5.0 s), a bridge repeating sticky last_hit_stamp=10.4
    # must NOT reject fresh HP!
    assert model.update_robot(
        RobotStatus(
            7,
            Team.ALLY,
            16.0,
            hp=360.0,
            max_hp=400.0,
            last_hit_stamp=10.4,
            hit_armor_id=2,
            posture="defense",
            posture_enhanced=False,
            posture_ready_at=14.0,
            posture_time_sec=(12.0, 30.0, 45.0),
            enhanced_used_sec=(0.0, 3.5, 0.0),
            source=SOURCE_REFEREE,
        ),
        16.0,
    )
    view_expired = view_inputs(model.snapshot(16.0), config)
    assert view_expired.self_hp == pytest.approx(0.9)
    assert view_expired.self_last_hit_stamp is None
    assert view_expired.self_posture == "defense"
    assert view_expired.self_posture_time_sec == (12.0, 30.0, 45.0)
    assert view_expired.self_enhanced_used_sec == (0.0, 3.5, 0.0)

    # Invalid 2027 fields and config invariants are rejected cleanly.
    from sentry_core.config import validate_config
    from sentry_core.model import GameState

    assert not model.update_robot(
        RobotStatus(7, Team.ALLY, 16.2, posture="invalid", source=SOURCE_REFEREE), 16.2
    )
    assert not model.update_robot(
        RobotStatus(7, Team.ALLY, 16.2, hit_armor_id=6, source=SOURCE_REFEREE), 16.2
    )
    assert not model.update_game(GameState(16.2, own_gold=-1.0, source=SOURCE_REFEREE), 16.2)
    with pytest.raises(ValueError, match="hit_timeout"):
        validate_config(replace(config, hit_timeout=2.0, spin_hold_sec=3.5))


def test_spin_policy_triggers_flips_stops_in_transit_and_never_spins_when_unsafe(config):
    from sentry_core.model import Phase, RobotStatus, TaskType, Team
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="spin-test")

    # 1. Calm far transit (> 3.5 m to goal, not hit, no enemy within 7.0 m) -> spin OFF.
    _, calm_world = make_running_world(config, now=10.0, self_pos=(10.0, 6.0))
    calm = engine.step(calm_world)
    assert calm.task.kind != TaskType.HOLD
    assert calm.task.spin_enabled is False and calm.task.spin_rate_rad_s == 0.0

    # 2. Armor hit (0x0206) -> immediate spin ON at +7.0 rad/s.
    hit_status = RobotStatus(
        7,
        Team.ALLY,
        10.5,
        hp=380.0,
        max_hp=400.0,
        ammo=0.8,
        last_hit_stamp=10.5,
        hit_armor_id=1,
        source=1,
    )
    _, hit_world = make_running_world(
        config, now=10.5, self_pos=(10.0, 6.0), self_status=hit_status
    )
    d1 = engine.step(hit_world)
    assert d1.task.spin_enabled is True
    assert d1.task.spin_rate_rad_s == pytest.approx(config.spin_speed_rad_s)

    # 3. New hit >= 0.6 s later flips spin sign immediately to defeat predictive auto-aim!
    hit2_status = replace(hit_status, stamp=11.3, hp=360.0, last_hit_stamp=11.3)
    _, hit2_world = make_running_world(
        config, now=11.3, self_pos=(10.0, 6.0), self_status=hit2_status
    )
    d2 = engine.step(hit2_world)
    assert d2.task.spin_enabled is True
    assert d2.task.spin_rate_rad_s == pytest.approx(-config.spin_speed_rad_s)

    # 4. Periodic flip after spin_flip_sec (2.2 s) while enemy stays within 7.0 m combat range.
    _, combat_world = make_running_world(
        config,
        now=13.6,
        self_pos=(4.0, 6.0),
        enemies=((103, 8.5, 6.0, -0.2, 0.0),),
        self_status=replace(hit2_status, stamp=13.6),
    )
    d3 = engine.step(combat_world)
    assert d3.task.spin_enabled is True
    assert d3.task.spin_rate_rad_s == pytest.approx(config.spin_speed_rad_s)

    # 5. Safety HOLD (GAME_NOT_RUNNING / LOCALIZATION_INVALID / ROBOT_DISABLED) -> NEVER spin!
    _, ready_world = make_running_world(
        config,
        now=14.0,
        self_pos=(4.0, 6.0),
        enemies=((103, 5.0, 6.0, 0.0, 0.0),),
        self_status=replace(hit2_status, stamp=14.0, last_hit_stamp=14.0),
        phase=Phase.READY,
    )
    unsafe_dec = engine.step(ready_world)
    assert unsafe_dec.task.kind == TaskType.HOLD
    assert unsafe_dec.task.spin_enabled is False and unsafe_dec.task.spin_rate_rad_s == 0.0
    assert unsafe_dec.task.posture_enhanced is False


def test_posture_policy_respects_5s_cooldown_180s_decay_and_15s_enhanced_caps(config):
    from sentry_core.model import RobotStatus, Team
    from sentry_core.policy import DecisionEngine

    engine = DecisionEngine(config, session_id="posture-test")

    # Match start in calm transit -> default "move", not enhanced.
    _, start_world = make_running_world(config, now=10.0, self_pos=(12.0, 6.0))
    d0 = engine.step(start_world)
    assert d0.task.posture == "move" and d0.task.posture_enhanced is False

    # High heat (0.85) + enemy in range -> switches to "attack" + enhanced (0 heat per shot!),
    # starting a 5.0 s switch cooldown.
    hot_status = RobotStatus(
        7,
        Team.ALLY,
        10.5,
        hp=380.0,
        max_hp=400.0,
        ammo=0.8,
        heat_ratio=0.85,
        posture="move",
        posture_enhanced=False,
        posture_ready_at=0.0,
        posture_time_sec=(10.0, 5.0, 20.0),
        enhanced_used_sec=(2.0, 0.0, 0.0),
        source=1,
    )
    _, hot_world = make_running_world(
        config,
        now=10.5,
        self_pos=(4.0, 5.0),
        enemies=((103, 7.5, 5.5, -0.5, 0.0),),
        self_status=hot_status,
    )
    d_attack = engine.step(hot_world)
    assert d_attack.task.posture == "attack" and d_attack.task.posture_enhanced is True

    # During 5.0 s cooldown (posture_ready_at=15.5, now=12.0), even if low HP hit arrives,
    # referee cooldown locks the posture at "attack".
    locked_status = RobotStatus(
        7,
        Team.ALLY,
        12.0,
        hp=80.0,
        max_hp=400.0,
        ammo=0.8,
        heat_ratio=0.2,
        last_hit_stamp=12.0,
        posture="attack",
        posture_enhanced=True,
        posture_ready_at=15.5,
        posture_time_sec=(11.5, 5.0, 20.0),
        enhanced_used_sec=(3.5, 0.0, 0.0),
        source=1,
    )
    _, locked_world = make_running_world(
        config, now=12.0, self_pos=(4.0, 5.0), self_status=locked_status
    )
    d_locked = engine.step(locked_world)
    assert d_locked.task.posture == "attack"
    assert d_locked.policy_info["posture"]["reason"] == "COOLDOWN_LOCKED"

    # Cooldown ready (now=16.0), low HP under fire -> switches to "defense" + enhanced (99% DR),
    # UNLESS defense enhanced 15.0 s cap is exhausted!
    ready_capped = replace(
        locked_status,
        stamp=16.0,
        last_hit_stamp=15.8,
        posture_ready_at=15.5,
        enhanced_used_sec=(5.0, 15.0, 0.0),
    )
    _, capped_world = make_running_world(
        config, now=16.0, self_pos=(4.0, 5.0), self_status=ready_capped
    )
    d_def = engine.step(capped_world)
    assert d_def.task.posture == "defense"
    assert d_def.task.posture_enhanced is False  # 15s defense enhanced cap respected!

    # 180s decay rotation: when "attack" is decayed (>180s) and taking fire in combat while
    # "defense" is still fresh (<180s), rotates into "defense".
    decayed_att = RobotStatus(
        7,
        Team.ALLY,
        22.0,
        hp=320.0,
        max_hp=400.0,
        ammo=0.8,
        heat_ratio=0.3,
        last_hit_stamp=21.8,
        posture="attack",
        posture_enhanced=False,
        posture_ready_at=20.0,
        posture_time_sec=(185.0, 40.0, 90.0),
        enhanced_used_sec=(15.0, 15.0, 0.0),
        source=1,
    )
    _, decayed_world = make_running_world(
        config,
        now=22.0,
        self_pos=(4.0, 5.0),
        enemies=((103, 7.5, 5.5, 0.0, 0.0),),
        self_status=decayed_att,
    )
    assert engine.step(decayed_world).task.posture == "defense"


def test_executor_allows_posture_and_spin_updates_on_task_lease_renewal(config):
    from sentry_core.execution import Executor
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Task, TaskType

    arena = TacticalMap(config)
    goal = config.points[0].position
    machine = Executor(arena, check_path_before_navigate=False)
    machine.safety(True, 10.0, 100.0)
    t1 = Task(
        "s1", 1, 1, 10.0, 12.0, TaskType.DEFEND, config.points[0].zone, None, "p0", goal, 0.0,
        0.7, 0.9, False, "AREA_UNCOVERED", posture="move", posture_enhanced=False,
        spin_enabled=False, spin_rate_rad_s=0.0,
    )
    machine.submit(t1, 10.0, 100.0)
    machine.drain()
    machine.accepted(machine.token, True, 10.1, 100.1)
    machine.drain()
    assert machine.state == "RUNNING"

    # Renewing same (session_id, task_id) with updated posture and spin rate must NOT fail!
    t1_renewed = replace(
        t1, world_seq=2, valid_until=12.5, posture="attack", posture_enhanced=True,
        spin_enabled=True, spin_rate_rad_s=-7.0,
    )
    machine.submit(t1_renewed, 10.5, 100.5)
    assert machine.drain() == []
    assert machine.state == "RUNNING"
    assert machine.current.posture == "attack" and machine.current.spin_rate_rad_s == -7.0

    # Invalid posture or boolean spin_rate_rad_s is rejected by _valid_task.
    machine.submit(replace(t1_renewed, task_id=2, posture="fly"), 10.6, 100.6)
    cmds = machine.drain()
    assert cmds and cmds[0].state == "REJECTED" and cmds[0].reason == "INVALID_OR_EXPIRED"


def test_serde_round_trips_2027_game_and_robot_posture_tuples(config):
    from sentry_core.model import RobotStatus, Team
    from sentry_core.serde import json_line, world_from_dict, world_to_dict

    status = RobotStatus(
        7, Team.ALLY, 10.0, hp=350.0, max_hp=400.0, alive=True, ammo=0.75, source=1,
        heat_ratio=0.4, last_hit_stamp=9.8, hit_armor_id=3, posture="attack",
        posture_enhanced=True, posture_ready_at=13.5, posture_time_sec=(25.5, 10.0, 60.0),
        enhanced_used_sec=(4.5, 0.0, 1.0),
    )
    _, world = make_running_world(
        config, now=10.0, self_status=status, own_gold=320.0, enemy_gold=180.0
    )
    restored = world_from_dict(json.loads(json_line(world_to_dict(world))))
    assert restored == world
