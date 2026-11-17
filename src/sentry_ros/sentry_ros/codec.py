"""Lossless typed transport adapters. No tactical logic belongs in this file."""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from sentry_interfaces import msg as wire

from sentry_core.model import (
    POSTURES, POSTURE_BY_CODE, POSTURE_CODES, GameState, IntentType, Observation, Phase, Referee,
    RobotStatus, StructureState, Task, TaskType, Team, Track, TrackStatus, Vec2, World,
)


FEEDBACK_STATES = ("IDLE", "ACCEPTED", "RUNNING", "CANCELING",
                   "SUCCEEDED", "FAILED", "CANCELED", "REJECTED")


def stamp(value: float) -> Time:
    if not math.isfinite(value) or value < 0:
        raise ValueError("invalid timestamp")
    sec, nanosec = divmod(round(value * 1_000_000_000), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def to_seconds(value: Time) -> float:
    return value.sec + value.nanosec / 1e9


def yaw(orientation) -> float:
    q = orientation
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v * v for v in values) - 1) > 0.02:
        raise ValueError("invalid pose quaternion")
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose(position: Vec2, angle: float, frame: str, now: float) -> PoseStamped:
    result = PoseStamped()
    result.header.frame_id, result.header.stamp = frame, stamp(now)
    result.pose.position.x, result.pose.position.y = float(position.x), float(position.y)
    result.pose.orientation.z, result.pose.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
    return result


def observation_to_msg(observation: Observation, frame: str):
    result = wire.RobotObservation()
    result.header.frame_id, result.header.stamp = frame, stamp(observation.stamp)
    result.robot_id, result.team = observation.robot_id, int(observation.team)
    result.pose.pose = pose(observation.position, 0., frame, observation.stamp).pose
    result.confidence, result.source_mask = float(observation.confidence), observation.source_mask
    result.velocity_valid = observation.velocity is not None
    if observation.velocity is not None:
        result.twist.twist.linear.x = float(observation.velocity.x)
        result.twist.twist.linear.y = float(observation.velocity.y)
    return result


def observation_from_msg(message) -> Observation:
    return Observation(
        message.robot_id, Team(message.team),
        Vec2(message.pose.pose.position.x, message.pose.pose.position.y),
        to_seconds(message.header.stamp), message.confidence,
        Vec2(message.twist.twist.linear.x, message.twist.twist.linear.y)
        if message.velocity_valid else None, message.source_mask,
    )


def track_to_msg(track: Track, frame: str):
    result = wire.RobotTrack()
    result.header.frame_id, result.header.stamp = frame, stamp(track.observed_at)
    result.robot_id, result.team = track.robot_id, int(track.team)
    result.pose.pose = pose(track.position, 0., frame, track.observed_at).pose
    result.twist.twist.linear.x, result.twist.twist.linear.y = float(track.velocity.x), float(track.velocity.y)
    result.velocity_valid, result.status = track.velocity_valid, int(track.status)
    result.confidence, result.age = float(track.confidence), float(track.age)
    result.generation = track.generation
    result.history = [observation_to_msg(o, frame) for o in track.history]
    return result


def track_from_msg(message, frame: str):
    if message.header.frame_id != frame or any(o.header.frame_id != frame for o in message.history):
        raise ValueError("track/history frame differs from world")
    result = Track(
        message.robot_id, Team(message.team),
        Vec2(message.pose.pose.position.x, message.pose.pose.position.y),
        Vec2(message.twist.twist.linear.x, message.twist.twist.linear.y),
        to_seconds(message.header.stamp), message.age, message.confidence,
        TrackStatus(message.status), message.generation, message.velocity_valid,
        tuple(observation_from_msg(o) for o in message.history),
    )
    if (not result.position.finite or not result.velocity.finite
            or not math.isfinite(result.age) or result.age < 0
            or not 0 <= result.confidence <= 1):
        raise ValueError("invalid world track values")
    return result


def referee_to_msg(referee: Referee, frame="map"):
    result = wire.RefereeState()
    result.header.frame_id, result.header.stamp = frame, stamp(referee.stamp)
    result.phase, result.phase_valid = int(referee.phase), referee.phase != Phase.UNKNOWN
    for source, target, valid in (
        ("hp", "hp_ratio", "hp_valid"), ("ammo", "ammo_ratio", "ammo_valid"),
        ("own_outpost", "own_outpost_alive", "own_outpost_valid"),
        ("enemy_outpost", "enemy_outpost_alive", "enemy_outpost_valid"),
        ("supply_available", "supply_available", "supply_valid"),
        ("remaining", "remaining_time", "remaining_time_valid"),
    ):
        value = getattr(referee, source)
        setattr(result, valid, value is not None)
        if value is not None:
            setattr(result, target, value)
    return result


def referee_from_msg(message):
    referee = Referee(
        to_seconds(message.header.stamp), Phase(message.phase) if message.phase_valid else Phase.UNKNOWN,
        message.hp_ratio if message.hp_valid else None,
        message.ammo_ratio if message.ammo_valid else None,
        message.own_outpost_alive if message.own_outpost_valid else None,
        message.enemy_outpost_alive if message.enemy_outpost_valid else None,
        message.supply_available if message.supply_valid else None,
        message.remaining_time if message.remaining_time_valid else None,
    )
    if any(v is not None and (not math.isfinite(v) or not 0 <= v <= 1)
           for v in (referee.hp, referee.ammo)):
        raise ValueError("invalid referee resource fraction")
    return referee


def game_to_msg(game: GameState, frame="map"):
    result = wire.GameStatus()
    result.header.frame_id, result.header.stamp = frame, stamp(game.stamp)
    result.season = game.season
    result.own_team = {"RED": wire.GameStatus.RED, "BLUE": wire.GameStatus.BLUE}.get(
        game.own_team, wire.GameStatus.TEAM_UNKNOWN)
    result.phase, result.phase_valid = int(game.phase), game.phase != Phase.UNKNOWN
    result.remaining_time_valid = game.remaining is not None
    if game.remaining is not None:
        result.remaining_time = float(game.remaining)
    result.supply_valid = game.supply_available is not None
    if game.supply_available is not None:
        result.supply_available = game.supply_available
    result.source = game.source
    result.own_gold_valid = game.own_gold is not None
    if game.own_gold is not None:
        result.own_gold = float(game.own_gold)
    result.enemy_gold_valid = game.enemy_gold is not None
    if game.enemy_gold is not None:
        result.enemy_gold = float(game.enemy_gold)
    return result


def game_from_msg(message) -> GameState:
    if message.own_team not in (wire.GameStatus.TEAM_UNKNOWN, wire.GameStatus.RED,
                                wire.GameStatus.BLUE):
        raise ValueError("invalid own_team")
    return GameState(
        to_seconds(message.header.stamp), message.season,
        {wire.GameStatus.RED: "RED", wire.GameStatus.BLUE: "BLUE"}.get(message.own_team),
        Phase(message.phase) if message.phase_valid else Phase.UNKNOWN,
        message.remaining_time if message.remaining_time_valid else None,
        message.supply_available if message.supply_valid else None, message.source,
        own_gold=message.own_gold if message.own_gold_valid else None,
        enemy_gold=message.enemy_gold if message.enemy_gold_valid else None,
    )


def robot_status_to_msg(status: RobotStatus):
    result = wire.RobotStatus()
    result.stamp = stamp(status.stamp)
    result.robot_id, result.team = status.robot_id, int(status.team)
    # hp and max_hp are valid independently: a known hp is kept without max_hp.
    result.hp_valid = status.hp is not None
    if status.hp is not None:
        result.hp = float(status.hp)
    result.max_hp_valid = status.max_hp is not None
    if status.max_hp is not None:
        result.max_hp = float(status.max_hp)
    result.alive_valid = status.alive is not None
    if status.alive is not None:
        result.alive = status.alive
    result.level_valid = status.level is not None
    if status.level is not None:
        result.level = status.level
    result.ammo_valid = status.ammo is not None
    if status.ammo is not None:
        result.ammo_ratio = float(status.ammo)
    result.source = status.source
    result.heat_valid = status.heat_ratio is not None
    if status.heat_ratio is not None:
        result.heat_ratio = float(status.heat_ratio)
    result.last_hit_valid = status.last_hit_stamp is not None
    if status.last_hit_stamp is not None:
        result.last_hit_stamp = stamp(status.last_hit_stamp)
    result.hit_armor_valid = status.hit_armor_id is not None
    if status.hit_armor_id is not None:
        result.hit_armor_id = int(status.hit_armor_id)
    result.posture_valid = status.posture is not None
    if status.posture is not None:
        if status.posture not in POSTURE_CODES:
            raise ValueError(f"invalid sentry posture {status.posture!r}")
        result.posture = POSTURE_CODES[status.posture]
    result.posture_enhanced_valid = status.posture_enhanced is not None
    if status.posture_enhanced is not None:
        result.posture_enhanced = bool(status.posture_enhanced)
    result.posture_ready_valid = status.posture_ready_at is not None
    if status.posture_ready_at is not None:
        result.posture_ready_at = float(status.posture_ready_at)
    result.posture_time_valid = status.posture_time_sec is not None
    if status.posture_time_sec is not None:
        if len(status.posture_time_sec) != 3:
            raise ValueError("posture_time_sec must have 3 elements")
        result.posture_time_sec = [float(x) for x in status.posture_time_sec]
    result.enhanced_used_valid = status.enhanced_used_sec is not None
    if status.enhanced_used_sec is not None:
        if len(status.enhanced_used_sec) != 3:
            raise ValueError("enhanced_used_sec must have 3 elements")
        result.enhanced_used_sec = [float(x) for x in status.enhanced_used_sec]
    return result


def robot_status_from_msg(message) -> RobotStatus:
    if (message.posture != wire.RobotStatus.POSTURE_UNKNOWN
            and message.posture not in POSTURE_BY_CODE):
        raise ValueError(f"invalid sentry posture code {message.posture}")
    posture = POSTURE_BY_CODE.get(message.posture) if message.posture_valid else None
    posture_time = None
    if message.posture_time_valid:
        if len(message.posture_time_sec) != 3:
            raise ValueError("posture_time_sec must have 3 elements")
        posture_time = tuple(float(x) for x in message.posture_time_sec)
    enhanced_used = None
    if message.enhanced_used_valid:
        if len(message.enhanced_used_sec) != 3:
            raise ValueError("enhanced_used_sec must have 3 elements")
        enhanced_used = tuple(float(x) for x in message.enhanced_used_sec)
    return RobotStatus(
        message.robot_id, Team(message.team), to_seconds(message.stamp),
        message.hp if message.hp_valid else None,
        message.max_hp if message.max_hp_valid else None,
        message.alive if message.alive_valid else None,
        message.level if message.level_valid else None,
        message.ammo_ratio if message.ammo_valid else None, message.source,
        heat_ratio=message.heat_ratio if message.heat_valid else None,
        last_hit_stamp=to_seconds(message.last_hit_stamp) if message.last_hit_valid else None,
        hit_armor_id=int(message.hit_armor_id) if message.hit_armor_valid else None,
        posture=posture,
        posture_enhanced=(
            bool(message.posture_enhanced) if message.posture_enhanced_valid else None
        ),
        posture_ready_at=(
            float(message.posture_ready_at) if message.posture_ready_valid else None
        ),
        posture_time_sec=posture_time,
        enhanced_used_sec=enhanced_used,
    )


def structure_to_msg(state: StructureState):
    result = wire.StructureState()
    result.stamp, result.id = stamp(state.stamp), state.name
    result.hp_valid = state.hp is not None and state.max_hp is not None
    if result.hp_valid:
        result.hp, result.max_hp = float(state.hp), float(state.max_hp)
    result.alive_valid = state.alive is not None
    if state.alive is not None:
        result.alive = state.alive
    result.shield_valid = state.shield is not None
    if state.shield is not None:
        result.shield = float(state.shield)
    result.source = state.source
    return result


def structure_from_msg(message) -> StructureState:
    return StructureState(
        message.id, to_seconds(message.stamp),
        message.hp if message.hp_valid else None, message.max_hp if message.hp_valid else None,
        message.alive if message.alive_valid else None,
        message.shield if message.shield_valid else None, message.source,
    )


def world_to_msg(world: World):
    result = wire.WorldState()
    result.header.frame_id, result.header.stamp = world.frame, stamp(world.stamp)
    result.world_seq, result.epoch, result.mode = world.seq, world.epoch, world.mode
    result.self_valid = world.self_track is not None
    if world.self_track:
        result.self_robot = track_to_msg(world.self_track, world.frame)
    result.allies = [track_to_msg(t, world.frame) for t in world.allies]
    result.enemies = [track_to_msg(t, world.frame) for t in world.enemies]
    result.referee_valid = world.referee is not None
    if world.referee:
        result.referee = referee_to_msg(world.referee, world.frame)
    result.game_valid = world.game is not None
    if world.game:
        result.game = game_to_msg(world.game, world.frame)
    result.robots = [robot_status_to_msg(s) for s in world.robots.values()]
    result.structures = [structure_to_msg(s) for s in world.structures.values()]
    return result


def world_from_msg(message):
    frame = message.header.frame_id
    return World(
        message.world_seq, to_seconds(message.header.stamp),
        track_from_msg(message.self_robot, frame) if message.self_valid else None,
        tuple(track_from_msg(t, frame) for t in message.allies),
        tuple(track_from_msg(t, frame) for t in message.enemies),
        referee_from_msg(message.referee) if message.referee_valid else None,
        message.mode, frame, message.epoch,
        game_from_msg(message.game) if message.game_valid else None,
        {s.robot_id: s for s in map(robot_status_from_msg, message.robots)},
        {s.name: s for s in map(structure_from_msg, message.structures)},
    )


def task_to_msg(task: Task, frame: str, now: float):
    if task.posture not in POSTURES:
        raise ValueError(f"invalid sentry posture {task.posture!r}")
    result = wire.TacticalTask()
    result.header.frame_id, result.header.stamp = frame, stamp(now)
    result.session_id, result.task_id = task.session_id, task.task_id
    result.based_on_world_seq, result.task_type = task.world_seq, int(task.kind)
    result.target_zone_id, result.goal_id = task.zone_id, task.goal_id
    result.target_robot_valid = task.target_id is not None
    result.target_robot_id = task.target_id or 0
    result.target_pose_valid = task.goal is not None
    if task.goal is not None:
        result.target_pose = pose(task.goal, task.yaw, frame, now)
    result.utility, result.confidence, result.urgent = task.utility, task.confidence, task.urgent
    result.created_at, result.valid_until = stamp(task.created_at), stamp(task.valid_until)
    result.reason_code = task.reason
    result.posture = task.posture
    result.posture_enhanced = bool(task.posture_enhanced)
    result.spin_enabled = bool(task.spin_enabled)
    result.spin_rate_rad_s = float(task.spin_rate_rad_s)
    return result


def task_from_msg(message, frame: str):
    if (message.header.frame_id != frame
            or message.target_pose_valid and message.target_pose.header.frame_id != frame):
        raise ValueError("task frame mismatch")
    posture = message.posture or "move"
    if posture not in POSTURES:
        raise ValueError(f"invalid sentry posture {message.posture!r}")
    p = message.target_pose.pose
    return Task(
        message.session_id, message.task_id, message.based_on_world_seq,
        to_seconds(message.created_at), to_seconds(message.valid_until),
        TaskType(message.task_type),
        message.target_zone_id, message.target_robot_id if message.target_robot_valid else None,
        message.goal_id, Vec2(p.position.x, p.position.y) if message.target_pose_valid else None,
        yaw(p.orientation) if message.target_pose_valid else 0.,
        message.utility, message.confidence, message.urgent, message.reason_code,
        posture=posture,
        posture_enhanced=bool(message.posture_enhanced),
        spin_enabled=bool(message.spin_enabled),
        spin_rate_rad_s=float(message.spin_rate_rad_s),
    )


def sentry_referee_cmd_to_msg(task: Task, frame: str, now: float):
    if task.posture not in POSTURE_CODES:
        raise ValueError(f"invalid sentry posture {task.posture!r}")
    code = POSTURE_CODES[task.posture]
    result = wire.SentryRefereeCmd()
    result.header.frame_id, result.header.stamp = frame, stamp(now)
    result.posture_code = code
    result.posture = task.posture
    result.posture_enhanced = bool(task.posture_enhanced)
    result.raw_sentry_cmd = (code & 0x3) << 21
    result.based_on_task_id = task.task_id
    result.reason_code = task.reason
    return result


def intents_to_msg(intents, frame, now):
    result = wire.TeammateIntentArray()
    result.header.frame_id, result.header.stamp = frame, stamp(now)
    result.intents = [wire.TeammateIntent(
        robot_id=i.robot_id, intent_type=int(IntentType(i.kind)), zone_id=i.zone_id,
        probabilities=list(i.probabilities), confidence=i.confidence, valid=i.valid,
    ) for i in intents]
    return result


def threats_to_msg(threats, frame, now):
    result = wire.ThreatState()
    result.header.frame_id, result.header.stamp = frame, stamp(now)
    result.zones = [wire.ZoneThreat(
        zone_id=t.zone_id, score=t.score, enemy_id=t.enemy_id or 0, enemy_valid=t.enemy_id is not None,
        eta_sec=t.eta or 0., eta_valid=t.eta is not None,
    ) for t in threats]
    return result
