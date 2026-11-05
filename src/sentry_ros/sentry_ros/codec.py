"""Lossless typed transport adapters. No tactical logic belongs in this file."""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from sentry_interfaces import msg as wire

from sentry_core.model import (
    IntentType, Observation, Phase, Referee, Task, TaskType, Team, Track, TrackStatus, Vec2, World,
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
    )


def task_to_msg(task: Task, frame: str, now: float):
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
    return result


def task_from_msg(message, frame: str):
    if (message.header.frame_id != frame
            or message.target_pose_valid and message.target_pose.header.frame_id != frame):
        raise ValueError("task frame mismatch")
    p = message.target_pose.pose
    return Task(
        message.session_id, message.task_id, message.based_on_world_seq,
        to_seconds(message.created_at), to_seconds(message.valid_until), TaskType(message.task_type),
        message.target_zone_id, message.target_robot_id if message.target_robot_valid else None,
        message.goal_id, Vec2(p.position.x, p.position.y) if message.target_pose_valid else None,
        yaw(p.orientation) if message.target_pose_valid else 0.,
        message.utility, message.confidence, message.urgent, message.reason_code,
    )


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
