"""Explicit replay format, versioned separately from ROS transport."""

from dataclasses import asdict
import json

from .model import (
    GameState, Observation, Phase, Referee, RobotStatus, StructureState, Team, Track, TrackStatus,
    Vec2, World,
)


def json_line(value) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def world_to_dict(world: World) -> dict:
    return asdict(world)


def world_from_dict(raw: dict) -> World:
    def observation(o):
        return Observation(o["robot_id"], Team(o["team"]), Vec2(**o["position"]), o["stamp"],
                           o["confidence"], Vec2(**o["velocity"]) if o["velocity"] else None,
                           o.get("source_mask", 1))

    def track(t):
        if t is None:
            return None
        return Track(t["robot_id"], Team(t["team"]), Vec2(**t["position"]), Vec2(**t["velocity"]),
                     t["observed_at"], t["age"], t["confidence"], TrackStatus(t["status"]),
                     t["generation"], t["velocity_valid"],
                     tuple(observation(o) for o in t.get("history", [])))

    referee = raw.get("referee")
    game = raw.get("game")
    return World(
        raw["seq"], raw["stamp"], track(raw.get("self_track")),
        tuple(track(t) for t in raw.get("allies", [])),
        tuple(track(t) for t in raw.get("enemies", [])),
        Referee(**{**referee, "phase": Phase(referee["phase"])}) if referee else None,
        raw["mode"], raw["frame"], raw.get("epoch", 0),
        GameState(**{**game, "phase": Phase(game["phase"])}) if game else None,
        {int(key): RobotStatus(**{**value, "team": Team(value["team"])})
         for key, value in raw.get("robots", {}).items()},
        {key: StructureState(**value) for key, value in raw.get("structures", {}).items()},
    )
