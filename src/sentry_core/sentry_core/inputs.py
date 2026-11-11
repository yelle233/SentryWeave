"""One merged view of game, robot and structure inputs for the decision core.

The typed interfaces (GameState, RobotStatus, StructureState) win over the
legacy Referee whenever they carry a valid, unexpired value. Invalid or expired
values stay None: unknown is never treated as zero or as full.
"""

from dataclasses import dataclass, field

from .model import STRUCTURE_IDS, Phase, RobotStatus, Team, World


@dataclass(frozen=True)
class InputView:
    phase: Phase = Phase.UNKNOWN
    remaining: float | None = None
    own_team: str | None = None
    supply_available: bool | None = None
    self_hp: float | None = None
    self_ammo: float | None = None
    self_alive: bool | None = None
    robots: dict[int, RobotStatus] = field(default_factory=dict)
    structures: dict[str, bool | None] = field(default_factory=dict)
    sources: dict[str, str] = field(default_factory=dict)

    def robot(self, team: Team, robot_id: int) -> RobotStatus | None:
        status = self.robots.get(robot_id)
        return status if status is not None and status.team == team else None


def _fresh(world: World, stamp: float, timeout: float) -> bool:
    return -0.05 <= world.stamp - stamp <= timeout


def view_inputs(world: World, config) -> InputView:
    game = world.game if world.game and _fresh(world, world.game.stamp, config.game_timeout) else None
    ref = (world.referee if world.referee and _fresh(world, world.referee.stamp, config.referee_timeout)
           else None)
    sources = {}

    def pick(name, new, legacy):
        if new is not None:
            sources[name] = "typed"
            return new
        if legacy is not None:
            sources[name] = "referee"
        return legacy

    phase = pick("phase", game.phase if game and game.phase != Phase.UNKNOWN else None,
                 ref.phase if ref and ref.phase != Phase.UNKNOWN else None)
    remaining = pick("remaining", game.remaining if game else None, ref.remaining if ref else None)
    supply = pick("supply_available", game.supply_available if game else None,
                  ref.supply_available if ref else None)
    robots = {robot_id: status for robot_id, status in world.robots.items()
              if _fresh(world, status.stamp, config.robot_status_timeout)}
    own = robots.get(config.self_id)
    own = own if own is not None and own.team == Team.ALLY else None
    self_hp = pick("self_hp", own.hp_ratio if own else None, ref.hp if ref else None)
    self_ammo = pick("self_ammo", own.ammo if own else None, ref.ammo if ref else None)
    states = {name: state for name, state in world.structures.items()
              if _fresh(world, state.stamp, config.structure_timeout)}
    legacy = {"own_outpost": ref.own_outpost if ref else None,
              "enemy_outpost": ref.enemy_outpost if ref else None}
    structures = {}
    for name in (*STRUCTURE_IDS, *(name for name in states if name not in STRUCTURE_IDS)):
        state = states.get(name)
        structures[name] = pick(f"structure.{name}", state.known_alive if state else None,
                                legacy.get(name))
    return InputView(phase or Phase.UNKNOWN, remaining, game.own_team if game else None, supply,
                     self_hp, self_ammo, own.known_alive if own else None, robots, structures,
                     sources)


# Categories the offline navigation model cannot mark as invalid (it was trained on
# logs with every robot localized). Low availability here lowers the RL blend.
RL_CATEGORIES = ("allies", "enemies")


@dataclass(frozen=True)
class InfoReport:
    completeness: dict[str, float]
    missing: tuple[str, ...]


def assess(world: World, config, view: InputView) -> InfoReport:
    """Per-category availability in [0, 1] and the missing items behind it."""
    missing = []
    if view.phase == Phase.UNKNOWN:
        missing.append("game.phase")
    if view.remaining is None:
        missing.append("game.remaining_time")
    if view.self_hp is None:
        missing.append("self.hp")
    if view.self_ammo is None:
        missing.append("self.ammo")
    completeness = {
        "game": 0.5 * (view.phase != Phase.UNKNOWN) + 0.5 * (view.remaining is not None),
        "self": 0.5 * (view.self_hp is not None) + 0.5 * (view.self_ammo is not None),
    }
    subjects = set()
    for team, name, tracks, expected in ((Team.ALLY, "allies", world.allies, config.expected_allies),
                                         (Team.ENEMY, "enemies", world.enemies,
                                          config.expected_enemies)):
        dead = {s.robot_id for s in view.robots.values() if s.team == team and s.known_alive is False}
        known = ({t.robot_id for t in tracks}
                 | {s.robot_id for s in view.robots.values() if s.team == team}) - dead
        if team == Team.ALLY:
            known.discard(config.self_id)
            dead.discard(config.self_id)
        usable = {t.robot_id for t in tracks if t.usable} - dead
        # Robots known to be down are not expected on the field until they respawn.
        total = max(expected - len(dead), len(known))
        completeness[name] = min(1.0, len(usable) / total) if total else 1.0
        if len(usable) < total:
            missing.append(f"{name}.positions {len(usable)}/{total}")
        missing += [f"{name}.{robot_id}.stale" for robot_id in sorted(known - usable)]
        subjects |= {(team, robot_id) for robot_id in known | dead}
    total = max(config.expected_allies + config.expected_enemies, len(subjects))
    rated = sum(1 for team, robot_id in subjects
                if (status := view.robot(team, robot_id)) and status.hp_ratio is not None)
    completeness["robot_status"] = min(1.0, rated / total) if total else 1.0
    if rated < total:
        missing.append(f"robots.hp {rated}/{total}")
    known_structures = [name for name, alive in view.structures.items() if alive is not None]
    completeness["structures"] = len(known_structures) / len(view.structures)
    missing += [f"structure.{name}" for name, alive in view.structures.items() if alive is None]
    return InfoReport(completeness, tuple(missing))
