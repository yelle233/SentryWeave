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
