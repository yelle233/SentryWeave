"""Value objects shared by online decisions, training and replay.

Times are seconds in one observation clock. Executor deadlines additionally use
a separate monotonic clock. Optional referee, game, robot and structure fields
mean unknown, never zero and never full.
"""

from dataclasses import dataclass, field
from enum import IntEnum
import math


class Team(IntEnum):
    UNKNOWN = 0
    ALLY = 1
    ENEMY = 2


class TrackStatus(IntEnum):
    UNKNOWN = 0
    OBSERVED = 1
    PREDICTED = 2
    STALE = 3


class Phase(IntEnum):
    UNKNOWN = 0
    READY = 1
    RUNNING = 2
    FINISHED = 3


class IntentType(IntEnum):
    UNKNOWN = 0
    ATTACK_OUTPOST = 1
    DEFEND_BASE = 2
    CONTEST_AREA = 3
    RETREAT = 4
    TRANSIT = 5


class TaskType(IntEnum):
    HOLD = 0
    SUPPORT = 1
    INTERCEPT = 2
    DEFEND = 3
    RECOVER = 4


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def distance(self, other: "Vec2") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    @property
    def finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y)

    @property
    def norm(self) -> float:
        return math.hypot(self.x, self.y)


@dataclass(frozen=True)
class Observation:
    robot_id: int
    team: Team
    position: Vec2
    stamp: float
    confidence: float = 1.0
    velocity: Vec2 | None = None
    source_mask: int = 1


@dataclass(frozen=True)
class Track:
    robot_id: int
    team: Team
    position: Vec2
    velocity: Vec2
    observed_at: float
    age: float
    confidence: float
    status: TrackStatus
    generation: int
    velocity_valid: bool
    history: tuple[Observation, ...] = ()

    @property
    def usable(self) -> bool:
        return (self.status in (TrackStatus.OBSERVED, TrackStatus.PREDICTED)
                and self.confidence > 0.05 and self.position.finite)


@dataclass(frozen=True)
class Referee:
    stamp: float
    phase: Phase = Phase.UNKNOWN
    hp: float | None = None
    ammo: float | None = None
    own_outpost: bool | None = None
    enemy_outpost: bool | None = None
    supply_available: bool | None = None
    remaining: float | None = None


# Source bits shared by GameStatus, RobotStatus and StructureState messages.
SOURCE_REFEREE = 1
SOURCE_RADAR = 2
SOURCE_ONBOARD = 4
SOURCE_TEAM = 8
TEAM_COLORS = ("RED", "BLUE")
STRUCTURE_IDS = ("own_base", "own_outpost", "enemy_base", "enemy_outpost")


def _ratio(value: float | None, maximum: float | None) -> float | None:
    if value is None or maximum is None or maximum <= 0:
        return None
    return min(1.0, max(0.0, value / maximum))


@dataclass(frozen=True)
class GameState:
    """Game-level referee information. None or Phase.UNKNOWN means unknown."""

    stamp: float
    season: str = ""
    own_team: str | None = None
    phase: Phase = Phase.UNKNOWN
    remaining: float | None = None
    supply_available: bool | None = None
    source: int = SOURCE_REFEREE


@dataclass(frozen=True)
class RobotStatus:
    """One robot's status; ``team`` is relative to the own team, like tracks."""

    robot_id: int
    team: Team
    stamp: float
    hp: float | None = None
    max_hp: float | None = None
    alive: bool | None = None
    level: int | None = None
    ammo: float | None = None
    source: int = 0

    @property
    def hp_ratio(self) -> float | None:
        return _ratio(self.hp, self.max_hp)

    @property
    def known_alive(self) -> bool | None:
        if self.alive is not None:
            return self.alive
        return None if self.hp is None else self.hp > 0


@dataclass(frozen=True)
class StructureState:
    """Base, outpost or field-defined structure; unknown values stay None."""

    name: str
    stamp: float
    hp: float | None = None
    max_hp: float | None = None
    alive: bool | None = None
    shield: float | None = None
    source: int = 0

    @property
    def hp_ratio(self) -> float | None:
        return _ratio(self.hp, self.max_hp)

    @property
    def known_alive(self) -> bool | None:
        if self.alive is not None:
            return self.alive
        return None if self.hp is None else self.hp > 0


@dataclass(frozen=True)
class World:
    seq: int
    stamp: float
    self_track: Track | None
    allies: tuple[Track, ...] = ()
    enemies: tuple[Track, ...] = ()
    referee: Referee | None = None
    mode: str = "SAFE_STOP"
    frame: str = "map"
    epoch: int = 0
    game: GameState | None = None
    robots: dict[int, RobotStatus] = field(default_factory=dict)
    structures: dict[str, StructureState] = field(default_factory=dict)


@dataclass(frozen=True)
class Intent:
    robot_id: int
    kind: IntentType
    probabilities: tuple[float, ...]
    confidence: float
    zone_id: str = ""
    valid: bool = False
    features: tuple[float, ...] = ()


@dataclass(frozen=True)
class Threat:
    zone_id: str
    score: float
    enemy_id: int | None
    eta: float | None


@dataclass(frozen=True)
class Candidate:
    kind: TaskType
    zone_id: str
    target_id: int | None
    goal_id: str
    goal: Vec2 | None
    yaw: float
    features: tuple[float, ...]
    utility: float
    confidence: float
    urgent: bool
    reason: str
    rule_utility: float | None = None
    learned_utility: float | None = None

    @property
    def key(self) -> tuple:
        return self.kind, self.zone_id, self.target_id, self.goal_id


@dataclass(frozen=True)
class Task:
    session_id: str
    task_id: int
    world_seq: int
    created_at: float
    valid_until: float
    kind: TaskType
    zone_id: str = ""
    target_id: int | None = None
    goal_id: str = ""
    goal: Vec2 | None = None
    yaw: float = 0.0
    utility: float = 0.0
    confidence: float = 0.0
    urgent: bool = False
    reason: str = ""

    @property
    def key(self) -> tuple:
        return self.kind, self.zone_id, self.target_id, self.goal_id

    @property
    def identity(self) -> tuple[str, int]:
        return self.session_id, self.task_id


@dataclass(frozen=True)
class Decision:
    task: Task
    intents: tuple[Intent, ...]
    threats: tuple[Threat, ...]
    candidates: tuple[Candidate, ...]
    rejected: tuple[str, ...]
    arbitration: str
    mode: str
    policy_info: dict = field(default_factory=dict)
