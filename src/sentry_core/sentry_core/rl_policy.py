"""NumPy inference for exported offline RL actors. No torch dependency on the robot."""

import json
import math
from dataclasses import replace
import hashlib

import numpy as np

from .inputs import view_inputs
from .model import TaskType, Vec2
from .navigation_features import FEATURE_NAMES, FEATURE_VERSION, NavigationSpec, encode_world


class NumpyNavigationPolicy:
    def __init__(self, payload):
        if (payload.get("artifact_type") != "sentry_navigation_policy"
                or payload.get("schema_version") != 1
                or payload.get("feature_version") != FEATURE_VERSION
                or payload.get("feature_names") != list(FEATURE_NAMES)):
            raise ValueError("incompatible navigation policy feature schema")
        if payload.get("algorithm") not in ("bc", "iql", "dt"):
            raise ValueError("unknown navigation algorithm")
        self.payload = payload
        self.spec = NavigationSpec(**payload["spec"]).validate()
        self.architecture = payload["architecture"]
        self.target_return = float(payload["target_return"])
        self.return_scale = float(payload["return_scale"])
        if not math.isfinite(self.target_return) or not math.isfinite(self.return_scale) or self.return_scale <= 0:
            raise ValueError("invalid return conditioning")
        self.parameters = {name: np.asarray(value, dtype=np.float32)
                           for name, value in payload["parameters"].items()}
        self._validate()

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as stream:
            return cls(json.load(stream))

    def _validate(self):
        architecture = self.architecture
        shapes = {"log_std": (2,)}

        def linear(name, input_size, output_size):
            shapes[name + ".weight"] = output_size, input_size
            shapes[name + ".bias"] = (output_size,)

        def norm(name, width):
            shapes[name + ".weight"] = shapes[name + ".bias"] = (width,)

        if architecture["type"] == "mlp":
            hidden = architecture["hidden"]
            if not isinstance(hidden, int) or not 8 <= hidden <= 512:
                raise ValueError("invalid MLP size")
            linear("layers.0", len(FEATURE_NAMES), hidden)
            linear("layers.2", hidden, hidden)
            linear("layers.4", hidden, 2)
        elif architecture["type"] == "dt":
            width, heads, depth = (architecture[k] for k in ("width", "heads", "layers"))
            context, timestep = architecture["context"], architecture["max_timestep"]
            if (not all(isinstance(v, int) for v in (width, heads, depth, context, timestep))
                    or not 8 <= width <= 512 or not 1 <= heads <= 16 or width % heads
                    or not 1 <= depth <= 6 or not 1 <= context <= 32 or not 1 <= timestep <= 4096):
                raise ValueError("invalid transformer size")
            for name, size in (("return_embed", 1), ("state_embed", len(FEATURE_NAMES)),
                               ("action_embed", 2)):
                linear(name, size, width)
            shapes["time_embed.weight"] = (timestep + 1, width)
            for layer in range(depth):
                prefix = f"blocks.{layer}"
                norm(prefix + ".norm1", width)
                norm(prefix + ".norm2", width)
                linear(prefix + ".attn.qkv", width, 3 * width)
                linear(prefix + ".attn.out", width, width)
                linear(prefix + ".fc1", width, 4 * width)
                linear(prefix + ".fc2", 4 * width, width)
            norm("norm_out", width)
            linear("head", width, 2)
        else:
            raise ValueError("unsupported portable architecture")
        if set(self.parameters) != set(shapes):
            raise ValueError("unexpected/missing model parameters")
        for name, shape in shapes.items():
            parameter = self.parameters[name]
            if parameter.shape != shape or not np.isfinite(parameter).all():
                raise ValueError(f"invalid model parameter {name}")

    def _linear(self, x, name):
        return x @ self.parameters[name + ".weight"].T + self.parameters[name + ".bias"]

    def _norm(self, x, name):
        mean = x.mean(axis=-1, keepdims=True)
        variance = ((x - mean) ** 2).mean(axis=-1, keepdims=True)
        return ((x - mean) / np.sqrt(variance + 1e-5)
                * self.parameters[name + ".weight"] + self.parameters[name + ".bias"])

    def predict(self, observations, actions=None, returns=None, times=None):
        observations = np.atleast_2d(np.asarray(observations, dtype=np.float32))
        if observations.shape[1] != len(FEATURE_NAMES) or not np.isfinite(observations).all():
            raise ValueError("invalid policy observation")
        if self.architecture["type"] == "mlp":
            x = np.maximum(0., self._linear(observations[-1], "layers.0"))
            x = np.maximum(0., self._linear(x, "layers.2"))
            return np.tanh(self._linear(x, "layers.4"))
        context = self.architecture["context"]
        x = observations[-context:]
        length = len(x)
        a = np.asarray(actions[-length:], dtype=np.float32) if actions is not None else np.zeros((length, 2), np.float32)
        r = (np.asarray(returns[-length:], dtype=np.float32) if returns is not None else
             np.full((length, 1), self.target_return / self.return_scale, dtype=np.float32))
        t = np.asarray(times[-length:], dtype=np.int64) if times is not None else np.arange(length)
        if (a.shape != (length, 2) or r.shape != (length, 1) or t.shape != (length,)
                or not np.isfinite(a).all() or not np.isfinite(r).all()):
            raise ValueError("invalid transformer context")
        time = self.parameters["time_embed.weight"][np.clip(t, 0, self.architecture["max_timestep"])]
        width, heads = self.architecture["width"], self.architecture["heads"]
        tokens = np.stack((self._linear(r, "return_embed") + time,
                           self._linear(x, "state_embed") + time,
                           self._linear(a, "action_embed") + time), axis=1).reshape(3 * length, width)
        n = len(tokens)
        causal = np.triu(np.ones((n, n), dtype=bool), k=1)
        for layer in range(self.architecture["layers"]):
            prefix = f"blocks.{layer}"
            normalized = self._norm(tokens, prefix + ".norm1")
            q, k, v = self._linear(normalized, prefix + ".attn.qkv").reshape(
                n, 3, heads, width // heads).transpose(1, 2, 0, 3)
            score = (q @ k.transpose(0, 2, 1)) / math.sqrt(width // heads)
            score = np.where(causal, np.float32(-1e30), score)
            probability = np.exp(score - score.max(axis=-1, keepdims=True))
            probability /= probability.sum(axis=-1, keepdims=True)
            attended = (probability @ v).transpose(1, 0, 2).reshape(n, width)
            tokens = tokens + self._linear(attended, prefix + ".attn.out")
            hidden = self._linear(self._norm(tokens, prefix + ".norm2"), prefix + ".fc1")
            hidden = 0.5 * hidden * (1 + np.tanh(math.sqrt(2 / math.pi) * (hidden + 0.044715 * hidden ** 3)))
            tokens = tokens + self._linear(hidden, prefix + ".fc2")
        output = np.tanh(self._linear(self._norm(tokens, "norm_out")[-2], "head"))
        if not np.isfinite(output).all():
            raise ValueError("nonfinite model output")
        return output


SUBGOAL_KINDS = (TaskType.SUPPORT, TaskType.DEFEND)
# Until the next inference these proposals cannot steer; a refined task keeps its sub-goal.
KEEP_STATUSES = ("OUT_OF_FIELD", "NO_DIRECTION", "PROPOSAL_REACHED", "NO_LEGAL_PROJECTION")
# rl_mirror / --mirror -> GoalPrior ``mirror``: auto follows the field's mirror setting (a
# mirrored BLUE field mirrors the features); true and false force mirrored or unmirrored.
MIRROR_SETTINGS = {"auto": None, "true": True, "false": False}


def mirror_setting(value: str) -> bool | None:
    """The GoalPrior ``mirror`` argument for an rl_mirror string (auto, true or false)."""
    setting = value.lower() if isinstance(value, str) else None
    if setting not in MIRROR_SETTINGS:
        raise ValueError(f"rl_mirror must be auto, true or false, not {value!r}")
    return MIRROR_SETTINGS[setting]


class GoalPrior:
    """Rank feasible goals with an offline-trained navigation proposal.

    Rule candidates are blended with how well their routes agree with the proposal.
    The proposal, projected onto the nearest legal, reachable position, is never a
    candidate: after arbitration this sub-goal may only refine the goal position of
    the task the (blended) rules chose, a non-urgent SUPPORT/DEFEND task, so the model
    never changes which task wins, nor its kind, zone, target or urgency. A refined
    task keeps its sub-goal while the proposal cannot steer (for example once it is
    reached) instead of jumping back to its rule goal; normal hysteresis decides when
    to leave the task. With blend 0 the proposal is only recorded. The decision engine
    lowers the blend for a cycle, down to 0, while teammate or enemy positions are
    incomplete (``Config.completeness_threshold``). The model never moves rule
    candidates or bypasses rule filters. DT context records observed past
    displacement, not unexecuted plans.
    """

    def __init__(self, model, config, blend=0.25, mirror=None, allow_transfer=False):
        if not math.isfinite(blend) or not 0 <= blend <= 1:
            raise ValueError("rl_blend must be in [0,1]")
        x0, y0, x1, y1 = config.bounds
        self.domain_transfer = not (
            math.isclose(x1 - x0, model.spec.width) and math.isclose(y1 - y0, model.spec.height)
        )
        if self.domain_transfer and not allow_transfer:
            raise ValueError("RL field dimensions differ; explicit domain-transfer opt-in required")
        # None follows the field: a mirrored (BLUE) field also mirrors the model features.
        self.mirror = config.mirrored if mirror is None else bool(mirror)
        self.model, self.config, self.blend = model, config, blend
        self.spec = replace(model.spec, width=x1 - x0, height=y1 - y0, origin_x=x0, origin_y=y0)
        self.model_id = hashlib.sha256(
            json.dumps(model.payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        self.reset()

    @classmethod
    def load(cls, path, config, blend=0.25, mirror=None, allow_transfer=False):
        return cls(NumpyNavigationPolicy.load(path), config, blend, mirror, allow_transfer)

    def reset(self):
        self.frames, self.completed_actions = [], []
        self.last_update, self.first_time, self.generation = None, None, None
        self.proposal = None
        self.proposal_origin = None
        self.proposal_satisfied = False
        self.subgoal = None
        self.info = {}
        self._status = "UNINITIALIZED"

    def _update(self, world):
        own = world.self_track
        if (self.last_update is not None and (
            world.stamp < self.last_update or world.stamp - self.last_update > 2 * self.spec.horizon
            or self.generation != own.generation
        )):
            self.reset()
        if self.last_update is not None and world.stamp - self.last_update < self.spec.horizon:
            return
        inputs = view_inputs(world, self.config)
        observation = encode_world(world, self.spec, self.mirror, inputs)
        if self.first_time is None:
            self.first_time = world.stamp
        if self.frames:
            old_position = self.frames[-1][3]
            sign = -1. if self.mirror else 1.
            self.completed_actions.append(np.array([
                sign * (own.position.x - old_position.x) / self.spec.width,
                sign * (own.position.y - old_position.y) / self.spec.height,
            ], dtype=np.float32))
        remaining = inputs.remaining
        timestep = (max(0., self.spec.regulation_seconds - remaining) if remaining is not None else
                    max(0., world.stamp - self.first_time))
        self.frames.append((observation, world.stamp, int(timestep), own.position))
        context = self.model.architecture.get("context", 1)
        if len(self.frames) > context:
            self.frames = self.frames[-context:]
            self.completed_actions = self.completed_actions[-max(0, context - 1):] if context > 1 else []
        observations = np.stack([frame[0] for frame in self.frames])
        actions = np.asarray([*self.completed_actions, np.zeros(2)], dtype=np.float32)
        times = np.asarray([frame[2] for frame in self.frames], dtype=np.int64)
        returns = np.full((len(self.frames), 1), self.model.target_return / self.model.return_scale,
                          dtype=np.float32)
        prediction = self.model.predict(observations, actions, returns, times)
        if prediction.shape != (2,) or not np.isfinite(prediction).all():
            raise ValueError("invalid learned navigation proposal")
        sign = -1. if self.mirror else 1.
        self.proposal = Vec2(
            own.position.x + sign * float(prediction[0]) * self.spec.width,
            own.position.y + sign * float(prediction[1]) * self.spec.height,
        )
        self.proposal_origin = own.position
        self.proposal_satisfied = False
        x0, y0, x1, y1 = self.config.bounds
        self._status = ("READY" if x0 <= self.proposal.x <= x1 and y0 <= self.proposal.y <= y1
                        else "OUT_OF_FIELD")
        self.last_update, self.generation = world.stamp, own.generation

    def rank(self, world, candidates, arena, blend_scale=1.0, blend_reason=""):
        """Blend rule utilities with the proposal and project it onto one legal sub-goal.

        The sub-goal (``subgoal``) is not added to the candidates; see ``subgoal_for``.
        ``blend_scale`` < 1 lowers the configured blend when inputs are incomplete
        (``blend_reason``). An effective blend of 0 only records the proposal (SHADOW),
        as a configured blend of 0 does, so no sub-goal is offered that cycle either.
        """
        self.subgoal = None
        blend = self.blend * min(1., max(0., blend_scale))
        self.info = {
            "algorithm": self.model.payload["algorithm"], "model_id": self.model_id,
            "dataset_id": self.model.payload.get("provenance", {}).get("dataset_id"),
            "domain_transfer": self.domain_transfer, "mirror": self.mirror,
            "blend": self.blend, "effective_blend": blend, "used": False,
        }
        if blend_reason:
            self.info["blend_reason"] = blend_reason
        if not any(c.kind != TaskType.HOLD and c.goal is not None for c in candidates):
            self.info["status"] = "NO_FEASIBLE_TASK"
            return candidates
        try:
            self._update(world)
        except (ValueError, FloatingPointError) as error:
            self.reset()
            self.info = {"used": False, "status": "INVALID_MODEL_OUTPUT", "reason": str(error)}
            return candidates
        age = world.stamp - self.last_update
        self.info.update(status=self._status, cache_age=age,
                         proposal=[self.proposal.x, self.proposal.y])
        if blend == 0:
            self.info.update(status="SHADOW", proposal_status=self._status)
            return candidates
        own = world.self_track.position
        if self._status != "READY":
            return candidates
        direction = Vec2(self.proposal.x - self.proposal_origin.x,
                         self.proposal.y - self.proposal_origin.y)
        remaining = Vec2(self.proposal.x - own.x, self.proposal.y - own.y)
        if direction.norm < 0.3:
            self.info["status"] = "NO_DIRECTION"
            return candidates
        if (self.proposal_satisfied or remaining.norm <= 0.5
                or direction.x * remaining.x + direction.y * remaining.y <= 0):
            self.proposal_satisfied = True
            self.info["status"] = "PROPOSAL_REACHED"
            return candidates
        # Match progress lengths. A short human movement suggestion must not be
        # compared to a forced maximum-speed waypoint on every distant route.
        travel = min(remaining.norm, self.config.nominal_speed * max(0.2, self.spec.horizon - age))
        radius = max(0.5, 0.5 * travel)
        comparison_goal = Vec2(own.x + remaining.x * travel / remaining.norm,
                               own.y + remaining.y * travel / remaining.norm)
        projected = arena.nearest_legal(comparison_goal, anchor=own)
        if projected is None:
            self.info["status"] = "NO_LEGAL_PROJECTION"
            return candidates
        self.subgoal = projected
        self.info.update(projection_distance_m=travel, projected_goal=[projected.x, projected.y])

        def blended(candidate):
            waypoint = arena.advance_from(own, candidate.goal, travel)
            score = math.exp(-0.5 * (waypoint.distance(projected) / radius) ** 2)
            return replace(candidate, utility=(1 - blend) * candidate.utility + blend * score,
                           rule_utility=candidate.utility, learned_utility=score)

        ranked = [c if c.kind == TaskType.HOLD or c.goal is None else blended(c) for c in candidates]
        self.info["used"] = True
        return ranked

    def subgoal_for(self, chosen, kept=None):
        """The sub-goal that may refine ``chosen``, the task the rules chose, and why (not).

        Only a non-urgent SUPPORT/DEFEND task is refined: by this cycle's projected
        proposal or, while the proposal cannot steer, by ``kept``, the sub-goal the task
        is already refined with. Returns ``(position or None, reason)``.
        """
        if chosen.kind == TaskType.HOLD or chosen.goal is None:
            return None, "NO_RULE_TASK"
        if chosen.urgent:
            return None, "URGENT_TASK"
        if chosen.kind not in SUBGOAL_KINDS:
            return None, f"NOT_REFINABLE:{chosen.kind.name}"
        if self.subgoal is not None:
            return self.subgoal, "REFINED"
        status = self.info.get("status", "UNINITIALIZED")
        if kept is not None and status in KEEP_STATUSES:
            return kept, f"KEPT:{status}"
        return None, status
