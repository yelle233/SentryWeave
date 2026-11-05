from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    from sentry_core.config import load_config

    return load_config(ROOT / "src/sentry_bringup/config/tactics.yaml")


@pytest.fixture
def world_factory(config):
    from sentry_core.model import Observation, Phase, Referee, Team, Vec2
    from sentry_core.tracking import WorldModel

    def make(now=10.0, allies=(), enemies=(), hp=0.9, phase=Phase.RUNNING,
             enemy_outpost=True, supply=True, self_position=(3.0, 6.0)):
        model = WorldModel(config)
        for stamp in (now - 1.0, now - 0.5, now):
            model.observe(
                Observation(7, Team.ALLY, Vec2(*self_position), stamp), stamp
            )
            for robot_id, x, y, vx, vy in allies:
                model.observe(
                    Observation(robot_id, Team.ALLY, Vec2(x - vx * (now - stamp),
                                                        y - vy * (now - stamp)),
                                stamp, velocity=Vec2(vx, vy)), stamp
                )
            for robot_id, x, y, vx, vy in enemies:
                model.observe(
                    Observation(robot_id, Team.ENEMY, Vec2(x - vx * (now - stamp),
                                                         y - vy * (now - stamp)),
                                stamp, velocity=Vec2(vx, vy)), stamp
                )
        model.update_referee(Referee(now, phase, hp, 0.9, True, enemy_outpost, supply))
        return model.snapshot(now)

    return make
