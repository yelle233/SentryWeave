import math


def test_stale_prediction_preserves_source_time(config):
    from sentry_core.model import Observation, Team, TrackStatus, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    assert model.observe(Observation(1, Team.ALLY, Vec2(5, 4), 10), 10)
    assert model.observe(Observation(1, Team.ALLY, Vec2(5.5, 4), 10.5), 10.5)
    live = model.snapshot(10.5).allies[0]
    predicted = model.snapshot(11.0).allies[0]
    stale = model.snapshot(12.0).allies[0]
    assert live.status == TrackStatus.OBSERVED
    assert predicted.status == TrackStatus.PREDICTED
    assert predicted.observed_at == 10.5
    assert predicted.position.x > live.position.x
    assert stale.status == TrackStatus.STALE
    assert stale.confidence < predicted.confidence


def test_rejects_bad_input_and_duplicate_stamps(config):
    from sentry_core.model import Observation, Team, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    assert model.observe(Observation(1, Team.ALLY, Vec2(5, 4), 10), 10)
    for observation in (
        Observation(1, Team.ALLY, Vec2(5, 4), 9),
        Observation(1, Team.ALLY, Vec2(5, 4), 10),
        Observation(1, Team.ALLY, Vec2(5, 4), 15),
        Observation(1, Team.ALLY, Vec2(math.nan, 4), 10.1),
        Observation(1, Team.ALLY, Vec2(-10, 4), 10.1),
        Observation(0, Team.ALLY, Vec2(5, 4), 10.1),
        Observation(1, Team.ALLY, Vec2(5, 4), 10.1, confidence=math.nan),
    ):
        assert not model.observe(observation, 10.1)
    assert model.snapshot(10.1).allies[0].observed_at == 10


def test_opposing_teams_with_same_id_do_not_share_history(config):
    from sentry_core.model import Observation, Team, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    model.observe(Observation(2, Team.ALLY, Vec2(3, 4), 10), 10)
    model.observe(Observation(2, Team.ENEMY, Vec2(17, 4), 10), 10)
    world = model.snapshot(10)
    assert world.allies[0].position != world.enemies[0].position


def test_clock_rewind_clears_tracks_and_referee(config):
    from sentry_core.model import Observation, Phase, Referee, Team, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    model.observe(Observation(7, Team.ALLY, Vec2(3, 4), 10), 10)
    model.update_referee(Referee(10, Phase.RUNNING))
    first = model.snapshot(10)
    rewound = model.snapshot(1)
    assert rewound.self_track is None
    assert rewound.referee is None
    assert rewound.epoch > first.epoch


def test_unknown_and_expired_referee_are_not_zero_health(config):
    from sentry_core.model import Phase, Referee
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    model.update_referee(Referee(10, Phase.RUNNING))
    assert model.snapshot(10).referee.hp is None
    assert model.snapshot(20).referee is None


def test_long_gap_rebuilds_identity_without_old_velocity(config):
    from sentry_core.model import Observation, Team, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    model.observe(Observation(1, Team.ALLY, Vec2(3, 4), 10), 10)
    before = model.snapshot(10).allies[0]
    model.observe(Observation(1, Team.ALLY, Vec2(15, 4), 15), 15)
    after = model.snapshot(15).allies[0]
    assert after.generation > before.generation
    assert after.velocity == Vec2(0, 0)
    assert len(after.history) == 1


def test_self_localization_does_not_smooth_across_an_obstacle_corner(config):
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Observation, Team, Vec2
    from sentry_core.tracking import WorldModel

    model = WorldModel(config)
    poses = (Vec2(8.6799, 5), Vec2(8.6799, 4.1799), Vec2(9.5, 4.1799))
    for i, position in enumerate(poses):
        now = 10 + i * 0.5
        model.observe(Observation(7, Team.ALLY, position, now), now)
    track = model.snapshot(11.1).self_track
    assert track.position == poses[-1]
    assert TacticalMap(config).legal(track.position)
