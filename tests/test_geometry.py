import math

import pytest


def test_path_goes_around_obstacle_and_stays_legal(config):
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2

    arena = TacticalMap(config)
    start, end = Vec2(7, 6), Vec2(13, 6)
    route = arena.route(start, end)
    assert len(route) > 2
    assert arena.distance(start, end) > start.distance(end)
    assert all(arena.segment_legal(a, b) for a, b in zip(route, route[1:]))
    assert arena.route(start, Vec2(10, 6)) == ()
    assert math.isinf(arena.distance(start, Vec2(10, 6)))


def test_configuration_rejects_unsafe_weights_and_points(config):
    from dataclasses import replace
    from sentry_core.config import validate_config
    from sentry_core.model import Vec2

    with pytest.raises(ValueError, match="weights"):
        validate_config(replace(config, weights=(float("nan"),) * 6))
    point = replace(config.points[0], position=Vec2(100, 100))
    with pytest.raises(ValueError, match="point"):
        validate_config(replace(config, points=(point, *config.points[1:])))


def test_keepout_mask_covers_rules_and_bounds_in_navigation_map_coordinates(config, tmp_path):
    import yaml
    from sentry_core.keepout import render_keepout

    (tmp_path / "map.pgm").write_bytes(b"P5\n260 160\n255\n" + bytes([254]) * 260 * 160)
    source = tmp_path / "map.yaml"
    source.write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 0.1,
                                      "origin": [-3., -2., 0.]}))
    result = render_keepout(config, source, tmp_path / "mask")
    metadata = yaml.safe_load(result.read_text())
    pixels = (result.parent / metadata["image"]).read_bytes().split(b"\n", 3)[3]

    def cell(x, y):
        column, row = int((x + 3) / 0.1), 159 - int((y + 2) / 0.1)
        return pixels[row * 260 + column]

    assert cell(3, 3) == 254
    assert cell(10, 6) == 0
    assert cell(-1, 4) == 0
    assert cell(0.01, 4) == 0
    assert metadata["origin"] == [-3., -2., 0.]


def test_zone_uses_reachable_entry_points_when_structure_blocks_its_center(config):
    from dataclasses import replace
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2

    config = replace(config, obstacles=(*config.obstacles, (17.5, 5.5, 18.5, 6.5)))
    arena = TacticalMap(config)
    zone = next(z for z in config.zones if z.name == "enemy_outpost")
    assert not arena.legal(zone.center)
    route = arena.zone_route(Vec2(13, 4), zone)
    assert route and route[-1] == Vec2(15, 4)
    assert arena.zone_distance(Vec2(13, 4), zone) == pytest.approx(2.)
