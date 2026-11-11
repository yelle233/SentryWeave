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


def test_polygon_obstacles_and_keepouts_keep_robot_clearance(config):
    from dataclasses import replace
    from sentry_core.config import Area
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2

    diamond = Area("diamond", (Vec2(15, 8), Vec2(16, 9), Vec2(15, 10), Vec2(14, 9)))
    wedge = Area("wedge", (Vec2(12, 10), Vec2(15, 10.5), Vec2(12, 11)))
    rule = Area("enemy_supply", (Vec2(17, 0.5), Vec2(19.5, 0.5), Vec2(19.5, 2.5), Vec2(17, 2.5)))
    arena = TacticalMap(replace(config, obstacles=(*config.obstacles, diamond, wedge),
                                keepouts=(rule,)))
    assert not arena.legal(Vec2(15, 9))
    assert not arena.legal(Vec2(16.2, 9))
    assert arena.legal(Vec2(16.5, 9))
    # A sharp corner is bevelled at the robot radius instead of a 1.9 m miter spike.
    assert not arena.legal(Vec2(15.3, 10.5))
    assert arena.legal(Vec2(15.4, 10.5))
    assert not arena.legal(Vec2(18, 2))
    route = arena.route(Vec2(13, 9), Vec2(17, 9))
    assert len(route) > 2
    assert all(arena.segment_legal(a, b) for a, b in zip(route, route[1:]))


def test_grid_connectivity_and_projection_to_the_nearest_legal_position(config):
    from dataclasses import replace
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2

    ring = ((14, 7, 18, 7.4), (14, 10.6, 18, 11), (14, 7, 14.4, 11), (17.6, 7, 18, 11))
    arena = TacticalMap(replace(config, obstacles=(*config.obstacles, *ring)))
    inside, outside = Vec2(16, 9), Vec2(5, 6)
    assert arena.legal(inside) and arena.legal(outside)
    assert not arena.reachable(outside, inside)
    assert arena.reachable(outside, Vec2(5, 3))
    assert arena.route(outside, inside) == ()
    snapped = arena.nearest_legal(inside, anchor=outside)
    assert arena.reachable(outside, snapped)
    assert snapped.distance(inside) > 1.5
    blocked = arena.nearest_legal(Vec2(10, 6))
    assert arena.legal(blocked) and blocked.distance(Vec2(10, 6)) < 2.0
    assert arena.nearest_legal(outside, anchor=outside) == outside
    assert arena.nearest_legal(Vec2(float("nan"), 1)) is None


def test_batch_distances_and_start_rooted_paths_match_routes(config):
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2

    arena = TacticalMap(config)
    start = Vec2(7, 6)
    goals = [Vec2(13, 6), Vec2(15, 8), Vec2(10, 6), Vec2(4, 4)]
    batch = arena.distances(start, goals)
    for goal, value in zip(goals, batch):
        assert value == pytest.approx(arena.distance(start, goal))
    assert math.isinf(batch[2])
    path = arena.path_from(start, goals[1])
    assert path[0] == start and path[-1] == goals[1]
    assert all(arena.segment_legal(a, b) for a, b in zip(path, path[1:]))
    assert arena.advance_from(start, goals[0], 1.0).distance(start) == pytest.approx(1.0)


def test_corner_visibility_is_memoized_per_point_and_read_only(config):
    import numpy as np
    from sentry_core.geometry import TacticalMap
    from sentry_core.model import Vec2

    arena = TacticalMap(config)
    seen = arena._visible(Vec2(7, 6))
    # Routes, batch distances and RL waypoints of one cycle query the same points.
    assert arena._visible(Vec2(7, 6)) is seen
    assert not seen.flags.writeable
    origin = np.repeat([[7., 6.]], len(arena.corners), axis=0)
    assert np.array_equal(seen, ~arena._blocked(origin, arena._nodes))


def test_concave_area_is_rejected(config):
    from dataclasses import replace
    from sentry_core.config import Area, validate_config
    from sentry_core.model import Vec2

    concave = Area("l_shape", (Vec2(1, 1), Vec2(3, 1), Vec2(3, 2), Vec2(2, 2), Vec2(2, 3),
                               Vec2(1, 3)))
    with pytest.raises(ValueError, match="convex"):
        validate_config(replace(config, keepouts=(concave,)))


def test_keepout_mask_rasterizes_polygon_keepouts(config, tmp_path):
    from dataclasses import replace
    import yaml
    from sentry_core.config import Area
    from sentry_core.keepout import render_keepout
    from sentry_core.model import Vec2

    triangle = Area("enemy_zone", (Vec2(14, 1), Vec2(18, 1), Vec2(16, 4)))
    (tmp_path / "map.pgm").write_bytes(b"P5\n200 120\n255\n" + bytes([254]) * 200 * 120)
    source = tmp_path / "map.yaml"
    source.write_text(yaml.safe_dump({"image": "map.pgm", "resolution": 0.1,
                                      "origin": [0., 0., 0.]}))
    result = render_keepout(replace(config, keepouts=(triangle,)), source, tmp_path / "mask")
    pixels = (result.parent / "keepout.pgm").read_bytes().split(b"\n", 3)[3]

    def cell(x, y):
        return pixels[(119 - int(y / 0.1)) * 200 + int(x / 0.1)]

    assert cell(16, 2) == 0
    assert cell(14.2, 3.5) == 254
    assert cell(3, 3) == 254
