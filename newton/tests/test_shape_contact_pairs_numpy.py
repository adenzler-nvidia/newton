# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

import itertools
import unittest

import numpy as np

from newton import ModelBuilder, ShapeFlags
from newton.tests.unittest_utils import add_function_test, get_test_devices


def _assert_pair_filters(test, builder, device):
    excluded = {tuple(sorted(pair)) for pair in builder.shape_collision_filter_pairs}
    indices = [i for i, flags in enumerate(builder.shape_flags) if flags & ShapeFlags.COLLIDE_SHAPES]
    expected = []
    for a, b in itertools.combinations(indices, 2):
        ba, bb = builder.shape_body[a], builder.shape_body[b]
        if ba == bb or (ba < 0 and bb < 0) or (a, b) in excluded:
            continue
        if builder._test_world_and_group_pair(
            builder.shape_world[a],
            builder.shape_world[b],
            builder.shape_collision_group[a],
            builder.shape_collision_group[b],
        ):
            expected.append((a, b))
    model = builder.finalize(device=device)
    actual = model.shape_contact_pairs.numpy()
    actual = actual[np.lexsort((actual[:, 1], actual[:, 0]))]
    np.testing.assert_array_equal(actual, np.asarray(expected, dtype=np.int32).reshape((-1, 2)))
    test.assertEqual(model.shape_contact_pair_count, len(expected))
    test.assertIsInstance(model.shape_contact_pair_count, int)


def test_pair_filters(test, device):
    """Match scalar eligibility across worlds, bodies, groups, and exclusions."""
    for seed in range(8):
        rng = np.random.default_rng(seed)
        builder = ModelBuilder()
        for world in range(4):
            if world < 3:
                builder.begin_world()
            body = -1
            for i in range(70):
                if i % 3 == 0:
                    body = builder.add_body() if rng.integers(2) else -1
                cfg = ModelBuilder.ShapeConfig(
                    collision_group=int(rng.integers(-3, 4)), has_shape_collision=bool(rng.integers(4))
                )
                builder.add_shape_sphere(body, radius=0.5, cfg=cfg)
            if world < 3:
                builder.end_world()
        excluded = {tuple(sorted(map(int, pair))) for pair in rng.integers(builder.shape_count, size=(500, 2))}
        builder.shape_collision_filter_pairs.extend(excluded)
        _assert_pair_filters(test, builder, device)


def test_pair_block_boundaries(test, device):
    """Handle empty scenes and partial row blocks without losing or duplicating pairs."""
    for n in (0, 1, 127, 128, 129, 257, 513):
        builder = ModelBuilder()
        for _ in range(n):
            builder.add_shape_sphere(builder.add_body(), radius=0.5)
        model = builder.finalize(device=device)
        expected = np.asarray(list(itertools.combinations(range(n), 2)), dtype=np.int32).reshape((-1, 2))
        np.testing.assert_array_equal(model.shape_contact_pairs.numpy(), expected)
        test.assertEqual(model.shape_contact_pair_count, n * (n - 1) // 2)


def test_reused_world_pair_variations(test, device):
    """Respect body, group, flag, and filter differences between repeated worlds."""
    source = ModelBuilder()
    bodies = [source.add_body() for _ in range(6)]
    for body in (bodies[0], bodies[0], bodies[1], bodies[2], -1, bodies[3], bodies[4], bodies[5]):
        source.add_shape_sphere(body, radius=0.5)
    source.shape_collision_filter_pairs.append((0, 2))
    builder = ModelBuilder()
    builder.replicate(source, 12)
    for world in range(12):
        start = 8 * world
        match world % 6:
            case 1:
                builder.shape_collision_group[start + 1] = 2
            case 2:
                builder.shape_collision_filter_pairs.append((start + 2, start + 3))
            case 3:
                builder.shape_flags[start] &= ~int(ShapeFlags.COLLIDE_SHAPES)
            case 4:
                builder.shape_body[start + 2] = builder.shape_body[start + 3]
            case 5:
                builder.shape_body[start] = -1
                builder.shape_body[start + 1] = -1
    global_body = builder.add_body()
    global_shape = builder.add_shape_sphere(global_body, radius=0.5)
    builder.add_shape_sphere(global_body, radius=0.5)
    builder.add_shape_sphere(builder.shape_body[2], radius=0.5)
    builder.add_ground_plane()
    builder.shape_collision_filter_pairs.append((global_shape, 3))
    _assert_pair_filters(test, builder, device)
    builder.shape_collision_filter_pairs.append((0, 3))
    _assert_pair_filters(test, builder, device)


def test_reused_world_compressed_indices(test, device):
    """Map reused pairs back to shapes when different positions are disabled."""
    source = ModelBuilder()
    for _ in range(8):
        source.add_shape_sphere(source.add_body(), radius=0.5)
    builder = ModelBuilder()
    builder.replicate(source, 4)
    for world in range(4):
        disabled = 8 * world + (0 if world % 2 == 0 else 7)
        builder.shape_flags[disabled] &= ~int(ShapeFlags.COLLIDE_SHAPES)
    _assert_pair_filters(test, builder, device)


class TestShapeContactPairsNumpy(unittest.TestCase):
    pass


for func in (
    test_pair_filters,
    test_pair_block_boundaries,
    test_reused_world_pair_variations,
    test_reused_world_compressed_indices,
):
    add_function_test(TestShapeContactPairsNumpy, func.__name__, func, devices=get_test_devices())


if __name__ == "__main__":
    unittest.main(verbosity=2)
