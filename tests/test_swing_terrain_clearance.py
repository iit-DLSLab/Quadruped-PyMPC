"""Checks of the swing clearance from the terrain seen by the heightmaps (hermite generator)."""
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose

from quadruped_pympc import config as cfg
from quadruped_pympc.helpers.swing_generators.cubic_hermite_swing_trajectory_generator import (
    FOOT_RADIUS,
    SwingTrajectoryGenerator,
)

STEP_HEIGHT = 0.06
SWING_PERIOD = 0.25
CLEARANCE = 0.04
LIFT_OFF = np.array([0.0, 0.0, FOOT_RADIUS])
TOUCH_DOWN = np.array([0.35, 0.0, FOOT_RADIUS])


def terrain(height_fn, resolution=0.02):
    """Grid of terrain points around the path, as a heightmap gives them."""
    x, y = np.meshgrid(np.arange(-0.1, 0.46, resolution), np.arange(-0.12, 0.121, resolution))
    return np.stack((x, y, height_fn(x, y)), axis=-1).reshape(-1, 3)


def trajectory(generator, lift_off=LIFT_OFF, touch_down=TOUCH_DOWN, terrain_points=None, num=2001):
    times = np.linspace(0.0, SWING_PERIOD, num)
    return times, np.array([
        generator.compute_trajectory_references(t, lift_off, touch_down, terrain_points=terrain_points)[0]
        for t in times
    ])


def gap_over(points, positions):
    """Height of the foot center above each point, when the foot passes over it."""
    passing = np.argmin(np.abs(positions[None, :, 0] - points[:, None, 0]), axis=1)
    return positions[passing, 2] - points[:, 2]


class SwingTerrainClearanceTests(unittest.TestCase):
    def setUp(self):
        with patch.dict(cfg.locomotion_params, swing_terrain_clearance=CLEARANCE):
            self.generator = SwingTrajectoryGenerator(step_height=STEP_HEIGHT, swing_period=SWING_PERIOD)

    def test_flat_terrain_does_not_change_the_swing(self):
        _, nominal = trajectory(self.generator)
        _, flat = trajectory(self.generator, terrain_points=terrain(lambda x, y: 0.0 * x))
        assert_allclose(flat, nominal, atol=1e-12)

    def test_foot_clears_box_along_the_path_without_going_much_higher(self):
        box = terrain(lambda x, y: np.where((x > 0.1) & (x < 0.2), 0.06, 0.0))
        _, positions = trajectory(self.generator, terrain_points=box)
        obstacles = box[(box[:, 2] > 0.0) & (np.abs(box[:, 1]) <= CLEARANCE)]
        self.assertGreaterEqual(gap_over(obstacles, positions).min(), CLEARANCE - 1e-3)
        # The cubic has to stay above the whole width of the box, so it peaks a bit higher
        self.assertLess(positions[:, 2].max(), 0.06 + CLEARANCE + 0.015)

    def test_obstacle_close_to_lift_off_moves_the_apex_earlier(self):
        wall = terrain(lambda x, y: np.where(x > 0.03, 0.08, 0.0))
        touch_down = np.array([0.35, 0.0, 0.08 + FOOT_RADIUS])
        apex_z, apex_time = self.generator.compute_apex(LIFT_OFF, touch_down, STEP_HEIGHT, wall)
        self.assertLess(apex_time, 0.5)
        self.assertLess(apex_z, touch_down[2] + cfg.locomotion_params['reflex_max_step_height'])
        _, positions = trajectory(self.generator, touch_down=touch_down, terrain_points=wall)
        edge = wall[(wall[:, 2] > 0.0) & (np.abs(wall[:, 1]) <= CLEARANCE) & (wall[:, 0] < 0.05)]
        self.assertGreaterEqual(gap_over(edge, positions).min(), CLEARANCE - 1e-3)

    def test_obstacle_at_lift_off_saturates_at_the_maximum_step_height(self):
        pole = terrain(lambda x, y: np.where((x > 0.01) & (x < 0.05), 0.1, 0.0))
        apex_z, _ = self.generator.compute_apex(LIFT_OFF, TOUCH_DOWN, STEP_HEIGHT, pole)
        assert_allclose(apex_z, LIFT_OFF[2] + cfg.locomotion_params['reflex_max_step_height'])

    def test_step_up_clears_the_edge_lower_than_step_height_above_the_landing(self):
        step_up = terrain(lambda x, y: np.where(x > 0.2, 0.08, 0.0))
        touch_down = TOUCH_DOWN + np.array([0.0, 0.0, 0.08])
        _, positions = trajectory(self.generator, touch_down=touch_down, terrain_points=step_up)
        self.assertLess(positions[:, 2].max(), touch_down[2] + STEP_HEIGHT - 0.01)
        edge = step_up[np.isclose(step_up[:, 0], 0.22) & (np.abs(step_up[:, 1]) <= CLEARANCE)]
        self.assertGreaterEqual(gap_over(edge, positions).min(), CLEARANCE - 1e-3)

    def test_apex_does_not_change_when_the_touchdown_height_is_adapted(self):
        # Before the apex the foothold still has the lift-off height, the visual adaptation raises it later
        step_up = terrain(lambda x, y: np.where(x > 0.25, 0.08, 0.0))
        adapted = TOUCH_DOWN + np.array([0.0, 0.0, 0.08])
        before_apex = self.generator.compute_apex(LIFT_OFF, TOUCH_DOWN, STEP_HEIGHT, step_up)
        assert_allclose(self.generator.compute_apex(LIFT_OFF, adapted, STEP_HEIGHT, step_up), before_apex)
        times, before = trajectory(self.generator, touch_down=TOUCH_DOWN, terrain_points=step_up)
        _, after = trajectory(self.generator, touch_down=adapted, terrain_points=step_up)
        rising = times <= before_apex[1] * SWING_PERIOD
        assert_allclose(before[rising], after[rising], atol=1e-12)

    def test_points_beside_the_path_are_ignored(self):
        _, nominal = trajectory(self.generator)
        beside = terrain(lambda x, y: np.where(np.abs(y) > 0.08, 0.3, 0.0))
        _, positions = trajectory(self.generator, terrain_points=beside)
        assert_allclose(positions, nominal, atol=1e-12)

    def test_horizontal_motion_does_not_depend_on_the_apex(self):
        _, nominal = trajectory(self.generator)
        wall = terrain(lambda x, y: np.where(x > 0.03, 0.08, 0.0))
        _, positions = trajectory(self.generator, terrain_points=wall)
        assert_allclose(positions[:, :2], nominal[:, :2], atol=1e-12)

    def test_disabled_clearance_ignores_the_terrain(self):
        with patch.dict(cfg.locomotion_params, swing_terrain_clearance=None):
            generator = SwingTrajectoryGenerator(step_height=STEP_HEIGHT, swing_period=SWING_PERIOD)
        _, nominal = trajectory(generator)
        box = terrain(lambda x, y: np.where((x > 0.1) & (x < 0.2), 0.1, 0.0))
        _, positions = trajectory(generator, terrain_points=box)
        assert_allclose(positions, nominal, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
