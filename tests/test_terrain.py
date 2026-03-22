"""Tests for terrain generation and terrain adaptation."""

import numpy as np
import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.terrain import TerrainGenerator, TerrainConfig, TerrainType
from locomotion.terrain_adaptation import TerrainAdaptationController, TerrainAdaptationConfig, TerrainMode


# ---------------------------------------------------------------------------
# TerrainGenerator (analytical, no PyBullet)
# ---------------------------------------------------------------------------

class TestTerrainGeneratorAnalytical:
    """Tests that don't require a PyBullet connection."""

    def test_flat_height(self):
        cfg = TerrainConfig(terrain_type=TerrainType.FLAT)
        gen = TerrainGenerator(cfg)
        assert gen._analytical_height(0.0, 0.0) == pytest.approx(0.0)
        assert gen._analytical_height(5.0, 3.0) == pytest.approx(0.0)

    def test_slope_height_at_origin(self):
        cfg = TerrainConfig(terrain_type=TerrainType.SLOPE, slope_angle_deg=10.0, slope_length=4.0)
        gen = TerrainGenerator(cfg)
        assert gen._analytical_height(0.0, 0.0) == pytest.approx(0.0, abs=1e-6)

    def test_slope_height_increases(self):
        cfg = TerrainConfig(terrain_type=TerrainType.SLOPE, slope_angle_deg=10.0, slope_length=4.0)
        gen = TerrainGenerator(cfg)
        h1 = gen._analytical_height(1.0, 0.0)
        h2 = gen._analytical_height(2.0, 0.0)
        assert h2 > h1 > 0.0

    def test_slope_height_clamps_beyond_length(self):
        cfg = TerrainConfig(terrain_type=TerrainType.SLOPE, slope_angle_deg=10.0, slope_length=4.0)
        gen = TerrainGenerator(cfg)
        h_end  = gen._analytical_height(4.0, 0.0)
        h_past = gen._analytical_height(8.0, 0.0)
        assert h_past == pytest.approx(h_end, abs=1e-6)

    def test_stairs_height_discrete(self):
        cfg = TerrainConfig(
            terrain_type=TerrainType.STAIRS,
            num_steps=5,
            step_height=0.15,
            step_depth=0.30,
        )
        gen = TerrainGenerator(cfg)
        h0 = gen._analytical_height(-0.1, 0.0)  # before stairs
        h1 = gen._analytical_height(0.1,  0.0)  # step 0 (index 0 → 0*0.15 = 0)
        h2 = gen._analytical_height(0.35, 0.0)  # step 1 (index 1 → 1*0.15)
        assert h0 == pytest.approx(0.0)
        assert h2 == pytest.approx(0.15, abs=1e-6)


# ---------------------------------------------------------------------------
# TerrainAdaptationController
# ---------------------------------------------------------------------------

class TestTerrainAdaptationController:
    """Tests that don't require PyBullet."""

    def _make_ctrl(self):
        return TerrainAdaptationController(TerrainAdaptationConfig())

    def test_adapt_flat_no_change(self):
        ctrl = self._make_ctrl()
        # No terrain generator → passthrough
        fp = ctrl.adapt_foot_targets(
            np.array([0.0, 0.1]),
            np.array([0.0, -0.1]),
            nominal_left_z=0.0,
            nominal_right_z=0.0,
        )
        assert fp.left_target[2] == pytest.approx(0.0)
        assert fp.right_target[2] == pytest.approx(0.0)

    def test_adapt_with_mock_terrain(self):
        ctrl = self._make_ctrl()

        class MockTerrain:
            def get_height_at(self, x, y): return 0.20
            def get_normal_at(self, x, y): return np.array([0.0, 0.0, 1.0])

        ctrl.set_terrain_generator(MockTerrain())
        fp = ctrl.adapt_foot_targets(
            np.array([1.0, 0.1]),
            np.array([1.0, -0.1]),
            nominal_left_z=0.0,
            nominal_right_z=0.0,
        )
        # Stance foot should land at terrain height
        assert fp.left_target[2] == pytest.approx(0.20, abs=1e-6)

    def test_swing_foot_gets_clearance(self):
        ctrl = self._make_ctrl()

        class MockTerrain:
            def get_height_at(self, x, y): return 0.10
            def get_normal_at(self, x, y): return np.array([0.0, 0.0, 1.0])

        ctrl.set_terrain_generator(MockTerrain())
        # nominal_left_z > 0 → swing phase
        fp = ctrl.adapt_foot_targets(
            np.array([0.5, 0.1]),
            np.array([0.5, -0.1]),
            nominal_left_z=0.05,   # swing
            nominal_right_z=0.0,   # stance
        )
        # Swing foot z = terrain_h + nominal_z + clearance
        expected_min = 0.10 + 0.05  # at least terrain + swing height
        assert fp.left_target[2] >= expected_min

    def test_body_lean_no_terrain(self):
        ctrl = self._make_ctrl()
        lean = ctrl.get_body_lean_compensation(np.array([0.0, 0.0, 1.0]))
        assert np.allclose(lean, 0.0)

    def test_ankle_correction_flat(self):
        ctrl = self._make_ctrl()
        roll, pitch = ctrl.get_ankle_correction(np.array([0.0, 0.0, 1.0]))
        assert abs(roll)  < 1e-6
        assert abs(pitch) < 1e-6

    def test_ankle_correction_slope(self):
        ctrl = self._make_ctrl()
        # Tilted normal → non-zero correction
        angle = np.radians(10)
        normal = np.array([np.sin(angle), 0.0, np.cos(angle)])
        roll, pitch = ctrl.get_ankle_correction(normal)
        assert abs(pitch) > 0.01

    def test_terrain_mode_default_flat(self):
        ctrl = self._make_ctrl()
        assert ctrl.current_mode == TerrainMode.FLAT

    def test_step_height_override_no_terrain(self):
        ctrl = self._make_ctrl()
        override = ctrl.get_step_height_override(np.array([0.0, 0.0]), np.array([1.0, 0.0]))
        assert override == pytest.approx(0.0)
