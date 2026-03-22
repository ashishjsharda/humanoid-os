"""Tests for vision system, path planner, and navigation controller."""

import numpy as np
import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from navigation.path_planner import (
    PathPlanner,
    PathPlannerConfig,
    NavigationController,
    NavigationConfig,
    NavigationGoal,
    NavState,
)
from sensors.vision import VisionSystem, VisionConfig


# ---------------------------------------------------------------------------
# PathPlanner (A*)
# ---------------------------------------------------------------------------

class TestPathPlanner:
    def _empty_grid(self, rows=20, cols=20):
        return np.zeros((rows, cols), dtype=np.uint8)

    def _planner(self):
        return PathPlanner(PathPlannerConfig(obstacle_inflation=0))

    def test_plan_straight_line(self):
        grid   = self._empty_grid()
        origin = np.array([0.0, 0.0])
        res    = 0.1
        planner = self._planner()
        path = planner.plan(
            np.array([0.05, 0.05]),
            np.array([1.95, 0.05]),
            grid, origin, res,
        )
        assert path is not None
        assert len(path) >= 2

    def test_plan_reaches_goal(self):
        grid   = self._empty_grid(30, 30)
        origin = np.array([0.0, 0.0])
        res    = 0.1
        planner = self._planner()
        goal_world = np.array([2.5, 1.5])
        path = planner.plan(np.array([0.05, 0.05]), goal_world, grid, origin, res)
        assert path is not None
        # Last waypoint should be close to goal
        assert np.linalg.norm(path[-1] - goal_world) < res * 3

    def test_plan_around_wall(self):
        grid   = self._empty_grid(20, 20)
        origin = np.array([0.0, 0.0])
        res    = 0.1
        # Vertical wall at col 10 except top rows — path must go around it
        grid[2:18, 10] = 1
        planner = PathPlanner(PathPlannerConfig(obstacle_inflation=0))
        path = planner.plan(
            np.array([0.85, 0.05]),   # left of wall
            np.array([1.15, 0.05]),   # right of wall
            grid, origin, res,
        )
        assert path is not None

    def test_plan_blocked_returns_none(self):
        grid   = self._empty_grid(10, 10)
        origin = np.array([0.0, 0.0])
        res    = 0.1
        grid[5, :] = 1   # full horizontal wall — no passage
        planner = self._planner()
        path = planner.plan(
            np.array([0.05, 0.35]),
            np.array([0.05, 0.65]),
            grid, origin, res,
        )
        assert path is None

    def test_plan_same_start_goal(self):
        grid   = self._empty_grid()
        planner = self._planner()
        path = planner.plan(
            np.array([0.5, 0.5]),
            np.array([0.5, 0.5]),
            grid, np.array([0.0, 0.0]), 0.1,
        )
        assert path is not None

    def test_obstacle_inflation(self):
        grid   = self._empty_grid(20, 20)
        origin = np.array([0.0, 0.0])
        res    = 0.1
        # Single obstacle cell
        grid[10, 10] = 1
        planner = PathPlanner(PathPlannerConfig(obstacle_inflation=2))
        inflated = planner._inflate(grid, radius=2)
        # Cells near obstacle should also be marked
        assert inflated[9, 10] == 1
        assert inflated[10, 9] == 1

    def test_heuristic_octile(self):
        h = PathPlanner._heuristic((0, 0), (3, 4))
        # Octile: max(3,4) + (sqrt2-1)*min(3,4) = 4 + 0.414*3 ≈ 5.243
        assert h == pytest.approx(4.0 + (np.sqrt(2) - 1) * 3.0, abs=1e-6)


# ---------------------------------------------------------------------------
# NavigationController
# ---------------------------------------------------------------------------

class TestNavigationController:
    def _ctrl(self):
        cfg = NavigationConfig(
            max_linear_speed=0.5,
            lookahead_distance=0.5,
            goal_slowdown_radius=1.0,
        )
        return NavigationController(config=cfg)

    def test_idle_no_goal(self):
        ctrl = self._ctrl()
        cmd = ctrl.update(0.1, np.array([0.0, 0.0, 0.0]), 0.0)
        assert np.allclose(cmd, 0.0)

    def test_set_goal_changes_state(self):
        ctrl = self._ctrl()
        ctrl.set_goal(NavigationGoal(position=np.array([3.0, 0.0])))
        assert ctrl.nav_state == NavState.PLANNING

    def test_arrived_at_goal(self):
        ctrl = self._ctrl()
        goal = NavigationGoal(position=np.array([1.0, 0.0]), tolerance=0.3)
        ctrl.set_goal(goal)
        # Place robot at goal
        ctrl._path = [np.array([1.0, 0.0])]
        cmd = ctrl.update(0.1, np.array([1.0, 0.0, 0.0]), 0.0)
        assert ctrl.nav_state == NavState.ARRIVED

    def test_cancel_resets_state(self):
        ctrl = self._ctrl()
        ctrl.set_goal(NavigationGoal(position=np.array([3.0, 0.0])))
        ctrl.cancel()
        assert ctrl.nav_state == NavState.IDLE
        cmd = ctrl.update(0.1, np.array([0.0, 0.0, 0.0]), 0.0)
        assert np.allclose(cmd, 0.0)

    def test_wrap_angle(self):
        from navigation.path_planner import NavigationController as NC
        assert NC._wrap_angle(0.0)          == pytest.approx(0.0)
        # np.pi wraps to ±pi — just check magnitude
        assert abs(NC._wrap_angle(np.pi))   == pytest.approx(np.pi, abs=1e-6)
        assert abs(NC._wrap_angle(3*np.pi)) == pytest.approx(np.pi, abs=1e-6)
        assert NC._wrap_angle(np.pi / 2)    == pytest.approx(np.pi / 2, abs=1e-6)
        assert NC._wrap_angle(-np.pi / 2)   == pytest.approx(-np.pi / 2, abs=1e-6)

    def test_pure_pursuit_moves_toward_goal(self):
        ctrl = self._ctrl()
        goal_pos = np.array([3.0, 0.0])
        ctrl.set_goal(NavigationGoal(position=goal_pos, tolerance=0.2))
        # Inject a straight path
        ctrl._path = [np.array([0.0, 0.0]), np.array([1.5, 0.0]), np.array([3.0, 0.0])]
        ctrl._path_index = 0
        ctrl.state = NavState.MOVING

        robot_pos = np.array([0.0, 0.0, 0.0])
        cmd = ctrl.update(0.01, robot_pos, 0.0)
        # vx should be positive (moving toward goal)
        assert cmd[0] > 0.0


# ---------------------------------------------------------------------------
# VisionSystem (stub mode — no PyBullet)
# ---------------------------------------------------------------------------

class TestVisionSystemStub:
    def test_init_without_pybullet(self, monkeypatch):
        """VisionSystem should not crash if pybullet import fails."""
        import importlib
        import sensors.vision as vis_module
        # Temporarily make PyBullet unavailable
        original = vis_module.VisionSystem.__init__

        def patched_init(self, config=None):
            self.config = config or VisionConfig()
            self._depth_image = None
            self._pixels = []
            self._occupancy_grid = None
            self._grid_origin = np.zeros(2)
            self._p = None  # simulate no pybullet

        monkeypatch.setattr(vis_module.VisionSystem, "__init__", patched_init)
        vs = VisionSystem()
        result = vs.update(np.array([0.0, 0.0, 1.0]), 0.0)
        assert result == []

    def test_build_empty_occupancy_grid(self):
        vs = VisionSystem()
        vs._p = None
        vs._pixels = []
        grid, origin = vs.build_occupancy_grid(np.array([0.0, 0.0, 1.0]))
        assert grid.ndim == 2
        assert np.all(grid == 0)

    def test_get_floor_height_no_pixels(self):
        vs = VisionSystem()
        vs._p = None
        vs._pixels = []
        h = vs.get_floor_height(1.0, 1.0)
        assert h == 0.0

    def test_detect_obstacles_empty(self):
        vs = VisionSystem()
        vs._pixels = []
        obstacles = vs.detect_obstacles(np.array([0.0, 0.0, 1.0]))
        assert obstacles == []

    def test_ray_direction_forward(self):
        d = VisionSystem._ray_direction(0.0, 0.0)
        assert np.allclose(d, [1.0, 0.0, 0.0])

    def test_ray_direction_up(self):
        d = VisionSystem._ray_direction(0.0, np.pi / 2)
        assert np.allclose(d, [0.0, 0.0, 1.0], atol=1e-6)

    def test_rotate_offset_zero_yaw(self):
        offset = np.array([0.1, 0.0, 0.5])
        rotated = VisionSystem._rotate_offset(offset, yaw=0.0)
        assert np.allclose(rotated, offset)

    def test_rotate_offset_90_yaw(self):
        offset = np.array([1.0, 0.0, 0.0])
        rotated = VisionSystem._rotate_offset(offset, yaw=np.pi / 2)
        assert np.allclose(rotated, [0.0, 1.0, 0.0], atol=1e-6)
