"""
Vision-Based Navigation: Path Planning and Navigation Controller

Implements A* path planning on an occupancy grid produced by the
VisionSystem, and a high-level NavigationController that converts
the planned path into robot velocity commands.
"""

import numpy as np
import heapq
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class NavigationGoal:
    """A navigation target"""
    position: np.ndarray          # [x, y] world position
    tolerance: float = 0.20       # metres — considered arrived within this


@dataclass
class PathPlannerConfig:
    """Configuration for the path planner"""
    # A* heuristic weight (1.0 = pure A*, >1.0 = faster but sub-optimal)
    heuristic_weight: float = 1.2

    # Safety inflation radius around obstacles (grid cells)
    obstacle_inflation: int = 2

    # Maximum number of A* nodes to expand before giving up
    max_nodes: int = 50_000


@dataclass
class NavigationConfig:
    """Configuration for the navigation controller"""
    # Maximum forward/lateral speed (m/s)
    max_linear_speed: float = 0.5
    max_angular_speed: float = 0.8   # rad/s

    # Lookahead distance on path (metres)
    lookahead_distance: float = 0.5

    # Slow-down radius near goal (metres)
    goal_slowdown_radius: float = 1.0

    # Obstacle avoidance — minimum clearance before braking (metres)
    obstacle_brake_distance: float = 0.4

    # Replanning interval (seconds)
    replan_interval: float = 1.0


class NavState(Enum):
    IDLE     = "idle"
    PLANNING = "planning"
    MOVING   = "moving"
    ARRIVED  = "arrived"
    STUCK    = "stuck"


# ---------------------------------------------------------------------------
# A* Path Planner
# ---------------------------------------------------------------------------

class PathPlanner:
    """
    A* path planner on a 2-D occupancy grid.

    Grid convention:
        grid[row, col] == 0  →  free
        grid[row, col] == 1  →  occupied

    Origin (world coordinates of grid[0,0]) is stored in `origin`.
    Cell size is `resolution` metres.
    """

    def __init__(self, config: Optional[PathPlannerConfig] = None):
        self.config = config or PathPlannerConfig()

    def plan(
        self,
        start_world: np.ndarray,
        goal_world: np.ndarray,
        grid: np.ndarray,
        origin: np.ndarray,
        resolution: float,
    ) -> Optional[List[np.ndarray]]:
        """
        Plan a path from start to goal in world coordinates.

        Args:
            start_world: [x, y] start position (metres).
            goal_world:  [x, y] goal position (metres).
            grid:        2-D occupancy grid (rows × cols).
            origin:      World [x, y] of grid cell [0, 0].
            resolution:  Metres per grid cell.

        Returns:
            List of [x, y] waypoints in world frame, or None if no path found.
        """
        rows, cols = grid.shape

        def to_cell(world: np.ndarray) -> Tuple[int, int]:
            ix = int((world[0] - origin[0]) / resolution)
            iy = int((world[1] - origin[1]) / resolution)
            return (iy, ix)

        def to_world(cell: Tuple[int, int]) -> np.ndarray:
            row, col = cell
            x = origin[0] + col * resolution + resolution / 2
            y = origin[1] + row * resolution + resolution / 2
            return np.array([x, y])

        def in_bounds(r: int, c: int) -> bool:
            return 0 <= r < rows and 0 <= c < cols

        start_cell = to_cell(start_world)
        goal_cell  = to_cell(goal_world)

        # Inflate obstacles
        inflated = self._inflate(grid, self.config.obstacle_inflation)

        # Safety check
        if not in_bounds(*start_cell) or not in_bounds(*goal_cell):
            logger.warning("Start or goal outside grid — cannot plan")
            return None
        if inflated[goal_cell] == 1:
            logger.warning("Goal is in obstacle — cannot plan")
            return None

        # A* search
        open_heap: List[Tuple[float, Tuple[int, int]]] = []
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {}
        g_score: Dict[Tuple[int, int], float] = {}

        g_score[start_cell] = 0.0
        h = self._heuristic(start_cell, goal_cell) * self.config.heuristic_weight
        heapq.heappush(open_heap, (h, start_cell))
        came_from[start_cell] = None

        expanded = 0
        neighbours = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        costs      = [1.0,  1.0,   1.0,   1.0,   1.414,1.414,1.414, 1.414]

        while open_heap and expanded < self.config.max_nodes:
            _, current = heapq.heappop(open_heap)
            expanded += 1

            if current == goal_cell:
                return self._reconstruct_path(came_from, current, to_world)

            for (dr, dc), cost in zip(neighbours, costs):
                nr, nc = current[0] + dr, current[1] + dc
                if not in_bounds(nr, nc) or inflated[nr, nc] == 1:
                    continue
                neighbour = (nr, nc)
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbour, float("inf")):
                    came_from[neighbour] = current
                    g_score[neighbour]   = tentative_g
                    f = tentative_g + self._heuristic(neighbour, goal_cell) * self.config.heuristic_weight
                    heapq.heappush(open_heap, (f, neighbour))

        logger.warning(f"A* failed to find path after {expanded} expansions")
        return None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Octile distance heuristic."""
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return max(dr, dc) + (np.sqrt(2) - 1) * min(dr, dc)

    @staticmethod
    def _inflate(grid: np.ndarray, radius: int) -> np.ndarray:
        """Binary dilation of obstacles for safety margin."""
        if radius == 0:
            return grid
        inflated = grid.copy()
        rows, cols = grid.shape
        obstacle_cells = list(zip(*np.where(grid == 1)))
        for (r, c) in obstacle_cells:
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        inflated[nr, nc] = 1
        return inflated

    @staticmethod
    def _reconstruct_path(
        came_from: Dict,
        current: Tuple[int, int],
        to_world,
    ) -> List[np.ndarray]:
        path = []
        while current is not None:
            path.append(to_world(current))
            current = came_from[current]
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Navigation Controller
# ---------------------------------------------------------------------------

class NavigationController:
    """
    High-level navigation controller that uses:
    - VisionSystem for occupancy grid / obstacle detection
    - PathPlanner for global A* planning
    - Pure-pursuit lookahead for local velocity commands

    Integrates with the main ControlLoop as a registered subsystem.
    """

    def __init__(
        self,
        planner: Optional[PathPlanner] = None,
        config: Optional[NavigationConfig] = None,
    ):
        self.planner = planner or PathPlanner()
        self.config  = config  or NavigationConfig()

        self.state = NavState.IDLE
        self.goal: Optional[NavigationGoal] = None
        self._path: Optional[List[np.ndarray]] = None
        self._path_index: int = 0
        self._replan_timer: float = 0.0

        self._last_velocity = np.zeros(3)   # [vx, vy, omega]

        logger.info("Navigation controller initialised")

    # ------------------------------------------------------------------
    # Goal management
    # ------------------------------------------------------------------

    def set_goal(self, goal: NavigationGoal):
        """Set a new navigation goal. Triggers replanning."""
        self.goal  = goal
        self._path = None
        self._path_index = 0
        self.state = NavState.PLANNING
        logger.info(f"Navigation goal set: {goal.position}")

    def cancel(self):
        """Cancel current navigation goal."""
        self.goal  = None
        self._path = None
        self.state = NavState.IDLE

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def update(
        self,
        dt: float,
        robot_position: np.ndarray,
        robot_yaw: float,
        vision_system=None,       # VisionSystem instance (optional)
    ) -> np.ndarray:
        """
        Compute velocity command for this control cycle.

        Args:
            dt:             Time step (seconds).
            robot_position: Current robot [x, y, z].
            robot_yaw:      Current heading (radians).
            vision_system:  VisionSystem for grid building / obstacle detection.

        Returns:
            Velocity command [vx, vy, omega] in robot frame.
        """
        if self.goal is None or self.state == NavState.IDLE:
            return np.zeros(3)

        robot_xy = robot_position[:2]

        # ---- Check arrival ----
        if self._at_goal(robot_xy):
            self.state = NavState.ARRIVED
            logger.info("Navigation: arrived at goal")
            return np.zeros(3)

        # ---- Build grid & plan (or replan) ----
        self._replan_timer += dt
        needs_plan = (
            self._path is None
            or self._replan_timer > self.config.replan_interval
        )

        if needs_plan and vision_system is not None:
            grid, origin = vision_system.build_occupancy_grid(robot_position)
            resolution   = vision_system.config.grid_resolution
            self._path   = self.planner.plan(
                robot_xy, self.goal.position, grid, origin, resolution
            )
            self._path_index   = 0
            self._replan_timer = 0.0
            self.state = NavState.MOVING if self._path else NavState.STUCK

        if self._path is None:
            return np.zeros(3)

        # ---- Obstacle avoidance brake ----
        if vision_system is not None:
            obstacles = vision_system.detect_obstacles(robot_position)
            for obs in obstacles:
                if obs.distance < self.config.obstacle_brake_distance:
                    logger.debug(f"Obstacle at {obs.distance:.2f}m — braking")
                    return np.zeros(3)

        # ---- Pure-pursuit ----
        cmd = self._pure_pursuit(robot_xy, robot_yaw)
        self._last_velocity = cmd
        return cmd

    # ------------------------------------------------------------------
    # Pure-pursuit controller
    # ------------------------------------------------------------------

    def _pure_pursuit(
        self,
        robot_xy: np.ndarray,
        robot_yaw: float,
    ) -> np.ndarray:
        """
        Simple pure-pursuit that tracks the planned path.

        Returns:
            [vx, 0, omega] in robot base frame.
        """
        if self._path is None or len(self._path) == 0:
            return np.zeros(3)

        cfg = self.config

        # Advance path index past waypoints we have already passed
        while self._path_index < len(self._path) - 1:
            dist = float(np.linalg.norm(self._path[self._path_index] - robot_xy))
            if dist < cfg.lookahead_distance * 0.5:
                self._path_index += 1
            else:
                break

        # Find lookahead point
        lookahead = self._find_lookahead(robot_xy)

        # Vector to lookahead in world frame
        delta = lookahead - robot_xy
        world_angle = np.arctan2(delta[1], delta[0])

        # Heading error in robot frame
        heading_error = self._wrap_angle(world_angle - robot_yaw)

        # Distance to goal for speed scaling
        dist_to_goal = float(np.linalg.norm(self.goal.position - robot_xy))
        speed_scale  = min(1.0, dist_to_goal / cfg.goal_slowdown_radius)

        vx    = cfg.max_linear_speed * speed_scale * np.cos(heading_error)
        omega = np.clip(
            cfg.max_angular_speed * heading_error,
            -cfg.max_angular_speed,
            cfg.max_angular_speed,
        )

        return np.array([vx, 0.0, omega])

    def _find_lookahead(self, robot_xy: np.ndarray) -> np.ndarray:
        """Find a point on the path `lookahead_distance` ahead."""
        cfg = self.config
        path = self._path

        # Walk along path from current index to find lookahead
        accumulated = 0.0
        for i in range(self._path_index, len(path) - 1):
            seg = path[i + 1] - path[i]
            seg_len = float(np.linalg.norm(seg))
            if accumulated + seg_len >= cfg.lookahead_distance:
                remaining = cfg.lookahead_distance - accumulated
                return path[i] + seg / seg_len * remaining
            accumulated += seg_len

        return path[-1]

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _at_goal(self, robot_xy: np.ndarray) -> bool:
        if self.goal is None:
            return False
        return float(np.linalg.norm(robot_xy - self.goal.position)) < self.goal.tolerance

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return float((angle + np.pi) % (2 * np.pi) - np.pi)

    @property
    def current_path(self) -> Optional[List[np.ndarray]]:
        return self._path

    @property
    def nav_state(self) -> NavState:
        return self.state
