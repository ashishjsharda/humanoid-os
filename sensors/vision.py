"""
Vision System for Humanoid Robot Navigation

Provides simulated depth sensing via PyBullet raycasting, obstacle detection,
and occupancy grid building for vision-based navigation.
"""

import numpy as np
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field
import logging

logger = logging.getLogger(__name__)


@dataclass
class VisionConfig:
    """Configuration for the vision system"""
    # Camera mounting position relative to robot base (metres)
    camera_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.05, 0.0, 0.60])   # ~forehead height
    )

    # Depth camera field of view
    fov_horizontal_deg: float = 70.0
    fov_vertical_deg: float = 50.0

    # Resolution (rays per axis)
    horizontal_rays: int = 32
    vertical_rays: int = 24

    # Sensing range (metres)
    min_range: float = 0.20
    max_range: float = 5.0

    # Obstacle detection
    obstacle_height_min: float = 0.10    # minimum height to count as obstacle
    obstacle_height_max: float = 2.50    # maximum height (ignore ceiling)

    # Occupancy grid
    grid_resolution: float = 0.10        # metres per cell
    grid_radius: float = 3.0             # half-width of grid around robot (m)


@dataclass
class DepthPixel:
    """A single depth measurement"""
    azimuth: float      # horizontal angle (rad)
    elevation: float    # vertical angle (rad)
    distance: float     # metres (max_range if no hit)
    hit: bool
    hit_position: Optional[np.ndarray] = None   # [x, y, z] world coords
    hit_body_id: int = -1


@dataclass
class ObstacleInfo:
    """Detected obstacle summary"""
    position: np.ndarray    # [x, y, z] centroid in world frame
    size: np.ndarray        # bounding box half-extents estimate
    distance: float         # distance from robot
    body_id: int = -1


class VisionSystem:
    """
    Simulated depth vision system using PyBullet ray casting.

    Emulates a forward-facing depth camera (similar to RealSense / LiDAR).
    Provides:
    - Raw depth image (2-D array of distances)
    - Obstacle list in world frame
    - Occupancy grid around the robot
    - Floor height estimation
    """

    def __init__(self, config: Optional[VisionConfig] = None):
        self.config = config or VisionConfig()
        self._depth_image: Optional[np.ndarray] = None
        self._pixels: List[DepthPixel] = []
        self._occupancy_grid: Optional[np.ndarray] = None
        self._grid_origin: np.ndarray = np.zeros(2)

        try:
            import pybullet as p
            self._p = p
        except ImportError:
            self._p = None
            logger.warning("PyBullet not available — vision system in stub mode")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        robot_position: np.ndarray,
        robot_yaw: float,
    ) -> List[DepthPixel]:
        """
        Cast all depth rays from the current robot position/heading.

        Args:
            robot_position: Robot base position [x, y, z].
            robot_yaw:      Robot heading (radians).

        Returns:
            List of DepthPixel measurements.
        """
        if self._p is None:
            return []

        cfg = self.config
        cam_world = robot_position + self._rotate_offset(cfg.camera_offset, robot_yaw)

        h_angles = np.linspace(
            -np.radians(cfg.fov_horizontal_deg / 2),
             np.radians(cfg.fov_horizontal_deg / 2),
             cfg.horizontal_rays,
        )
        v_angles = np.linspace(
            -np.radians(cfg.fov_vertical_deg / 2),
             np.radians(cfg.fov_vertical_deg / 2),
             cfg.vertical_rays,
        )

        # Build ray start/end batches for efficiency
        ray_starts = []
        ray_ends   = []
        ray_meta   = []  # (azimuth, elevation)

        for az in h_angles:
            for el in v_angles:
                direction = self._ray_direction(az + robot_yaw, el)
                start = cam_world + direction * cfg.min_range
                end   = cam_world + direction * cfg.max_range
                ray_starts.append(start.tolist())
                ray_ends.append(end.tolist())
                ray_meta.append((az, el))

        # Batch ray test
        results = self._p.rayTestBatch(ray_starts, ray_ends)

        pixels = []
        depth_vals = np.full((cfg.vertical_rays, cfg.horizontal_rays), cfg.max_range)

        for idx, (res, (az, el)) in enumerate(zip(results, ray_meta)):
            body_id, link_id, hit_frac, hit_pos, hit_norm = res
            hi = idx // cfg.vertical_rays
            vi = idx % cfg.vertical_rays

            if body_id != -1 and hit_frac < 1.0:
                dist = cfg.min_range + hit_frac * (cfg.max_range - cfg.min_range)
                depth_vals[vi, hi] = dist
                px = DepthPixel(
                    azimuth=az, elevation=el, distance=dist,
                    hit=True, hit_position=np.array(hit_pos), hit_body_id=body_id
                )
            else:
                px = DepthPixel(
                    azimuth=az, elevation=el, distance=cfg.max_range, hit=False
                )
            pixels.append(px)

        self._pixels = pixels
        self._depth_image = depth_vals
        return pixels

    def detect_obstacles(
        self,
        robot_position: np.ndarray,
    ) -> List[ObstacleInfo]:
        """
        Cluster depth hits into obstacle objects.

        Args:
            robot_position: Robot base [x, y, z] for relative distance.

        Returns:
            List of detected obstacles.
        """
        cfg = self.config
        hits = [
            px for px in self._pixels
            if px.hit and px.hit_position is not None
               and cfg.obstacle_height_min <= px.hit_position[2] <= cfg.obstacle_height_max
        ]

        if not hits:
            return []

        # Simple grid-based clustering
        clusters: Dict[Tuple[int, int], List[DepthPixel]] = {}
        cell = cfg.grid_resolution * 2

        for px in hits:
            key = (
                int(px.hit_position[0] / cell),
                int(px.hit_position[1] / cell),
            )
            clusters.setdefault(key, []).append(px)

        obstacles = []
        for px_list in clusters.values():
            positions = np.array([px.hit_position for px in px_list])
            centroid = positions.mean(axis=0)
            size_est = (positions.max(axis=0) - positions.min(axis=0)) / 2 + 0.05
            dist = float(np.linalg.norm(centroid[:2] - robot_position[:2]))

            obstacles.append(ObstacleInfo(
                position=centroid,
                size=size_est,
                distance=dist,
                body_id=px_list[0].hit_body_id,
            ))

        obstacles.sort(key=lambda o: o.distance)
        return obstacles

    def build_occupancy_grid(
        self,
        robot_position: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build a 2-D occupancy grid centred on the robot.

        Returns:
            (grid, origin) where grid[row, col] == 1 means occupied,
            and origin is the world [x, y] of grid cell [0, 0].
        """
        cfg = self.config
        radius = cfg.grid_radius
        res    = cfg.grid_resolution
        n      = int(2 * radius / res)

        grid   = np.zeros((n, n), dtype=np.uint8)
        origin = robot_position[:2] - radius

        for px in self._pixels:
            if not px.hit or px.hit_position is None:
                continue
            z = px.hit_position[2]
            if z < cfg.obstacle_height_min or z > cfg.obstacle_height_max:
                continue
            ix = int((px.hit_position[0] - origin[0]) / res)
            iy = int((px.hit_position[1] - origin[1]) / res)
            if 0 <= ix < n and 0 <= iy < n:
                grid[iy, ix] = 1

        self._occupancy_grid = grid
        self._grid_origin    = origin
        return grid, origin

    def get_floor_height(
        self,
        x: float,
        y: float,
        search_radius: float = 0.3,
    ) -> float:
        """
        Estimate floor height at (x, y) using existing depth hits.

        Falls back to a downward raycast if no nearby hit is available.
        """
        best = None
        best_dist = search_radius + 1

        for px in self._pixels:
            if not px.hit or px.hit_position is None:
                continue
            d = np.hypot(px.hit_position[0] - x, px.hit_position[1] - y)
            if d < best_dist and px.hit_position[2] < 0.15:
                best = px.hit_position[2]
                best_dist = d

        if best is not None:
            return float(best)

        # Fallback raycast
        if self._p is not None:
            result = self._p.rayTest([x, y, 3.0], [x, y, -0.5])
            if result and result[0][0] != -1:
                return float(result[0][3][2])

        return 0.0

    def get_depth_image(self) -> Optional[np.ndarray]:
        """Return latest depth image (vertical_rays × horizontal_rays)."""
        return self._depth_image

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ray_direction(azimuth: float, elevation: float) -> np.ndarray:
        """Unit vector for a ray at given azimuth and elevation."""
        return np.array([
            np.cos(elevation) * np.cos(azimuth),
            np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation),
        ])

    @staticmethod
    def _rotate_offset(offset: np.ndarray, yaw: float) -> np.ndarray:
        """Rotate a body-frame offset by yaw around Z."""
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return R @ offset
