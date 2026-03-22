"""
Terrain Generation for Humanoid Robot Simulation

Supports flat ground, slopes, stairs, and uneven terrain using PyBullet.
"""

import numpy as np
import pybullet as p
import pybullet_data
from typing import List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class TerrainType(Enum):
    FLAT = "flat"
    SLOPE = "slope"
    STAIRS = "stairs"
    UNEVEN = "uneven"


@dataclass
class TerrainConfig:
    """Configuration for terrain generation"""
    terrain_type: TerrainType = TerrainType.FLAT

    # Slope parameters
    slope_angle_deg: float = 10.0       # degrees (0-30 practical range)
    slope_length: float = 4.0           # meters
    slope_width: float = 2.0            # meters

    # Stairs parameters
    num_steps: int = 8
    step_height: float = 0.15           # meters per step
    step_depth: float = 0.30            # meters per step (forward)
    stair_width: float = 1.5            # meters

    # Uneven terrain parameters
    uneven_amplitude: float = 0.03      # meters (noise amplitude)
    uneven_resolution: int = 64         # heightfield grid resolution
    uneven_size: float = 4.0            # meters (square)

    # Friction and restitution
    lateral_friction: float = 0.8
    restitution: float = 0.1


@dataclass
class TerrainInfo:
    """Runtime information about loaded terrain"""
    terrain_type: TerrainType
    body_ids: List[int] = field(default_factory=list)
    # Bounding box
    x_min: float = -2.0
    x_max: float = 6.0
    y_min: float = -2.0
    y_max: float = 2.0


class TerrainGenerator:
    """
    Generates and manages terrain in a PyBullet simulation.

    Supports:
    - Flat ground (default plane.urdf)
    - Inclined slopes (ramps)
    - Staircase environments
    - Uneven / noisy heightfields
    """

    def __init__(self, config: Optional[TerrainConfig] = None):
        self.config = config or TerrainConfig()
        self.terrain_info: Optional[TerrainInfo] = None
        self._height_cache: dict = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def create(self, terrain_type: Optional[TerrainType] = None) -> TerrainInfo:
        """
        Create terrain in the currently connected PyBullet session.

        Args:
            terrain_type: Override config terrain type if provided.

        Returns:
            TerrainInfo with body IDs and bounding information.
        """
        t = terrain_type or self.config.terrain_type

        if t == TerrainType.FLAT:
            info = self._create_flat()
        elif t == TerrainType.SLOPE:
            info = self._create_slope()
        elif t == TerrainType.STAIRS:
            info = self._create_stairs()
        elif t == TerrainType.UNEVEN:
            info = self._create_uneven()
        else:
            raise ValueError(f"Unknown terrain type: {t}")

        self.terrain_info = info
        logger.info(f"Terrain created: {t.value}, bodies={info.body_ids}")
        return info

    def get_height_at(self, x: float, y: float) -> float:
        """
        Estimate terrain height at (x, y) using a short raycast downward.

        Falls back to analytical calculation when no physics server is active.
        """
        key = (round(x, 2), round(y, 2))
        if key in self._height_cache:
            return self._height_cache[key]

        try:
            start = [x, y, 5.0]
            end = [x, y, -1.0]
            result = p.rayTest(start, end)
            if result and result[0][0] != -1:
                hit_pos = result[0][3]
                h = float(hit_pos[2])
            else:
                h = self._analytical_height(x, y)
        except Exception:
            h = self._analytical_height(x, y)

        self._height_cache[key] = h
        return h

    def get_normal_at(self, x: float, y: float) -> np.ndarray:
        """
        Estimate terrain surface normal at (x, y).

        Uses finite differences of height samples.
        """
        d = 0.05  # finite difference step
        dzdx = (self.get_height_at(x + d, y) - self.get_height_at(x - d, y)) / (2 * d)
        dzdy = (self.get_height_at(x, y + d) - self.get_height_at(x, y - d)) / (2 * d)
        normal = np.array([-dzdx, -dzdy, 1.0])
        return normal / np.linalg.norm(normal)

    def get_slope_angle(self, x: float, y: float) -> float:
        """Return terrain inclination angle (radians) at (x, y)."""
        normal = self.get_normal_at(x, y)
        return float(np.arccos(np.clip(normal[2], -1.0, 1.0)))

    def remove(self):
        """Remove all terrain bodies from simulation."""
        if self.terrain_info:
            for bid in self.terrain_info.body_ids:
                try:
                    p.removeBody(bid)
                except Exception:
                    pass
            self.terrain_info = None
            self._height_cache.clear()

    # ------------------------------------------------------------------
    # Terrain builders
    # ------------------------------------------------------------------

    def _create_flat(self) -> TerrainInfo:
        plane_id = p.loadURDF("plane.urdf")
        self._set_friction(plane_id, -1)
        return TerrainInfo(terrain_type=TerrainType.FLAT, body_ids=[plane_id])

    def _create_slope(self) -> TerrainInfo:
        cfg = self.config
        angle_rad = np.radians(cfg.slope_angle_deg)

        # Flat approach section (1 m before slope)
        flat_id = self._box(
            half_extents=[1.0, cfg.slope_width / 2, 0.01],
            position=[-1.0, 0.0, 0.0],
            orientation=[0, 0, 0, 1]
        )

        # Slope body
        slope_len = cfg.slope_length
        slope_thick = 0.05
        slope_id = self._box(
            half_extents=[slope_len / 2, cfg.slope_width / 2, slope_thick],
            position=[slope_len / 2, 0.0, slope_len / 2 * np.sin(angle_rad)],
            orientation=p.getQuaternionFromEuler([0, -angle_rad, 0])
        )

        # Flat landing section
        landing_h = slope_len * np.sin(angle_rad)
        landing_id = self._box(
            half_extents=[1.5, cfg.slope_width / 2, 0.01],
            position=[slope_len + 1.5, 0.0, landing_h],
            orientation=[0, 0, 0, 1]
        )

        ids = [flat_id, slope_id, landing_id]
        return TerrainInfo(
            terrain_type=TerrainType.SLOPE,
            body_ids=ids,
            x_min=-2.0,
            x_max=slope_len + 3.0,
            y_min=-cfg.slope_width,
            y_max=cfg.slope_width,
        )

    def _create_stairs(self) -> TerrainInfo:
        cfg = self.config
        ids = []

        # Flat approach
        flat_id = self._box(
            half_extents=[0.5, cfg.stair_width / 2, 0.01],
            position=[-0.5, 0.0, 0.0],
            orientation=[0, 0, 0, 1]
        )
        ids.append(flat_id)

        # Individual stair steps
        for i in range(cfg.num_steps):
            step_x = i * cfg.step_depth
            step_z = (i + 1) * cfg.step_height

            # Build as a solid block from ground up to step surface
            step_id = self._box(
                half_extents=[cfg.step_depth / 2, cfg.stair_width / 2, step_z / 2],
                position=[step_x + cfg.step_depth / 2, 0.0, step_z / 2],
                orientation=[0, 0, 0, 1]
            )
            ids.append(step_id)

        # Landing platform at top
        top_z = cfg.num_steps * cfg.step_height
        top_x = cfg.num_steps * cfg.step_depth
        landing_id = self._box(
            half_extents=[1.0, cfg.stair_width / 2, 0.01],
            position=[top_x + 1.0, 0.0, top_z],
            orientation=[0, 0, 0, 1]
        )
        ids.append(landing_id)

        return TerrainInfo(
            terrain_type=TerrainType.STAIRS,
            body_ids=ids,
            x_min=-1.0,
            x_max=top_x + 2.5,
            y_min=-cfg.stair_width,
            y_max=cfg.stair_width,
        )

    def _create_uneven(self) -> TerrainInfo:
        cfg = self.config
        res = cfg.uneven_resolution
        size = cfg.uneven_size

        # Generate Perlin-like noise using sum of sinusoids
        rng = np.random.RandomState(42)
        heights = np.zeros((res, res))
        for freq in [1, 2, 4]:
            phase = rng.uniform(0, 2 * np.pi, (2,))
            x = np.linspace(0, freq * 2 * np.pi, res)
            y = np.linspace(0, freq * 2 * np.pi, res)
            xx, yy = np.meshgrid(x, y)
            heights += cfg.uneven_amplitude / freq * np.sin(xx + phase[0]) * np.cos(yy + phase[1])

        # Flatten to 1-D list (row-major)
        height_data = heights.flatten().tolist()

        col_shape = p.createCollisionShape(
            p.GEOM_HEIGHTFIELD,
            meshScale=[size / res, size / res, 1.0],
            heightfieldData=height_data,
            numHeightfieldRows=res,
            numHeightfieldColumns=res,
        )
        vis_shape = p.createVisualShape(
            p.GEOM_HEIGHTFIELD,
            meshScale=[size / res, size / res, 1.0],
            heightfieldData=height_data,
            numHeightfieldRows=res,
            numHeightfieldColumns=res,
            rgbaColor=[0.6, 0.5, 0.4, 1.0],
        )
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col_shape,
            baseVisualShapeIndex=vis_shape,
            basePosition=[size / 2, size / 2, 0.0],
        )
        p.changeDynamics(body_id, -1,
                         lateralFriction=self.config.lateral_friction,
                         restitution=self.config.restitution)
        return TerrainInfo(
            terrain_type=TerrainType.UNEVEN,
            body_ids=[body_id],
            x_min=0, x_max=size,
            y_min=0, y_max=size,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _box(
        self,
        half_extents: List[float],
        position: List[float],
        orientation: List[float],
        color: Optional[List[float]] = None
    ) -> int:
        """Create a static box collision body."""
        color = color or [0.55, 0.55, 0.55, 1.0]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=position,
            baseOrientation=orientation,
        )
        self._set_friction(body_id, -1)
        return body_id

    def _set_friction(self, body_id: int, link_index: int):
        p.changeDynamics(
            body_id, link_index,
            lateralFriction=self.config.lateral_friction,
            restitution=self.config.restitution
        )

    def _analytical_height(self, x: float, y: float) -> float:
        """Compute approximate terrain height analytically (no physics server needed)."""
        cfg = self.config
        t = cfg.terrain_type

        if t == TerrainType.FLAT:
            return 0.0

        if t == TerrainType.SLOPE:
            angle_rad = np.radians(cfg.slope_angle_deg)
            if 0.0 <= x <= cfg.slope_length:
                return x * np.tan(angle_rad)
            elif x > cfg.slope_length:
                return cfg.slope_length * np.tan(angle_rad)
            return 0.0

        if t == TerrainType.STAIRS:
            if x < 0:
                return 0.0
            step_idx = int(x / cfg.step_depth)
            step_idx = min(step_idx, cfg.num_steps)
            return step_idx * cfg.step_height

        return 0.0
