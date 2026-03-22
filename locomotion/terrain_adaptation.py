"""
Terrain Adaptation Controller

Adapts foot placement, step height, and balance targets for
slopes, stairs, and uneven surfaces.
"""

import numpy as np
from typing import Optional, Tuple, Dict
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class TerrainMode(Enum):
    FLAT = "flat"
    SLOPE = "slope"
    STAIRS = "stairs"
    UNEVEN = "uneven"


@dataclass
class TerrainAdaptationConfig:
    """Configuration for terrain adaptation"""
    # Foot height look-ahead (meters ahead of current foot position)
    look_ahead_distance: float = 0.25

    # Extra foot lift margin over detected obstacle (meters)
    step_clearance: float = 0.05

    # Maximum slope angle the robot can walk on (radians)
    max_slope_angle: float = np.radians(20)

    # Stair detection thresholds
    stair_height_threshold: float = 0.04   # >4cm height change = step
    stair_depth_min: float = 0.15           # minimum step depth

    # Body posture adjustments on slope
    lean_gain: float = 0.5                  # body lean per radian of slope

    # Foot contact adaptation
    ankle_compensation_gain: float = 0.8


@dataclass
class FootPlacement:
    """Target foot placement with terrain-adapted height"""
    left_target: np.ndarray    # [x, y, z]
    right_target: np.ndarray   # [x, y, z]
    left_normal: np.ndarray    # surface normal at left foot
    right_normal: np.ndarray   # surface normal at right foot
    terrain_mode: TerrainMode = TerrainMode.FLAT


class TerrainAdaptationController:
    """
    Adapts locomotion for non-flat terrain.

    Responsibilities:
    - Estimate terrain height under each foot from a TerrainGenerator
    - Adjust foot z-targets to land on the terrain surface
    - Compensate body lean on slopes
    - Increase step height over stair risers
    - Provide ankle roll/pitch corrections for surface normal alignment
    """

    def __init__(self, config: Optional[TerrainAdaptationConfig] = None):
        self.config = config or TerrainAdaptationConfig()
        self._terrain_generator = None

        # History for stair detection
        self._left_height_history: list = []
        self._right_height_history: list = []
        self._history_len = 10

        self.current_mode = TerrainMode.FLAT
        logger.info("Terrain adaptation controller initialized")

    def set_terrain_generator(self, terrain_generator):
        """Attach a TerrainGenerator for height/normal queries."""
        self._terrain_generator = terrain_generator

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------

    def adapt_foot_targets(
        self,
        left_xy: np.ndarray,
        right_xy: np.ndarray,
        nominal_left_z: float,
        nominal_right_z: float,
    ) -> FootPlacement:
        """
        Given nominal (flat-ground) foot target XY positions, compute
        terrain-adapted 3-D foot targets.

        Args:
            left_xy:  Desired left foot position [x, y] on flat ground.
            right_xy: Desired right foot position [x, y] on flat ground.
            nominal_left_z:  Nominal left foot height (0 on flat ground).
            nominal_right_z: Nominal right foot height (0 on flat ground).

        Returns:
            FootPlacement with adapted 3-D targets and surface normals.
        """
        if self._terrain_generator is None:
            return FootPlacement(
                left_target=np.array([left_xy[0], left_xy[1], nominal_left_z]),
                right_target=np.array([right_xy[0], right_xy[1], nominal_right_z]),
                left_normal=np.array([0.0, 0.0, 1.0]),
                right_normal=np.array([0.0, 0.0, 1.0]),
            )

        # Query terrain heights at foot targets
        lh = self._terrain_generator.get_height_at(left_xy[0], left_xy[1])
        rh = self._terrain_generator.get_height_at(right_xy[0], right_xy[1])

        # Query normals
        ln = self._terrain_generator.get_normal_at(left_xy[0], left_xy[1])
        rn = self._terrain_generator.get_normal_at(right_xy[0], right_xy[1])

        # Update history for stair detection
        self._push_history(self._left_height_history, lh)
        self._push_history(self._right_height_history, rh)

        # Detect terrain mode
        self.current_mode = self._detect_terrain_mode(lh, rh, ln, rn)

        # Adapt heights with clearance on rising terrain
        adapted_lz = self._adapt_height(lh, nominal_left_z, ln)
        adapted_rz = self._adapt_height(rh, nominal_right_z, rn)

        return FootPlacement(
            left_target=np.array([left_xy[0], left_xy[1], adapted_lz]),
            right_target=np.array([right_xy[0], right_xy[1], adapted_rz]),
            left_normal=ln,
            right_normal=rn,
            terrain_mode=self.current_mode,
        )

    def get_body_lean_compensation(self, com_position: np.ndarray) -> np.ndarray:
        """
        Compute additional body lean (roll, pitch, yaw) to compensate for terrain slope.

        Returns:
            Orientation adjustment [roll, pitch, yaw] in radians.
        """
        if self._terrain_generator is None:
            return np.zeros(3)

        normal = self._terrain_generator.get_normal_at(com_position[0], com_position[1])
        slope_angle = float(np.arccos(np.clip(normal[2], -1.0, 1.0)))

        if slope_angle < 0.01:
            return np.zeros(3)

        # Pitch forward on uphill slope
        pitch_comp = -self.config.lean_gain * slope_angle * np.sign(normal[0])
        roll_comp = -self.config.lean_gain * slope_angle * np.sign(normal[1])

        return np.array([roll_comp, pitch_comp, 0.0])

    def get_step_height_override(
        self,
        foot_xy: np.ndarray,
        swing_direction: np.ndarray,
    ) -> float:
        """
        Return extra step height needed to clear upcoming terrain features.

        Args:
            foot_xy: Current foot position [x, y].
            swing_direction: Unit vector of foot travel direction [x, y].

        Returns:
            Additional step clearance height (meters).
        """
        if self._terrain_generator is None:
            return 0.0

        current_h = self._terrain_generator.get_height_at(foot_xy[0], foot_xy[1])

        # Sample ahead
        ahead_xy = foot_xy + swing_direction * self.config.look_ahead_distance
        ahead_h = self._terrain_generator.get_height_at(ahead_xy[0], ahead_xy[1])

        height_rise = ahead_h - current_h
        if height_rise > self.config.stair_height_threshold:
            return height_rise + self.config.step_clearance

        return 0.0

    def get_ankle_correction(self, foot_normal: np.ndarray) -> Tuple[float, float]:
        """
        Compute ankle roll/pitch corrections to align foot with terrain surface.

        Args:
            foot_normal: Surface normal under the foot.

        Returns:
            (ankle_roll, ankle_pitch) corrections in radians.
        """
        g = self.config.ankle_compensation_gain
        ankle_roll = g * np.arctan2(foot_normal[1], foot_normal[2])
        ankle_pitch = g * np.arctan2(-foot_normal[0], foot_normal[2])
        return float(ankle_roll), float(ankle_pitch)

    def get_com_height_adjustment(self) -> float:
        """
        Suggest an adjustment to the nominal COM height based on terrain.

        Returns:
            Height delta in meters (positive = raise COM).
        """
        if self._terrain_generator is None or self.current_mode == TerrainMode.FLAT:
            return 0.0

        if self.current_mode == TerrainMode.SLOPE:
            # Lower COM slightly on slopes for stability
            return -0.03
        if self.current_mode == TerrainMode.STAIRS:
            # Raise COM for higher knee clearance
            return 0.05

        return 0.0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _adapt_height(
        self, terrain_height: float, nominal_z: float, normal: np.ndarray
    ) -> float:
        """Return adapted foot z given terrain height."""
        slope = float(np.arccos(np.clip(normal[2], -1.0, 1.0)))
        # On rising terrain add clearance during swing (nominal_z > 0 means swing)
        if nominal_z > 0.0:
            return terrain_height + nominal_z + self.config.step_clearance
        # Stance foot: land exactly on terrain
        return terrain_height

    def _detect_terrain_mode(
        self,
        lh: float,
        rh: float,
        ln: np.ndarray,
        rn: np.ndarray,
    ) -> TerrainMode:
        """Heuristic terrain mode detection from height and normal samples."""
        avg_slope = (
            np.arccos(np.clip(ln[2], -1, 1)) + np.arccos(np.clip(rn[2], -1, 1))
        ) / 2

        if avg_slope > np.radians(5):
            # Check if slope is uniform (slope) or has discrete steps (stairs)
            if len(self._left_height_history) >= 3:
                dh = np.diff(self._left_height_history[-5:])
                if np.any(np.abs(dh) > self.config.stair_height_threshold):
                    return TerrainMode.STAIRS
            return TerrainMode.SLOPE

        # Uneven: high variance in height history
        if len(self._left_height_history) >= 5:
            variance = np.var(self._left_height_history[-5:])
            if variance > 1e-4:
                return TerrainMode.UNEVEN

        return TerrainMode.FLAT

    def _push_history(self, buf: list, value: float):
        buf.append(value)
        if len(buf) > self._history_len:
            buf.pop(0)
