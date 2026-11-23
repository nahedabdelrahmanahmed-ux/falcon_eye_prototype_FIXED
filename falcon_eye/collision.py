"""
collision.py
Collision-risk assessment and warning logic (spec section 7).

Implements:
  Target detected
    -> inside projected flight corridor?
    -> closing velocity > threshold?
    -> TTC below warning threshold?
    -> generate warning

All thresholds are configurable via CollisionRiskConfig.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import numpy as np


class RiskLevel(str, Enum):
    NONE = "none"
    ADVISORY = "advisory"
    WARNING = "warning"


@dataclass
class CollisionRiskConfig:
    corridor_half_angle_rad: float = np.radians(5.0)  # "inside flight path" cone
    closing_velocity_threshold_mps: float = 5.0
    ttc_warning_threshold_s: float = 25.0
    ttc_advisory_threshold_s: float = 45.0

    # Visibility-adaptive TTC margin (from the HEXA-TPU-RT research report, Section 3: the required
    # TTC margin widens as effective visibility shrinks, because reduced visibility shrinks the
    # reaction budget, not only the geometry). Off by default.
    visibility_margin_enabled: bool = False
    visibility_clear_m: float = 10000.0   # at/above this visibility no extra margin is applied
    visibility_min_m: float = 100.0       # at/below this visibility the full extra margin is applied
    max_margin_extra: float = 0.6         # extra margin fraction at full degradation (0.6 -> thresholds x1.6)

    # Protected-zone (closest-point-of-approach) threat test, replacing the fixed angular cone.
    # The cone ignores range: a target 300 m off the flight path is outside +-5 deg until it is ~3.4 km away
    # and then never inside it again as it closes, yet it IS a conflict. With miss_distance_mode the predicted
    # miss distance  R*|sin(bearing)|  is compared with protected_zone_m + miss_sigma_k * R * sigma_bearing.
    # ASSUMPTION: relative motion is roughly along the host's heading axis (head-on / overtaking); the
    # sensor gives no lateral velocity, so crossing traffic is NOT correctly handled by this test.
    miss_distance_mode: bool = False
    protected_zone_m: float = 500.0
    miss_sigma_k: float = 2.0


def ttc_margin_factor(config: "CollisionRiskConfig", visibility_m: Optional[float]) -> float:
    """Multiplier applied to the TTC thresholds. 1.0 when the feature is off, visibility is unknown,
    or visibility is at/above visibility_clear_m. Log-scaled in visibility (fog severity spans orders
    of magnitude) and monotonic: lower visibility -> larger factor, capped at 1 + max_margin_extra."""
    if not config.visibility_margin_enabled or visibility_m is None:
        return 1.0
    v = float(np.clip(visibility_m, config.visibility_min_m, config.visibility_clear_m))
    severity = np.log10(config.visibility_clear_m / v) / np.log10(config.visibility_clear_m / config.visibility_min_m)
    return float(1.0 + config.max_margin_extra * np.clip(severity, 0.0, 1.0))


@dataclass
class RiskAssessment:
    in_corridor: bool
    closing_velocity: float
    ttc: Optional[float]
    risk_level: RiskLevel


def assess_risk(
    estimated_range: Optional[float],
    relative_angle: Optional[float],
    relative_velocity: Optional[float],
    config: CollisionRiskConfig,
    visibility_m: Optional[float] = None,
    angle_sigma_rad: Optional[float] = None,
) -> RiskAssessment:
    """relative_velocity is defined as closing speed: positive = approaching."""
    if estimated_range is None or relative_angle is None or relative_velocity is None:
        return RiskAssessment(False, 0.0, None, RiskLevel.NONE)

    if config.miss_distance_mode:
        lateral_unc = config.miss_sigma_k * estimated_range * (angle_sigma_rad or 0.0)
        in_corridor = estimated_range * abs(np.sin(relative_angle)) <= config.protected_zone_m + lateral_unc
    else:
        in_corridor = abs(relative_angle) <= config.corridor_half_angle_rad
    closing = relative_velocity

    if closing > 1e-3:
        ttc = estimated_range / closing
    else:
        ttc = None  # not closing -> no meaningful TTC

    risk = RiskLevel.NONE
    if in_corridor and closing > config.closing_velocity_threshold_mps and ttc is not None:
        f = ttc_margin_factor(config, visibility_m)
        if ttc <= config.ttc_warning_threshold_s * f:
            risk = RiskLevel.WARNING
        elif ttc <= config.ttc_advisory_threshold_s * f:
            risk = RiskLevel.ADVISORY

    return RiskAssessment(in_corridor, closing, ttc, risk)
