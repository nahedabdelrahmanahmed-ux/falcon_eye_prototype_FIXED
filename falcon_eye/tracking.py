"""
tracking.py
Lightweight target tracking and multi-sensor fusion.

Track: a simple alpha-beta (constant-velocity) filter over range, so a
single noisy/intermittent sensor stream produces a smoothed range/velocity
estimate instead of a raw per-cycle measurement. This is intentionally
simple (not a full Kalman filter) to keep the first prototype easy to
validate, per spec section 14's "start simple" guidance.

Track management (added to fix ghost tracks / ghost warnings):
  * CONFIRMATION (M-of-N style): a track born from a detection is only
    TENTATIVE. It is not reported by is_valid() -- and therefore cannot
    drive collision-risk warnings -- until `confirm_hits` gated detections
    have been associated with it. A lone false return can no longer
    create a warning-capable track.
  * GATING: once a track exists, a detection is associated with it only if
    its range, closing-rate and bearing are consistent with the track's
    prediction. Inconsistent detections (typically false returns, which are
    uniformly random in range/angle/velocity) are rejected and count as a
    miss, so they can no longer drag a good track's state around.

Setting confirm_hits=1 and gating_enabled=False reproduces the legacy
(pre-fix) behaviour exactly; this is used for regression testing.

Fusion: combines LiDAR-like and radar measurements at a given time step
using confidence-weighted averaging when both report a CONSISTENT detection.
If both report but disagree (one is almost certainly a false return), only
the measurement that agrees with the track prediction (or, with no track,
the more confident one) is used, instead of averaging garbage into a
plausible-looking fused value. When only one sensor has a detection, it is
passed through and the track gate decides whether to accept it.
"""

from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from .sensor import SensorMeasurement


@dataclass
class AlphaBetaTrack:
    alpha: float = 0.6
    beta: float = 0.3
    range_est: Optional[float] = None
    range_rate_est: float = 0.0
    angle_est: Optional[float] = None
    initialized: bool = False
    misses_in_a_row: int = 0
    max_coast_misses: int = 5  # keep predicting through short dropouts

    # --- track management ---
    confirm_hits: int = 2                 # gated detections needed to confirm a track (2: best trade-off in benchmark)
    tentative_timeout_s: float = 3.0      # a tentative track is dropped if no gated hit arrives for this long
    gating_enabled: bool = True
    range_gate_m: float = 250.0           # base range gate (widened by gate_growth_m per miss)
    gate_growth_m: float = 15.0           # gate widening per missed update (coasting uncertainty)
    max_gate_m: float = 450.0             # cap so a long-coasting tentative track cannot admit arbitrary ghosts
    rate_gate_mps: float = 40.0           # closing-rate consistency gate
    angle_gate_rad: float = np.radians(6.0)
    hits: int = 0
    time_since_hit_s: float = 0.0
    confirmed: bool = False
    rejected_outliers: int = 0            # diagnostics: number of gated-out detections
    range_sigma_m: Optional[float] = None  # quality of the latest associated measurement (1-sigma, m)
    angle_sigma_rad: Optional[float] = None  # bearing sigma of the latest associated measurement

    # --- optional adaptive gains (scalar Kalman-style, driven by per-measurement sigma) ---
    # When enabled and the measurement carries sigma, the fixed alpha/beta are replaced by
    # K = P_pred / (P_pred + sigma_meas^2): accurate measurements (small sigma) get a large
    # gain, noisy ones a small gain, and gains fall as the track becomes certain.
    adaptive_gain: bool = False
    q_range_m: float = 2.0                # range process noise, m per sqrt(s) (manoeuvre allowance)
    q_rate_mps: float = 3.0               # closing-rate process noise, (m/s) per sqrt(s) (~0.3 g acceleration allowance; 1.0 was fine for
                                          # constant-velocity targets but 3.0 was more robust to accelerating targets in stress tests)
    range_var: Optional[float] = None
    rate_var: Optional[float] = None

    # ------------------------------------------------------------------
    def _birth(self, meas: SensorMeasurement):
        self.range_est = meas.estimated_range
        self.angle_est = meas.relative_angle
        self.range_sigma_m = meas.range_sigma_m
        self.angle_sigma_rad = meas.angle_sigma_rad
        self.range_var = (meas.range_sigma_m ** 2) if meas.range_sigma_m is not None else None
        self.rate_var = (meas.velocity_sigma_mps ** 2) if meas.velocity_sigma_mps is not None else None
        self.range_rate_est = -(meas.relative_velocity or 0.0)
        self.initialized = True
        self.misses_in_a_row = 0
        self.time_since_hit_s = 0.0
        self.hits = 1
        self.confirmed = self.hits >= self.confirm_hits

    def _drop(self):
        self.initialized = False
        self.range_est = None
        self.angle_est = None
        self.range_rate_est = 0.0
        self.hits = 0
        self.time_since_hit_s = 0.0
        self.confirmed = False
        self.range_var = None
        self.rate_var = None

    def _passes_gate(self, meas: SensorMeasurement, pred_range: float) -> bool:
        if not self.gating_enabled:
            return True
        gate = min(self.range_gate_m + self.gate_growth_m * self.misses_in_a_row, max(self.max_gate_m, self.range_gate_m))
        if abs(meas.estimated_range - pred_range) > gate:
            return False
        if meas.relative_velocity is not None and \
                abs(meas.relative_velocity - self.closing_velocity()) > self.rate_gate_mps:
            return False
        if meas.relative_angle is not None and self.angle_est is not None and \
                abs(meas.relative_angle - self.angle_est) > self.angle_gate_rad:
            return False
        return True

    def predicted_range(self) -> Optional[float]:
        """Latest range estimate of a live (tentative or confirmed) track, used
        by fusion as the reference for resolving disagreeing sensors."""
        return self.range_est if self.initialized else None

    def update(self, meas: Optional[SensorMeasurement], dt: float):
        """meas=None or non-detected measurement -> coast (predict only)."""
        detected = meas is not None and meas.target_detected

        if not self.initialized:
            if detected:
                self._birth(meas)
            return

        # Predict
        pred_range = self.range_est + self.range_rate_est * dt if self.range_est is not None else None
        self.time_since_hit_s += dt
        if self.adaptive_gain and self.range_var is not None and self.rate_var is not None:   # variance prediction (every step, incl. coasting)
            self.range_var += self.rate_var * dt * dt + (self.q_range_m ** 2) * dt
            self.rate_var += (self.q_rate_mps ** 2) * dt

        associated = detected and pred_range is not None and self._passes_gate(meas, pred_range)

        if associated:
            residual = meas.estimated_range - pred_range
            measured_range_rate = -(meas.relative_velocity or 0.0)
            use_adaptive = (self.adaptive_gain and self.range_var is not None and self.rate_var is not None
                            and meas.range_sigma_m is not None and meas.velocity_sigma_mps is not None)
            if use_adaptive:
                k_r = self.range_var / (self.range_var + meas.range_sigma_m ** 2)
                k_v = self.rate_var / (self.rate_var + meas.velocity_sigma_mps ** 2)
                self.range_est = pred_range + k_r * residual
                self.range_rate_est = self.range_rate_est + k_v * (measured_range_rate - self.range_rate_est)
                self.range_var = (1.0 - k_r) * self.range_var
                self.rate_var = (1.0 - k_v) * self.rate_var
            else:
                self.range_est = pred_range + self.alpha * residual
                # Rate estimate: low-pass filter the sensor's own directly-measured
                # relative_velocity rather than differencing noisy range residuals
                # over a fixed dt (differencing amplifies range noise by 1/dt).
                self.range_rate_est = (1.0 - self.beta) * self.range_rate_est + self.beta * measured_range_rate
            self.angle_est = meas.relative_angle
            self.range_sigma_m = meas.range_sigma_m
            self.angle_sigma_rad = meas.angle_sigma_rad
            self.misses_in_a_row = 0
            self.time_since_hit_s = 0.0
            self.hits += 1
            if self.hits >= self.confirm_hits:
                self.confirmed = True
            return

        # ---- not associated ----
        if detected:
            self.rejected_outliers += 1
            if not self.confirmed and self.hits <= 1:
                # A lone, never-corroborated return that is contradicted by a new
                # one: the old one is as likely to be the ghost, so restart from
                # the new return. A tentative track that already has >=2 mutually
                # consistent hits is NOT overwritten by one random return.
                self._birth(meas)
                return

        self.misses_in_a_row += 1
        if self.confirmed:
            alive = self.misses_in_a_row <= self.max_coast_misses
        else:
            alive = self.time_since_hit_s <= self.tentative_timeout_s
        if alive:
            self.range_est = pred_range
        else:
            self._drop()  # lost track

    def closing_velocity(self) -> float:
        return -self.range_rate_est

    def is_valid(self) -> bool:
        """True only for CONFIRMED tracks (tentative tracks must never drive warnings)."""
        return self.initialized and self.confirmed and self.range_est is not None

    def is_tentative(self) -> bool:
        return self.initialized and not self.confirmed


def measurements_consistent(
    lidar_meas: SensorMeasurement,
    radar_meas: SensorMeasurement,
    range_gate_m: float = 200.0,
    rate_gate_mps: float = 40.0,
    angle_gate_rad: float = np.radians(6.0),
) -> bool:
    """True if two DETECTED measurements agree on range, closing rate and bearing (same object).
    Persistent disagreement is also a sensing-health signal (one sensor biased or producing false returns)."""
    return bool(
        abs(lidar_meas.estimated_range - radar_meas.estimated_range) <= range_gate_m
        and abs((lidar_meas.relative_velocity or 0.0) - (radar_meas.relative_velocity or 0.0)) <= rate_gate_mps
        and abs((lidar_meas.relative_angle or 0.0) - (radar_meas.relative_angle or 0.0)) <= angle_gate_rad
    )


def fuse_measurements(
    lidar_meas: Optional[SensorMeasurement],
    radar_meas: Optional[SensorMeasurement],
    reference_range: Optional[float] = None,
    range_gate_m: float = 200.0,
    rate_gate_mps: float = 40.0,
    angle_gate_rad: float = np.radians(6.0),
    consistency_check: bool = True,
    weighting: str = "inverse_variance",
) -> Optional[SensorMeasurement]:
    """Confidence-weighted fusion of two independent sensor measurements
    taken at (approximately) the same time step, with a consistency check.

    consistency_check=False reproduces the legacy behaviour (always average).

    weighting="inverse_variance" (default): each channel (range, angle, closing rate) is
    weighted by 1/sigma^2 using the sigma carried by each measurement, giving
    sigma_fused = 1/sqrt(sum 1/sigma^2) <= min(sigma). Detection `confidence` is NOT used
    for weighting (it says whether a target was seen, not how accurate the estimate is);
    it remains the detection-quality signal. Falls back to confidence weighting when a
    measurement carries no sigma. weighting="confidence" reproduces the earlier scheme."""
    l_ok = lidar_meas is not None and lidar_meas.target_detected
    r_ok = radar_meas is not None and radar_meas.target_detected

    if not l_ok and not r_ok:
        return None

    if l_ok and not r_ok:
        return lidar_meas
    if r_ok and not l_ok:
        return radar_meas

    # Both report a detection. Do they describe the same object?
    if consistency_check:
        consistent = measurements_consistent(lidar_meas, radar_meas, range_gate_m, rate_gate_mps, angle_gate_rad)
        if not consistent:
            # One of them is (almost certainly) a false return. Averaging would
            # fabricate a plausible-looking but wrong measurement, so pick one.
            if reference_range is not None:
                dl = abs(lidar_meas.estimated_range - reference_range)
                dr = abs(radar_meas.estimated_range - reference_range)
                return lidar_meas if dl <= dr else radar_meas
            return lidar_meas if lidar_meas.confidence >= radar_meas.confidence else radar_meas

    # Consistent pair: combine.
    def _iv(a, b, sa, sb):
        """inverse-variance combination -> (value, sigma_fused)"""
        wa, wb = 1.0 / (sa * sa), 1.0 / (sb * sb)
        return (a * wa + b * wb) / (wa + wb), float(1.0 / np.sqrt(wa + wb))

    wl = max(lidar_meas.confidence, 1e-3)
    wr = max(radar_meas.confidence, 1e-3)
    fused_confidence = min(1.0, (wl + wr) / 2.0 + 0.15)  # agreement boosts confidence
    fused_false = lidar_meas.is_false_detection and radar_meas.is_false_detection

    use_iv = (weighting == "inverse_variance"
              and None not in (lidar_meas.range_sigma_m, radar_meas.range_sigma_m,
                               lidar_meas.angle_sigma_rad, radar_meas.angle_sigma_rad,
                               lidar_meas.velocity_sigma_mps, radar_meas.velocity_sigma_mps))
    if use_iv:
        fused_range, s_r = _iv(lidar_meas.estimated_range, radar_meas.estimated_range,
                               lidar_meas.range_sigma_m, radar_meas.range_sigma_m)
        fused_angle, s_a = _iv(lidar_meas.relative_angle, radar_meas.relative_angle,
                               lidar_meas.angle_sigma_rad, radar_meas.angle_sigma_rad)
        fused_vel, s_v = _iv(lidar_meas.relative_velocity, radar_meas.relative_velocity,
                             lidar_meas.velocity_sigma_mps, radar_meas.velocity_sigma_mps)
    else:
        total = wl + wr
        fused_range = (lidar_meas.estimated_range * wl + radar_meas.estimated_range * wr) / total
        fused_angle = (lidar_meas.relative_angle * wl + radar_meas.relative_angle * wr) / total
        fused_vel = (lidar_meas.relative_velocity * wl + radar_meas.relative_velocity * wr) / total
        s_r = s_a = s_v = None

    return SensorMeasurement(
        target_detected=True,
        estimated_range=fused_range,
        relative_angle=fused_angle,
        relative_velocity=fused_vel,
        confidence=fused_confidence,
        is_false_detection=fused_false,
        latency=max(lidar_meas.latency, radar_meas.latency),
        range_sigma_m=s_r, angle_sigma_rad=s_a, velocity_sigma_mps=s_v,
    )
