"""
health.py
Sensing-health monitor: tells the operator when ABSENCE OF AN ALERT CANNOT BE TRUSTED.

Why: a detect-and-avoid function that is silent because the sky is empty looks exactly like one that is silent
because its sensors cannot see. In fog, a LiDAR returns nothing at ranges where traffic may well exist. Without an
explicit health state, "no warning" is indistinguishable from "blind". This module adds that state, independent of
the risk level.

State  (NOMINAL / DEGRADED / BLIND) is derived per tick from three evidence sources
  1. DATA FAULTS      a sensor delivers failed/invalid cycles, or goes silent (stale)           -> sensor FAULT
  2. CAPABILITY       given the CURRENT visibility, can the sensor acquire a track at the range a warning needs?
                      (computed from the sensor's own detection model: P(confirm within a window) >= confidence)
                      -> sensor INCAPABLE when its effective range < required range
  3. CONSISTENCY      persistent LiDAR/radar disagreement (one is biased or producing false returns)
plus a TRACK_LOST_THREAT flag when a track that was an advisory/warning disappears (the usual way a real conflict
vanishes silently).

Per sensor class:  FAULT (stale / failing)  |  INCAPABLE (effective range < reduced_fraction of required)
                   REDUCED (reduced_fraction <= effective/required < 1)  |  OK (effective >= required, no fault)
Capability needs BOTH acquiring a track (P(confirm within the window) >= confidence) AND holding it (P(>=1 detection
within the tracker's coast window) >= confidence): a sensor that can start a track but not keep it flickers.

System state = "can the system warn at the design encounter?"
  NOMINAL   at least required_ok_sensors (default 1) sensors are OK
  DEGRADED  none OK, but at least one REDUCED (it works, but not out to the required range)
  BLIND     none OK and none REDUCED (all faulted or incapable)
  Losing a redundant sensor is reported as the info reason REDUNDANCY_LOST without lowering the state.
  Persistent sensor disagreement or a lost threat track raises NOMINAL to at least DEGRADED.

IMPORTANT LIMITS
  * "Capable" is defined against a DESIGN ENCOUNTER (design_closing_speed_mps x ttc_required_s), an explicit
    assumption: change it and the state changes. A faster encounter needs a longer range.
  * The capability model is only as good as the sensor model. With real sensors, calibrate it (falcon_eye.calibrate).
  * Silent faults (e.g. a bias common to both sensors) are NOT detectable from these signals.
  * This module only ANNUNCIATES. It never changes the risk level.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from math import comb
from typing import Dict, Optional, Tuple

import numpy as np

from .collision import CollisionRiskConfig, ttc_margin_factor
from .fog import FogScenario


class SensingState(str, Enum):
    NOMINAL = "nominal"
    DEGRADED = "degraded"
    BLIND = "blind"


@dataclass
class SensorObs:
    """One sensor cycle as seen by the monitor. `failed` = the cycle arrived but is a hardware failure / invalid data."""
    failed: bool = False
    age_s: Optional[float] = None        # data age / latency of this cycle, if known (None = unknown)


@dataclass
class HealthConfig:
    # design encounter that a warning must be able to support
    design_closing_speed_mps: float = 100.0
    ttc_required_s: float = 25.0
    # acquisition capability: P(>= confirm_hits detections within the window) must reach this confidence
    acquisition_window_s: float = 2.0
    acquisition_confidence: float = 0.9
    confirm_hits: int = 2
    # holding a track: the tracker coasts only a few cycles, so a detection must arrive within this window
    maintenance_window_s: float = 0.5
    maintenance_confidence: float = 0.9
    reduced_fraction: float = 0.25       # effective/required below this = INCAPABLE (not merely REDUCED)
    required_ok_sensors: int = 1         # sensors that must be OK for NOMINAL (set 2 to demand redundancy)
    cycle_rate_hz: Dict[str, float] = field(default_factory=lambda: {"lidar": 10.0, "radar": 5.0})
    # data faults
    fault_window_s: float = 1.0
    fault_fraction: float = 0.5          # failed cycles / cycles within the window
    min_cycles_for_fraction: int = 3
    stale_timeout_s: float = 0.5         # no good cycle for this long = stale (silent) sensor
    max_data_age_s: float = 0.5          # a cycle older than this is unusable for DAA (closing 100 m/s x 0.5 s = 50 m)
    # consistency between sensors
    disagree_window: int = 10
    disagree_fraction: float = 0.5
    disagree_min_events: int = 6
    # lost threat track
    track_lost_hold_s: float = 3.0
    heartbeat_s: float = 0.2             # replay: synthetic ticks inserted across data gaps


@dataclass
class HealthReport:
    state: SensingState
    reasons: Tuple[str, ...]
    sensor_ok: Dict[str, bool]
    sensor_class: Dict[str, str]
    required_range_m: float
    effective_range_m: Dict[str, Optional[float]]
    capability_known: bool


def acquisition_probability(p_cycle: float, n_cycles: int, confirm_hits: int = 2) -> float:
    """P(at least `confirm_hits` detections in `n_cycles` independent cycles)."""
    p = min(max(p_cycle, 0.0), 1.0)
    return 1.0 - sum(comb(n_cycles, k) * p ** k * (1.0 - p) ** (n_cycles - k) for k in range(confirm_hits))


class SensingHealthMonitor:
    def __init__(self, sensors: Dict[str, object], cfg: Optional[HealthConfig] = None,
                 collision_cfg: Optional[CollisionRiskConfig] = None):
        """sensors: {'lidar': FalconEyeSensor, 'radar': RadarSensor}; the expected set. Each must provide
        detection_probability(range, angle, fog) and max_range_m."""
        self.sensors = dict(sensors)
        self.cfg = cfg or HealthConfig()
        self.collision_cfg = collision_cfg or CollisionRiskConfig()
        self._cycles = {n: deque() for n in self.sensors}          # (t, failed, too_old)
        self._last_ok = {n: None for n in self.sensors}
        self._t0: Optional[float] = None
        self._pairs = deque(maxlen=self.cfg.disagree_window)       # True = inconsistent
        self._prev_track_valid = False
        self._prev_threat = False
        self._lost_until = -1.0
        self._reff_cache: Dict[tuple, float] = {}

    # ---------------- capability ----------------
    def required_range_m(self, visibility_m: Optional[float] = None) -> float:
        f = ttc_margin_factor(self.collision_cfg, visibility_m)
        return self.cfg.design_closing_speed_mps * self.cfg.ttc_required_s * f

    def effective_range_m(self, name: str, fog: FogScenario, extra_interference: float = 0.0) -> float:
        """min(range at which a track can be ACQUIRED, range at which it can be HELD) with the configured confidences."""
        key = (name, round(fog.visibility_m, 1), round(fog.noise_multiplier, 3), fog.wavelength_nm, fog.extinction_model,
               fog.extinction_scale, round(extra_interference, 3))
        if key in self._reff_cache:
            return self._reff_cache[key]
        s = self.sensors[name]
        rate = self.cfg.cycle_rate_hz.get(name, 10.0)
        n_acq = max(1, int(round(self.cfg.acquisition_window_s * rate)))
        n_hold = max(1, int(round(self.cfg.maintenance_window_s * rate)))
        p = lambda r: s.detection_probability(r, 0.0, fog, extra_interference)
        acq_ok = lambda r: acquisition_probability(p(r), n_acq, self.cfg.confirm_hits) >= self.cfg.acquisition_confidence
        hold_ok = lambda r: acquisition_probability(p(r), n_hold, 1) >= self.cfg.maintenance_confidence
        both = lambda r: acq_ok(r) and hold_ok(r)
        lo, hi = 0.0, float(s.max_range_m)
        if not both(1.0):
            res = 0.0
        elif both(hi - 1e-6):
            res = hi
        else:
            for _ in range(50):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if both(mid) else (lo, mid)
            res = lo
        self._reff_cache[key] = res
        return res

    # ---------------- update ----------------
    def _fault(self, name: str, t: float) -> Optional[str]:
        """FAULT/DATA_AGE: cycles ARE arriving but are failing or too old. STALE: no good cycle for too long
        (silence). The first is checked first because it is the more specific diagnosis."""
        cyc = self._cycles[name]
        while cyc and t - cyc[0][0] > self.cfg.fault_window_s:
            cyc.popleft()
        if len(cyc) >= self.cfg.min_cycles_for_fraction and sum(f for _, f, _ in cyc) / len(cyc) >= self.cfg.fault_fraction:
            old_share = sum(o for _, _, o in cyc) / len(cyc)
            return f"SENSOR_DATA_AGE:{name}" if old_share >= self.cfg.fault_fraction else f"SENSOR_FAULT:{name}"
        last = self._last_ok[name] if self._last_ok[name] is not None else self._t0
        if t - last > self.cfg.stale_timeout_s:
            return f"SENSOR_STALE:{name}"
        return None

    def update(self, t: float, obs: Dict[str, Optional[SensorObs]], fog: Optional[FogScenario] = None,
               disagree: Optional[bool] = None, track_valid: bool = False, threat_active: bool = False,
               extra_interference: float = 0.0) -> HealthReport:
        """obs[name] is None when NO cycle arrived from that sensor this tick; a SensorObs otherwise.
        disagree: True/False when both sensors detected this tick (inconsistent / consistent), None otherwise."""
        if self._t0 is None:
            self._t0 = t
        for n in self.sensors:
            o = obs.get(n)
            if o is not None:
                old = o.age_s is not None and o.age_s > self.cfg.max_data_age_s
                bad = bool(o.failed) or old
                self._cycles[n].append((t, bad, old))
                if not bad:
                    self._last_ok[n] = t
        if disagree is not None:
            self._pairs.append(bool(disagree))
        reasons, sensor_ok, sensor_class = [], {}, {}
        reff: Dict[str, Optional[float]] = {}
        required = self.required_range_m(fog.visibility_m if fog is not None else None)
        for n in self.sensors:
            f = self._fault(n, t)
            cls = "ok"
            if f:
                reasons.append(f); cls = "fault"
            if fog is not None:
                r = self.effective_range_m(n, fog, extra_interference); reff[n] = r
                if cls != "fault":
                    if r < self.cfg.reduced_fraction * required:
                        cls = "incapable"
                    elif r < required:
                        cls = "reduced"
                    if cls != "ok":
                        reasons.append(f"SENSOR_{cls.upper()}:{n}(effective {r:.0f} m vs required {required:.0f} m)")
            else:
                reff[n] = None
            sensor_class[n] = cls; sensor_ok[n] = (cls == "ok")
        n_ok, n_exp = sum(sensor_ok.values()), len(self.sensors)
        n_reduced = sum(c == "reduced" for c in sensor_class.values())
        if n_ok >= min(self.cfg.required_ok_sensors, n_exp):
            state = SensingState.NOMINAL
        elif n_ok + n_reduced >= 1:
            state = SensingState.DEGRADED
        else:
            state = SensingState.BLIND
        if 0 < n_ok < n_exp:
            reasons.append("REDUNDANCY_LOST")
        if len(self._pairs) >= self.cfg.disagree_min_events and sum(self._pairs) / len(self._pairs) >= self.cfg.disagree_fraction:
            reasons.append("SENSOR_DISAGREEMENT")
            if state == SensingState.NOMINAL:
                state = SensingState.DEGRADED
        if self._prev_track_valid and not track_valid and self._prev_threat:
            self._lost_until = t + self.cfg.track_lost_hold_s
        if t <= self._lost_until:
            reasons.append("TRACK_LOST_THREAT")
            if state == SensingState.NOMINAL:
                state = SensingState.DEGRADED
        self._prev_track_valid, self._prev_threat = bool(track_valid), bool(threat_active)
        return HealthReport(state, tuple(reasons), sensor_ok, sensor_class, required, reff, fog is not None)
