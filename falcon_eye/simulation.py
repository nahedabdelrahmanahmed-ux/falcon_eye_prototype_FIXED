"""
simulation.py
Single-scenario simulation engine.

Ties together: Aircraft A (host) + Aircraft B (target), FalconEyeSensor
(and optionally RadarSensor), FogScenario, FailureConfig, AlphaBetaTrack,
and the collision-risk/warning logic, producing a time series suitable for
plotting and metric extraction.
"""

from dataclasses import dataclass, field
from typing import Optional, List
import numpy as np

from .aircraft import Aircraft, AircraftState
from .sensor import FalconEyeSensor, SensorMeasurement
from .radar import RadarSensor
from .fog import FogScenario, get_fog_scenario
from .collision import CollisionRiskConfig, RiskLevel, assess_risk
from .failures import FailureConfig
from .fogdata import FogProfile
from .health import HealthConfig, SensingHealthMonitor, SensorObs
from .tracking import AlphaBetaTrack, fuse_measurements, measurements_consistent


def relative_geometry(host: Aircraft, target: Aircraft):
    """Compute true range, bearing (relative to host heading), and closing
    velocity between host and target."""
    rel_pos = target.position() - host.position()
    true_range = float(np.linalg.norm(rel_pos))

    bearing_to_target = np.arctan2(rel_pos[1], rel_pos[0])
    relative_angle = bearing_to_target - host.state.heading
    # wrap to [-pi, pi]
    relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

    rel_vel = target.velocity() - host.velocity()
    if true_range > 1e-6:
        los_unit = rel_pos / true_range
        closing_velocity = -float(np.dot(rel_vel, los_unit))  # positive = closing
    else:
        closing_velocity = 0.0

    return true_range, relative_angle, closing_velocity


@dataclass
class SimulationConfig:
    duration_s: float = 120.0
    dt_s: float = 0.1
    fog_key: str = "clear"
    # Optional time-varying fog (e.g. built from real visibility observations); overrides fog_key when set.
    fog_profile: Optional[FogProfile] = None
    mode: str = "lidar"  # "lidar" | "radar" | "fusion"
    failure: FailureConfig = field(default_factory=FailureConfig)
    collision_config: CollisionRiskConfig = field(default_factory=CollisionRiskConfig)
    seed: Optional[int] = None
    # Optional sensing-health monitor (annunciation only; never changes the risk level). None = off.
    health: Optional[HealthConfig] = None


@dataclass
class SimulationResult:
    time: np.ndarray
    true_range: np.ndarray
    true_ttc: np.ndarray
    tracked_range: np.ndarray
    detected: np.ndarray
    false_detection: np.ndarray
    risk_level: List[str]
    warning_time: Optional[float]
    collision_time: Optional[float]
    min_true_range: float
    first_detection_time: Optional[float]
    first_detection_range: Optional[float]
    # Added for honest accuracy metrics: the closing speed the TRACKER actually used for
    # risk assessment, and the true closing speed (None in results built elsewhere).
    tracked_closing: Optional[np.ndarray] = None
    true_closing: Optional[np.ndarray] = None
    sensing_state: Optional[List[str]] = None            # per step: nominal | degraded | blind (None if monitor off)
    sensing_reasons: Optional[List[tuple]] = None


def run_scenario(host: Aircraft, target: Aircraft, config: SimulationConfig) -> SimulationResult:
    rng = np.random.default_rng(config.seed)
    fog = get_fog_scenario(config.fog_key)

    lidar = FalconEyeSensor(rng=np.random.default_rng(rng.integers(1e9)))
    radar = RadarSensor(rng=np.random.default_rng(rng.integers(1e9)))

    track = AlphaBetaTrack()
    monitor = None
    if config.health is not None:
        _sens = {n: s for n, s in (("lidar", lidar), ("radar", radar)) if config.mode in (n, "fusion")}
        monitor = SensingHealthMonitor(_sens, config.health, config.collision_config)
    sensing_states: List[str] = []
    sensing_reasons: List[tuple] = []
    prev_threat = False

    n_steps = int(config.duration_s / config.dt_s)
    time = np.zeros(n_steps)
    true_range_arr = np.zeros(n_steps)
    true_ttc_arr = np.full(n_steps, np.nan)
    tracked_range_arr = np.full(n_steps, np.nan)
    tracked_closing_arr = np.full(n_steps, np.nan)
    true_closing_arr = np.zeros(n_steps)
    detected_arr = np.zeros(n_steps, dtype=bool)
    false_arr = np.zeros(n_steps, dtype=bool)
    risk_levels: List[str] = []

    warning_time = None
    first_detection_time = None
    first_detection_range = None
    collision_time = None
    min_true_range = np.inf

    lidar_acc = 0.0
    radar_acc = 0.0

    t = 0.0
    for i in range(n_steps):
        true_range, true_angle, closing_vel = relative_geometry(host, target)
        min_true_range = min(min_true_range, true_range)
        true_range_arr[i] = true_range
        true_ttc = true_range / closing_vel if closing_vel > 1e-3 else np.nan
        true_ttc_arr[i] = true_ttc
        true_closing_arr[i] = closing_vel
        if collision_time is None and true_range <= max(host.state.size, target.state.size):
            collision_time = t

        fog_now = config.fog_profile.at(t) if config.fog_profile is not None else fog
        extra_fail_p = config.failure.extra_sensor_failure_probability(t)
        extra_interf = config.failure.extra_interference(t)

        meas_for_track: Optional[SensorMeasurement] = None

        lidar_acc += config.dt_s
        radar_acc += config.dt_s
        lidar_meas = None
        radar_meas = None

        if config.mode in ("lidar", "fusion") and lidar_acc >= lidar.dt():
            lidar_acc = 0.0
            lidar_meas = lidar.measure(true_range, true_angle, closing_vel, fog_now,
                                        extra_fail_p, extra_interf)
            lidar_meas = config.failure.apply_post_measurement(t, lidar_meas, rng)

        if config.mode in ("radar", "fusion") and radar_acc >= radar.dt():
            radar_acc = 0.0
            radar_meas = radar.measure(true_range, true_angle, closing_vel, fog_now,
                                        extra_fail_p, extra_interf)
            radar_meas = config.failure.apply_post_measurement(t, radar_meas, rng)

        if config.mode == "lidar":
            meas_for_track = lidar_meas
        elif config.mode == "radar":
            meas_for_track = radar_meas
        elif config.mode == "fusion":
            if lidar_meas is not None or radar_meas is not None:
                meas_for_track = fuse_measurements(lidar_meas, radar_meas,
                                                   reference_range=track.predicted_range())

        if meas_for_track is not None:
            track.update(meas_for_track, config.dt_s)
            if meas_for_track.target_detected:
                detected_arr[i] = True
                if meas_for_track.is_false_detection:
                    false_arr[i] = True
                elif first_detection_time is None:
                    first_detection_time = t
                    first_detection_range = meas_for_track.estimated_range
        else:
            track.update(None, config.dt_s)

        tracked_range_arr[i] = track.range_est if track.is_valid() else np.nan
        tracked_closing_arr[i] = track.closing_velocity() if track.is_valid() else np.nan

        if track.is_valid():
            risk = assess_risk(track.range_est, track.angle_est,
                                track.closing_velocity(), config.collision_config,
                                visibility_m=fog_now.visibility_m, angle_sigma_rad=track.angle_sigma_rad)
        else:
            risk = assess_risk(None, None, None, config.collision_config)

        risk_levels.append(risk.risk_level.value)
        if monitor is not None:
            _obs = {}
            if config.mode in ("lidar", "fusion"):
                _obs["lidar"] = None if lidar_meas is None else SensorObs(failed=lidar_meas.sensor_failed, age_s=lidar_meas.latency)
            if config.mode in ("radar", "fusion"):
                _obs["radar"] = None if radar_meas is None else SensorObs(failed=radar_meas.sensor_failed, age_s=radar_meas.latency)
            _dis = None
            if lidar_meas is not None and radar_meas is not None and lidar_meas.target_detected and radar_meas.target_detected:
                _dis = not measurements_consistent(lidar_meas, radar_meas)
            _rep = monitor.update(t, _obs, fog_now, _dis, track.is_valid(), prev_threat, extra_interf)
            sensing_states.append(_rep.state.value); sensing_reasons.append(_rep.reasons)
            prev_threat = risk.risk_level in (RiskLevel.ADVISORY, RiskLevel.WARNING)
        if warning_time is None and risk.risk_level == RiskLevel.WARNING:
            warning_time = t

        time[i] = t
        host.step(config.dt_s)
        target.step(config.dt_s)
        t += config.dt_s

    return SimulationResult(
        time=time, true_range=true_range_arr, true_ttc=true_ttc_arr,
        tracked_range=tracked_range_arr, detected=detected_arr, false_detection=false_arr,
        risk_level=risk_levels, warning_time=warning_time, collision_time=collision_time,
        min_true_range=float(min_true_range),
        first_detection_time=first_detection_time, first_detection_range=first_detection_range,
        tracked_closing=tracked_closing_arr, true_closing=true_closing_arr,
        sensing_state=sensing_states if monitor is not None else None,
        sensing_reasons=sensing_reasons if monitor is not None else None,
    )


def make_head_on_scenario(
    initial_range_m: float = 6000.0,
    closing_speed_mps: float = 180.0,
    lateral_offset_m: float = 0.0,
    target_size_m: float = 15.0,
) -> (Aircraft, Aircraft):
    """Convenience builder: host at origin flying +x, target approaching
    head-on (or with a lateral offset to test corridor logic)."""
    host = Aircraft(
        name="Aircraft A (host)",
        state=AircraftState(x=0.0, y=0.0, z=3000.0, heading=0.0, speed=90.0, size=15.0),
        role="host",
    )
    target = Aircraft(
        name="Aircraft B (target)",
        state=AircraftState(
            x=initial_range_m, y=lateral_offset_m, z=3000.0,
            heading=np.pi, speed=closing_speed_mps - 90.0, size=target_size_m,
        ),
        role="target",
    )
    return host, target
