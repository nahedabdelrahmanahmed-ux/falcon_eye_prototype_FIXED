"""
replay.py
Run the Falcon Eye tracking / fusion / risk pipeline on RECORDED sensor data instead of simulated
sensors, and score it against a reference ("truth") trajectory, per visibility band.

Input files (CSV, headers case-insensitive; see docs/REAL_DATA_GUIDE.md)
------------------------------------------------------------------------
Measurement log (one row per sensor cycle, detected or not):
    time_s, sensor ('lidar'|'radar'), detected (0/1),
    range_m, angle_rad (or angle_deg), closing_speed_mps (positive = closing; or range_rate_mps, negative = closing)
    optional: confidence, range_sigma_m, angle_sigma_rad, velocity_sigma_mps, episode
Truth (reference track, e.g. ADS-B / RTK-GPS of the target relative to the host):
    time_s, true_range_m, true_closing_mps, optional: true_angle_rad|true_angle_deg, target_present, episode

`episode` separates independent passes/flights (the tracker is reset between episodes). Timestamps must
share one clock across files (synchronisation error directly becomes range error: closing speed x offset).

Everything is scored only against what the truth file says. Rows that fail quality control are counted and
reported, never silently fixed.
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import pandas as pd

from .collision import CollisionRiskConfig, RiskLevel, assess_risk
from .fog import FogScenario
from .fogdata import FogCalibration, VisibilitySeries, load_metar_file, load_visibility_csv
from .health import HealthConfig, SensingHealthMonitor, SensorObs
from .radar import RadarSensor
from .sensor import FalconEyeSensor
from .sensor import SensorMeasurement
from .tracking import AlphaBetaTrack, fuse_measurements, measurements_consistent

VIS_BINS_M = (0.0, 200.0, 800.0, 3000.0, 10000.0, np.inf)
_SENSORS = ("lidar", "radar")


@dataclass
class ReplayConfig:
    collision: CollisionRiskConfig = field(default_factory=CollisionRiskConfig)
    track_kwargs: dict = field(default_factory=dict)
    weighting: str = "inverse_variance"
    mode: str = "auto"                      # auto | lidar | radar | fusion
    cycle_tol_s: float = 0.02               # measurements closer than this share one fusion tick
    first_dt_s: float = 0.1
    accuracy_gate_m: float = 500.0          # |range error| above this = lost/ghost, excluded from RMSE
    protected_zone_m: float = 500.0         # truth miss distance below this = a real threat
    # sigma used when the log has none (None -> fusion falls back to confidence weighting)
    default_sigma: Optional[Dict[str, tuple]] = None   # {'lidar': (range_m, angle_rad, vel_mps), 'radar': (...)}
    vis_bins_m: tuple = VIS_BINS_M
    # Sensing-health monitor (annunciation only). None = off. When on, heartbeat ticks are inserted across data gaps
    # so total data loss is reported instead of silently producing no output.
    health: Optional[HealthConfig] = None
    health_sensors: Optional[Dict[str, object]] = None   # sensor models used for the capability estimate (default: project models;
                                                         # for real data pass models with the REAL sensor's max range / FOV)
    fog_calibration: Optional[FogCalibration] = None     # visibility -> sensor degradation for the capability estimate


# --------------------------------------------------------------------------------------
# loading + quality control
# --------------------------------------------------------------------------------------
def _lower(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy(); df.columns = [c.strip().lower() for c in df.columns]; return df


def load_measurements(path, max_range_m: float = 100000.0) -> (pd.DataFrame, dict):
    df = _lower(pd.read_csv(path, comment="#", skipinitialspace=True))
    qc = {"n_rows": int(len(df)), "warnings": []}
    for c in ("time_s", "sensor", "detected"):
        if c not in df.columns:
            raise ValueError(f"measurement log is missing required column '{c}' (has {list(df.columns)})")
    df["sensor"] = df["sensor"].astype(str).str.strip().str.lower()
    bad = ~df["sensor"].isin(_SENSORS)
    if bad.any():
        raise ValueError(f"unknown sensor names {sorted(df.loc[bad, 'sensor'].unique())}; allowed: {_SENSORS}")
    for c in df.columns:
        if c != "sensor":
            df[c] = pd.to_numeric(df[c], errors="coerce")
    if "angle_rad" not in df.columns:
        if "angle_deg" in df.columns:
            df["angle_rad"] = np.radians(df["angle_deg"])
        else:
            raise ValueError("measurement log needs angle_rad or angle_deg")
    if "closing_speed_mps" not in df.columns:
        if "range_rate_mps" in df.columns:
            df["closing_speed_mps"] = -df["range_rate_mps"]
        else:
            raise ValueError("measurement log needs closing_speed_mps (or range_rate_mps, negative = closing)")
    if "episode" not in df.columns:
        df["episode"] = 0
    df["episode"] = df["episode"].fillna(0).astype(int)

    miss_t = df["time_s"].isna() | df["detected"].isna()
    qc["n_dropped_missing_time_or_flag"] = int(miss_t.sum()); df = df[~miss_t]
    det = df["detected"] > 0.5
    invalid = det & (df["range_m"].isna() | df["angle_rad"].isna() | df["closing_speed_mps"].isna()
                     | (df["range_m"] <= 0) | (df["range_m"] > max_range_m))
    qc["n_dropped_invalid_detections"] = int(invalid.sum())
    # Invalid rows are KEPT as failed, non-detecting cycles (flag `invalid`): they are evidence of a sensor problem for
    # the health monitor, but they never count as detections or as clean cycles in calibration.
    df["invalid"] = invalid.astype(int)
    df.loc[invalid, "detected"] = 0
    qc["n_out_of_order"] = int((np.diff(df["time_s"].to_numpy()) < 0).sum())
    df = df.sort_values(["episode", "time_s", "sensor"], kind="stable")
    dup = df.duplicated(["episode", "time_s", "sensor"], keep="last")
    qc["n_duplicate_rows"] = int(dup.sum()); df = df[~dup]
    df["detected"] = (df["detected"] > 0.5).astype(int)
    qc["n_valid"] = int((df["invalid"] == 0).sum()); qc["sensors"] = sorted(df["sensor"].unique().tolist())
    qc["n_detections"] = int(df["detected"].sum())
    if qc["n_dropped_invalid_detections"]:
        qc["warnings"].append("detected rows with missing/invalid range, angle or speed were dropped")
    return df.reset_index(drop=True), qc


def load_truth(path) -> (pd.DataFrame, dict):
    df = _lower(pd.read_csv(path, comment="#", skipinitialspace=True))
    qc = {"n_rows": int(len(df)), "warnings": []}
    for c in ("time_s", "true_range_m", "true_closing_mps"):
        if c not in df.columns:
            raise ValueError(f"truth file is missing required column '{c}' (has {list(df.columns)})")
    if "true_angle_rad" not in df.columns:
        df["true_angle_rad"] = np.radians(df["true_angle_deg"]) if "true_angle_deg" in df.columns else np.nan
    if "target_present" not in df.columns:
        df["target_present"] = 1
    if "episode" not in df.columns:
        df["episode"] = 0
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df["episode"] = df["episode"].fillna(0).astype(int)
    df = df[~df["time_s"].isna()].sort_values(["episode", "time_s"], kind="stable")
    dup = df.duplicated(["episode", "time_s"], keep="last"); qc["n_duplicate_rows"] = int(dup.sum()); df = df[~dup]
    df["target_present"] = df["target_present"].fillna(1).astype(int)
    if (df.groupby("episode")["time_s"].diff().dropna() > 5.0).any():      # gaps WITHIN an episode only
        qc["warnings"].append("truth has gaps > 5 s inside an episode; interpolating truth across them is unreliable")
    qc["n_valid"] = int(len(df))
    return df.reset_index(drop=True), qc


# --------------------------------------------------------------------------------------
# replay
# --------------------------------------------------------------------------------------
def _to_meas(row, sigma_defaults) -> SensorMeasurement:
    if not row["detected"]:
        return SensorMeasurement(False, None, None, None, 0.0)
    conf = float(row["confidence"]) if "confidence" in row and not pd.isna(row["confidence"]) else 0.5
    rs = row.get("range_sigma_m", np.nan); a_s = row.get("angle_sigma_rad", np.nan); vs = row.get("velocity_sigma_mps", np.nan)
    if sigma_defaults is not None and row["sensor"] in sigma_defaults:
        d = sigma_defaults[row["sensor"]]
        rs = d[0] if pd.isna(rs) else rs; a_s = d[1] if pd.isna(a_s) else a_s; vs = d[2] if pd.isna(vs) else vs
    f = lambda x: None if pd.isna(x) else float(x)
    return SensorMeasurement(True, float(row["range_m"]), float(row["angle_rad"]), float(row["closing_speed_mps"]), conf,
                             range_sigma_m=f(rs), angle_sigma_rad=f(a_s), velocity_sigma_mps=f(vs))


def _ticks(meas: pd.DataFrame, tol: float) -> List[List[int]]:
    """Group row indices into ticks per episode: rows within `tol` seconds of the tick's first row."""
    out = []
    for ep, g in meas.groupby("episode", sort=True):
        t = g["time_s"].to_numpy(); idx = g.index.to_numpy(); start = 0
        for i in range(1, len(t) + 1):
            if i == len(t) or t[i] - t[start] > tol:
                out.append((ep, idx[start:i].tolist())); start = i
    return out


def run_replay(meas: pd.DataFrame, truth: Optional[pd.DataFrame] = None,
               visibility: Optional[VisibilitySeries] = None, cfg: Optional[ReplayConfig] = None) -> pd.DataFrame:
    """Drive tracker + fusion + risk (+ optional sensing-health monitor) on the log. Returns one row per tick."""
    cfg = cfg or ReplayConfig()
    sensors = set(meas["sensor"].unique())
    mode = cfg.mode if cfg.mode != "auto" else ("fusion" if len(sensors) > 1 else next(iter(sensors)))
    expected = {"lidar": ["lidar"], "radar": ["radar"]}.get(mode, sorted(sensors))
    hc = cfg.health
    cal = cfg.fog_calibration or FogCalibration()
    models = cfg.health_sensors or {"lidar": FalconEyeSensor(), "radar": RadarSensor()}
    rows, state = [], dict(track=None, monitor=None, prev_t=None, prev_ep=None, prev_threat=False)

    def emit(ep, t, meas_for_track, obs, dis, dt):
        track = state["track"]
        track.update(meas_for_track, dt)
        vis = float(visibility.at(t)) if visibility is not None else None
        valid = track.is_valid()
        risk = assess_risk(track.range_est, track.angle_est, track.closing_velocity(), cfg.collision, visibility_m=vis,
                           angle_sigma_rad=track.angle_sigma_rad) if valid else None
        row = dict(episode=ep, time_s=t, valid=valid, range_est=track.range_est if valid else np.nan,
                   closing_est=track.closing_velocity() if valid else np.nan,
                   angle_est=track.angle_est if valid else np.nan,
                   risk=(risk.risk_level.value if risk is not None else "none"),
                   visibility_m=vis if vis is not None else np.nan,
                   any_detection=bool(meas_for_track is not None and meas_for_track.target_detected))
        if state["monitor"] is not None:
            fog = cal.scenario_for(vis) if vis is not None else None
            rep = state["monitor"].update(t, obs, fog, dis, valid, state["prev_threat"])
            row["sensing_state"] = rep.state.value; row["sensing_reasons"] = ";".join(rep.reasons)
            state["prev_threat"] = risk is not None and risk.risk_level in (RiskLevel.ADVISORY, RiskLevel.WARNING)
        rows.append(row)

    for ep, idxs in _ticks(meas, cfg.cycle_tol_s):
        if ep != state["prev_ep"]:
            state.update(track=AlphaBetaTrack(**cfg.track_kwargs), prev_t=None, prev_ep=ep, prev_threat=False)
            state["monitor"] = (SensingHealthMonitor({n: models[n] for n in expected if n in models}, hc, cfg.collision)
                                if hc is not None else None)
        sub = meas.loc[idxs]
        t = float(sub["time_s"].iloc[0])
        by, obs = {}, {n: None for n in expected}
        for s_ in sensors:
            r_ = sub[sub["sensor"] == s_]
            if len(r_):
                row_ = r_.iloc[0]
                by[s_] = _to_meas(row_, cfg.default_sigma)
                if s_ in obs:
                    age = row_["latency_s"] if "latency_s" in row_ and not pd.isna(row_["latency_s"]) else None
                    obs[s_] = SensorObs(failed=bool(row_.get("invalid", 0)), age_s=None if age is None else float(age))
        l, r = by.get("lidar"), by.get("radar")
        if mode == "lidar": m = l
        elif mode == "radar": m = r
        else: m = fuse_measurements(l, r, reference_range=state["track"].predicted_range(), weighting=cfg.weighting) if (l or r) else None
        dis = None
        if l is not None and r is not None and l.target_detected and r.target_detected:
            dis = not measurements_consistent(l, r)
        # heartbeat ticks across a data gap, so that total data loss is observable
        if hc is not None and state["prev_t"] is not None and t - state["prev_t"] > 1.5 * hc.heartbeat_s:
            tk = state["prev_t"] + hc.heartbeat_s
            while tk < t - 1e-9:
                emit(ep, tk, None, {n: None for n in expected}, None, hc.heartbeat_s)
                tk += hc.heartbeat_s
            dt = max(t - (tk - hc.heartbeat_s), 1e-6)
        else:
            dt = cfg.first_dt_s if state["prev_t"] is None else max(t - state["prev_t"], 1e-6)
        emit(ep, t, m, obs, dis, dt)
        state["prev_t"] = t
    df = pd.DataFrame(rows)
    if truth is not None and len(df):
        df = _attach_truth(df, truth)
    return df


def _attach_truth(df: pd.DataFrame, truth: pd.DataFrame) -> pd.DataFrame:
    out = []
    for ep, g in df.groupby("episode", sort=False):
        tr = truth[truth["episode"] == ep]
        g = g.copy()
        if len(tr) < 2:
            for c in ("true_range_m", "true_closing_mps", "true_angle_rad"): g[c] = np.nan
            g["target_present"] = 0
        else:
            tt = tr["time_s"].to_numpy()
            inside = (g["time_s"] >= tt[0]) & (g["time_s"] <= tt[-1])
            for c in ("true_range_m", "true_closing_mps", "true_angle_rad"):
                col = tr[c].to_numpy(dtype=float); ok = ~np.isnan(col)
                g[c] = np.interp(g["time_s"], tt[ok], col[ok]) if ok.sum() >= 2 else np.nan
                g.loc[~inside, c] = np.nan
            idx = np.clip(np.searchsorted(tt, g["time_s"].to_numpy(), side="right") - 1, 0, len(tt) - 1)
            g["target_present"] = np.where(inside, tr["target_present"].to_numpy()[idx], 0)
        out.append(g)
    return pd.concat(out).reset_index(drop=True)


# --------------------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------------------
def _metrics(df: pd.DataFrame, cfg: ReplayConfig) -> dict:
    n = len(df)
    m = dict(n_ticks=int(n))
    if n == 0:
        return m
    has_truth = "true_range_m" in df.columns
    m["track_valid_frac"] = float(df["valid"].mean())
    m["warning_tick_frac"] = float((df["risk"] == "warning").mean())
    if not has_truth:
        if "sensing_state" in df.columns:
            for st in ("nominal", "degraded", "blind"):
                m[f"state_{st}_frac"] = float((df["sensing_state"] == st).mean())
        return m
    pres = df["target_present"] == 1
    valid = df["valid"]
    m["n_target_present_ticks"] = int(pres.sum())
    m["availability"] = float((valid & pres).sum() / pres.sum()) if pres.sum() else None
    both = valid & pres & ~df["true_range_m"].isna()
    e = (df["range_est"] - df["true_range_m"])[both].to_numpy()
    good = np.abs(e) < cfg.accuracy_gate_m
    m["range_rmse_m"] = float(np.sqrt(np.mean(e[good] ** 2))) if good.any() else None
    m["range_excluded_frac"] = float((~good).mean()) if len(e) else None
    ev = (df["closing_est"] - df["true_closing_mps"])[both].to_numpy()
    m["closing_rmse_mps"] = float(np.sqrt(np.mean(ev[good] ** 2))) if good.any() else None
    if valid.sum():
        err_all = (df["range_est"] - df["true_range_m"]).abs()
        ghost = valid & (~pres | (err_all >= cfg.accuracy_gate_m) | df["true_range_m"].isna())
        m["ghost_track_frac"] = float(ghost.sum() / valid.sum())
    # warnings versus truth
    miss = (df["true_range_m"] * np.sin(df["true_angle_rad"])).abs()
    threat = pres & ((miss <= cfg.protected_zone_m) if df["true_angle_rad"].notna().any() else True)
    w = df["risk"] == "warning"
    m["n_warning_ticks"] = int(w.sum())
    if w.sum():
        m["false_warning_frac"] = float((w & ~threat).sum() / w.sum())
        tw = w & threat & valid
        if tw.any():
            first = df[tw].iloc[0]
            true_ttc = first["true_range_m"] / first["true_closing_mps"] if first["true_closing_mps"] > 1e-3 else np.nan
            est_ttc = first["range_est"] / first["closing_est"] if first["closing_est"] > 1e-3 else np.nan
            m["reaction_budget_s"] = None if np.isnan(true_ttc) else float(true_ttc)
            m["ttc_error_at_warning_s"] = None if (np.isnan(true_ttc) or np.isnan(est_ttc)) else float(est_ttc - true_ttc)
    if "sensing_state" in df.columns:
        for st in ("nominal", "degraded", "blind"):
            m[f"state_{st}_frac"] = float((df["sensing_state"] == st).mean())
        ttc = df["true_range_m"] / df["true_closing_mps"].where(df["true_closing_mps"] > 1e-3)
        due = threat & (ttc <= cfg.collision.ttc_warning_threshold_s)
        unwarned = due & (df["risk"] != "warning")
        m["unwarned_threat_ticks"] = int(unwarned.sum())
        if unwarned.sum():
            m["unwarned_silent_frac"] = float((unwarned & (df["sensing_state"] == "nominal")).sum() / unwarned.sum())
    return m


def score(df: pd.DataFrame, cfg: Optional[ReplayConfig] = None) -> dict:
    cfg = cfg or ReplayConfig()
    out = {"overall": _metrics(df, cfg), "by_visibility": []}
    if "visibility_m" in df.columns and df["visibility_m"].notna().any():
        edges = cfg.vis_bins_m
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = df[(df["visibility_m"] >= lo) & (df["visibility_m"] < hi)]
            if len(sel):
                d = _metrics(sel, cfg); d["visibility_band_m"] = f"{lo:.0f}-{'inf' if np.isinf(hi) else format(hi, '.0f')}"
                out["by_visibility"].append(d)
    return out


def replay_files(measurements, truth=None, visibility_csv=None, metar=None, visibility_unit=None,
                 cfg: Optional[ReplayConfig] = None) -> dict:
    cfg = cfg or ReplayConfig()
    meas, qc_m = load_measurements(measurements)
    tr, qc_t = (load_truth(truth) if truth else (None, None))
    vis, qc_v = (None, None)
    if visibility_csv:
        vis = load_visibility_csv(visibility_csv, unit=visibility_unit); qc_v = vis.qc
    elif metar:
        vis = load_metar_file(metar); qc_v = vis.qc
    df = run_replay(meas, tr, vis, cfg)
    return {"qc": {"measurements": qc_m, "truth": qc_t, "visibility": qc_v}, "ticks": df, "scores": score(df, cfg)}


def main(argv=None):
    ap = argparse.ArgumentParser(description="Replay recorded LiDAR/radar logs through the Falcon Eye pipeline.")
    ap.add_argument("--measurements", required=True); ap.add_argument("--truth")
    ap.add_argument("--visibility", help="visibility CSV (unit in header or --visibility-unit)")
    ap.add_argument("--metar", help="text file of METARs"); ap.add_argument("--visibility-unit")
    ap.add_argument("--mode", default="auto", choices=["auto", "lidar", "radar", "fusion"])
    ap.add_argument("--adaptive-gain", action="store_true"); ap.add_argument("--visibility-margin", action="store_true")
    ap.add_argument("--health", action="store_true", help="enable the sensing-health monitor (nominal/degraded/blind annunciation)")
    ap.add_argument("--design-closing-speed", type=float, default=100.0, help="design encounter closing speed (m/s) for the capability check")
    ap.add_argument("--miss-distance", action="store_true", help="closest-point-of-approach threat test instead of the fixed 5 deg cone")
    ap.add_argument("--miss-sigma-k", type=float, default=0.0, help="lateral-uncertainty margin in sigmas (0 = none)")
    ap.add_argument("--out", default="replay_out")
    a = ap.parse_args(argv)
    cfg = ReplayConfig(mode=a.mode, track_kwargs={"adaptive_gain": True} if a.adaptive_gain else {},
                       health=HealthConfig(design_closing_speed_mps=a.design_closing_speed) if a.health else None,
                       collision=CollisionRiskConfig(visibility_margin_enabled=a.visibility_margin,
                                                     miss_distance_mode=a.miss_distance, miss_sigma_k=a.miss_sigma_k))
    res = replay_files(a.measurements, a.truth, a.visibility, a.metar, a.visibility_unit, cfg)
    os.makedirs(a.out, exist_ok=True)
    res["ticks"].to_csv(os.path.join(a.out, "ticks.csv"), index=False)
    json.dump({"qc": res["qc"], "scores": res["scores"]}, open(os.path.join(a.out, "scores.json"), "w"), indent=1, default=float)
    print(json.dumps({"qc": res["qc"], "scores": res["scores"]}, indent=1, default=float))


if __name__ == "__main__":
    main()
