"""
calibrate.py
Estimate how a real sensor behaves versus visibility from recorded logs + reference truth, and turn the
estimate into a FogCalibration (falcon_eye.fogdata) the simulator can use.

What is estimated (per sensor, per visibility band)
  * detection probability on genuine opportunities (target present, in range, in field of view)
  * false-detection rate (detections that do not match the truth, over ALL sensor cycles)
  * robust range / angle / closing-speed noise (1.4826 * MAD of matched residuals)
and, for the LiDAR, an empirical EXTINCTION SCALE k so that p = p0 * (1-(r/Rmax)^2) * exp(-2 k sigma(V,lambda) r)
best explains the observed detections (maximum likelihood, with a 95% profile interval).

Honest limits
  * Residual noise includes the truth source's own error (set truth_sigma_m to remove it in quadrature)
    and any clock offset between files (closing speed x offset).
  * noise / false-return multipliers are RELATIVE to the reference band (the clearest qualifying band);
    if no clear-air data exist, they are relative to whatever the clearest band was.
  * A band is used only if it has enough samples; otherwise it is skipped, never extrapolated.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pandas as pd

from .fog import angstrom_exponent
from .fogdata import FogCalibration, VisibilitySeries
from .replay import VIS_BINS_M, _attach_truth


def _wilson(k, n, z=1.96):
    if n == 0:
        return (np.nan, np.nan)
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def _robust_sigma(x):
    x = np.asarray(x, float); x = x[~np.isnan(x)]
    if len(x) < 5:
        return np.nan
    return float(1.4826 * np.median(np.abs(x - np.median(x))))


def prepare(meas: pd.DataFrame, truth: pd.DataFrame, vis: VisibilitySeries, sensor: str,
            max_range_m: float, fov_rad: float, gate_m: float) -> pd.DataFrame:
    d = meas[(meas["sensor"] == sensor) & (meas.get("invalid", 0) == 0)].copy()      # invalid rows are not genuine cycles
    d = _attach_truth(d, truth)
    d["visibility_m"] = vis.at(d["time_s"].to_numpy())
    ang_ok = d["true_angle_rad"].isna() | (d["true_angle_rad"].abs() <= fov_rad / 2.0)
    d["opportunity"] = ((d["target_present"] == 1) & (d["true_range_m"] > 0) & (d["true_range_m"] <= max_range_m) & ang_ok)
    d["matched"] = (d["detected"] == 1) & ((d["range_m"] - d["true_range_m"]).abs() <= gate_m)
    d["false_event"] = (d["detected"] == 1) & ~d["matched"]
    d["res_range"] = d["range_m"] - d["true_range_m"]
    d["res_angle"] = d["angle_rad"] - d["true_angle_rad"]
    d["res_vel"] = d["closing_speed_mps"] - d["true_closing_mps"]
    return d


def estimate_sensor_statistics(meas, truth, vis, sensor="lidar", max_range_m=9000.0, fov_rad=np.radians(60.0),
                               gate_m=300.0, bins=VIS_BINS_M, truth_sigma_m=0.0) -> pd.DataFrame:
    d = prepare(meas, truth, vis, sensor, max_range_m, fov_rad, gate_m)
    rows = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        s = d[(d["visibility_m"] >= lo) & (d["visibility_m"] < hi)]
        if not len(s):
            continue
        opp = s[s["opportunity"]]; mt = opp[opp["matched"]]
        lo_ci, hi_ci = _wilson(len(mt), len(opp))
        sr = _robust_sigma(mt["res_range"])
        if truth_sigma_m and not np.isnan(sr):
            sr = float(np.sqrt(max(sr ** 2 - truth_sigma_m ** 2, 0.0)))
        n_miss = len(s) - len(mt)                        # cycles without a genuine detection
        fr_miss = float(s["false_event"].sum() / n_miss) if n_miss > 0 else np.nan
        # Expected number of FALSE returns that fall inside the +-gate window by chance (false returns are
        # ~uniform over the range): if that is comparable to the genuine matches, residual statistics of the
        # 'matched' set are dominated by false returns and must not be trusted.
        exp_false_matches = (0.0 if np.isnan(fr_miss) else fr_miss) * max(len(opp) - len(mt), 0) * min(1.0, 2.0 * gate_m / max_range_m)
        contamination = (exp_false_matches / len(mt)) if len(mt) else np.inf
        rows.append(dict(sensor=sensor, band_m=f"{lo:.0f}-{'inf' if np.isinf(hi) else format(hi, '.0f')}",
                         median_visibility_m=float(s["visibility_m"].median()), n_cycles=int(len(s)),
                         n_opportunities=int(len(opp)), n_matched=int(len(mt)),
                         pd=(len(mt) / len(opp)) if len(opp) else np.nan, pd_ci_lo=lo_ci, pd_ci_hi=hi_ci,
                         false_events=int(s["false_event"].sum()), false_rate=float(s["false_event"].mean()),
                         false_rate_given_miss=fr_miss, contamination=float(contamination),
                         sigma_range_m=sr, sigma_angle_rad=_robust_sigma(mt["res_angle"]),
                         sigma_velocity_mps=_robust_sigma(mt["res_vel"])))
    return pd.DataFrame(rows)


def fit_fog_calibration(stats: pd.DataFrame, min_cycles=200, min_matched=30, max_contamination=0.2, wavelength_nm=550.0,
                        extinction_model="koschmieder", extinction_scale=1.0) -> FogCalibration:
    ok = ((stats["n_cycles"] >= min_cycles) & (stats["n_matched"] >= min_matched) & stats["sigma_range_m"].notna()
          & (stats["sigma_range_m"] > 0) & (stats["contamination"] <= max_contamination))
    skipped = stats.loc[~ok, "band_m"].tolist()
    q = stats[ok].sort_values("median_visibility_m")
    if len(q) < 2:
        raise ValueError(f"need >= 2 visibility bands with >= {min_cycles} cycles and >= {min_matched} matched detections; "
                         f"have {len(q)}. Collect more data in more fog conditions (skipped bands are never extrapolated).")
    ref = q.iloc[-1]                                   # clearest qualifying band
    n_miss = lambda r: r["n_cycles"] - r["n_matched"]
    f_ref = (ref["false_events"] + 0.5) / (n_miss(ref) + 1.0)
    vis = tuple(float(v) for v in q["median_visibility_m"])
    nm = tuple(float(s / ref["sigma_range_m"]) for s in q["sigma_range_m"])
    fr = tuple(float(((r["false_events"] + 0.5) / (n_miss(r) + 1.0)) / f_ref) for _, r in q.iterrows())
    return FogCalibration(vis, nm, fr, wavelength_nm, extinction_model, extinction_scale,
                          source=f"fitted from {int(q['n_cycles'].sum())} cycles; reference band {ref['band_m']} "
                                 f"(median vis {ref['median_visibility_m']:.0f} m) defines multiplier 1.0; skipped (too few genuine detections or "
                                 f"too contaminated by false returns): {skipped or 'none'}")


def fit_extinction_scale(meas, truth, vis, sensor="lidar", max_range_m=9000.0, fov_rad=np.radians(60.0), gate_m=300.0,
                         wavelength_nm=550.0, extinction_model="koschmieder",
                         k_grid=None, p0_grid=None) -> dict:
    """ML fit of (k, p0) in p = p0 (1-(r/Rmax)^2) exp(-2 k sigma r). Returns k, p0 and a 95% interval for k."""
    d = prepare(meas, truth, vis, sensor, max_range_m, fov_rad, gate_m)
    o = d[d["opportunity"]]
    if len(o) < 200:
        raise ValueError(f"only {len(o)} detection opportunities; need >= 200")
    r = o["true_range_m"].to_numpy(float); v = o["visibility_m"].to_numpy(float); y = o["matched"].to_numpy(float)
    sig = 3.912 / np.maximum(v, 1.0)
    if extinction_model != "koschmieder" and abs(wavelength_nm - 550.0) > 1e-9:
        uv = np.unique(np.round(v)); qmap = {u: angstrom_exponent(u / 1000.0, extinction_model) for u in uv}
        sig = sig * (wavelength_nm / 550.0) ** (-np.array([qmap[x] for x in np.round(v)]))
    fall = np.maximum(1.0 - (r / max_range_m) ** 2, 0.0)
    ks = np.geomspace(0.05, 8.0, 200) if k_grid is None else np.asarray(k_grid)
    ps = np.linspace(0.5, 1.0, 26) if p0_grid is None else np.asarray(p0_grid)
    ll = np.empty((len(ks), len(ps)))
    for i, k in enumerate(ks):
        T = np.exp(-2.0 * k * sig * r) * fall
        for j, p0 in enumerate(ps):
            p = np.clip(p0 * T, 1e-6, 1 - 1e-6)
            ll[i, j] = np.sum(y * np.log(p) + (1 - y) * np.log1p(-p))
    prof = ll.max(axis=1); ib = int(np.argmax(prof)); jb = int(np.argmax(ll[ib]))
    inside = ks[(2 * (prof.max() - prof)) <= 3.84]
    return dict(k_best=float(ks[ib]), p0_best=float(ps[jb]), k_ci95=(float(inside.min()), float(inside.max())),
                n_opportunities=int(len(o)), loglik=float(prof.max()),
                note="k=1 means the Koschmieder extinction at the chosen wavelength model explains the data; "
                     "k<1: sensor sees further than the model, k>1: less far.")
