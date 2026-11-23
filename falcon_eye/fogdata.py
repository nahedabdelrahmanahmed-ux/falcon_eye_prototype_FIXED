"""
fogdata.py
Ingestion of REAL visibility / fog observations and conversion to the simulator's fog model.

Why this module exists
----------------------
The built-in fog scenarios are four fixed, qualitative anchors. Real flights meet fog that
changes with time, is reported in different units and conventions, and comes with dropouts and
sentinel values. This module lets real observations drive the simulator and the replay tools:

  * parse_metar_visibility / find_rvr   : METAR (ICAO metres and US statute-mile forms) and RVR groups
  * load_visibility_csv                 : visibility-sensor logs (MOR/visibility in m, km, SM or ft)
                                          with strict unit handling and quality control
  * load_metar_file                     : text files of METARs (optionally time-stamped)
  * FogCalibration                      : visibility -> (noise multiplier, false-return multiplier,
                                          extinction scale); default = the original qualitative
                                          anchors, replaceable by values fitted from real data
  * FogProfile                          : time-varying fog usable by falcon_eye.simulation

Aviation conventions handled deliberately
-----------------------------------------
  * METAR "9999" and "CAVOK" mean >= 10 km (a LOWER BOUND, not a measurement) -> flagged.
  * "P6SM" means > 6 SM, "M1/4SM" means < 1/4 SM -> flagged with a qualifier.
  * "0000" means < 50 m.
  * RVR is a runway-light based quantity and is NOT the same as meteorological visibility (MOR);
    it is parsed separately and never silently substituted for visibility.
  * Units are never guessed: a CSV whose unit cannot be read from the header must be given
    `unit=` explicitly (a silent km-vs-m mistake would shift fog severity by 1000x).

None of this validates the physics against reality. It makes real data usable.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

from .fog import FogScenario, FOG_SCENARIOS

SM_M = 1609.344
FT_M = 0.3048
_UNIT_TO_M = {"m": 1.0, "km": 1000.0, "sm": SM_M, "mi": SM_M, "ft": FT_M}
VIS_CAP_M = 10000.0          # METAR upper reporting limit (>= 10 km)


# --------------------------------------------------------------------------------------
# METAR / RVR parsing
# --------------------------------------------------------------------------------------
@dataclass
class VisibilityObs:
    meters: float
    qualifier: str = "exact"   # exact | less_than | greater_than | cavok
    source: str = ""


_WIND = re.compile(r"^(\d{3}|VRB)\d{2,3}(G\d{2,3})?(KT|MPS|KMH)$")
_US_SM = re.compile(r"^([PM]?)(\d+/\d+|\d+)SM$")
_ICAO_VIS = re.compile(r"^(\d{4})(NDV|N|NE|E|SE|S|SW|W|NW)?$")


def _frac(s: str) -> float:
    if "/" in s:
        a, b = s.split("/")
        return float(a) / float(b)
    return float(s)


def parse_metar_visibility(text: str) -> Optional[VisibilityObs]:
    """Extract prevailing visibility from a METAR line (or a bare visibility group).
    Returns None when no visibility group is present (e.g. '////')."""
    tokens = text.upper().split()
    if not tokens:
        return None
    if "CAVOK" in tokens:
        return VisibilityObs(VIS_CAP_M, "cavok", "CAVOK")
    # US statute-mile form (possibly a whole number token followed by a fraction token)
    for i, tok in enumerate(tokens):
        m = _US_SM.match(tok)
        if not m:
            continue
        q, val = m.group(1), m.group(2)
        v = _frac(val)
        if "/" in val and i > 0 and re.fullmatch(r"\d+", tokens[i - 1]):
            v += float(tokens[i - 1])
        qual = {"P": "greater_than", "M": "less_than", "": "exact"}[q]
        return VisibilityObs(v * SM_M, qual, tok)
    # ICAO metres form: first 4-digit group after the wind group (and optional variable wind)
    for i, tok in enumerate(tokens):
        if _WIND.match(tok):
            j = i + 1
            while j < len(tokens) and re.fullmatch(r"\d{3}V\d{3}", tokens[j]):
                j += 1
            if j < len(tokens):
                m = _ICAO_VIS.match(tokens[j])
                if m:
                    return _icao(int(m.group(1)), tokens[j])
            return None
    if len(tokens) == 1:
        m = _ICAO_VIS.match(tokens[0])
        if m:
            return _icao(int(m.group(1)), tokens[0])
    return None


def _icao(v: int, src: str) -> VisibilityObs:
    if v >= 9999:
        return VisibilityObs(VIS_CAP_M, "greater_than", src)
    if v == 0:
        return VisibilityObs(25.0, "less_than", src)     # "0000" = below 50 m
    return VisibilityObs(float(v), "exact", src)


_RVR = re.compile(r"^R(\d{2}[LCR]?)/([PM]?)(\d{4})(?:V([PM]?)(\d{4}))?(FT)?([UDN])?$")


def find_rvr(text: str) -> List[dict]:
    """Parse Runway Visual Range groups (e.g. R28L/1200V1800FT, R10/P2000N).
    RVR is NOT meteorological visibility; it is returned separately on purpose."""
    out = []
    for tok in text.upper().split():
        m = _RVR.match(tok)
        if not m:
            continue
        k = FT_M if m.group(6) else 1.0
        d = dict(runway=m.group(1), rvr_m=int(m.group(3)) * k,
                 qualifier={"P": "greater_than", "M": "less_than", "": "exact"}[m.group(2)],
                 trend=m.group(7) or "")
        if m.group(5):
            d["rvr_variable_to_m"] = int(m.group(5)) * k
        out.append(d)
    return out


# --------------------------------------------------------------------------------------
# Visibility series (CSV / METAR files) with quality control
# --------------------------------------------------------------------------------------
@dataclass
class VisibilitySeries:
    t_s: np.ndarray
    vis_m: np.ndarray
    qc: Dict[str, object] = field(default_factory=dict)
    lower_bound: Optional[np.ndarray] = None    # True where the value is only a lower bound (>=10 km)

    def __len__(self):
        return len(self.t_s)

    def at(self, t) -> np.ndarray:
        """Log-linear interpolation (visibility spans orders of magnitude), held at the ends."""
        lv = np.interp(np.asarray(t, dtype=float), self.t_s, np.log10(self.vis_m))
        return 10.0 ** lv


_TIME_COLS = ("time_s", "t_s", "time", "timestamp", "datetime", "date_time", "utc", "epoch", "t")
_VIS_COLS = ("visibility", "vis", "mor", "visibility_m", "vis_m", "mor_m", "visibility_km", "vis_km",
             "mor_km", "visibility_sm", "vis_sm", "visibility_ft", "vis_ft", "prevailing_visibility")
_SENTINELS = {-9999.0, -999.0, -99.0, 99999.0, 999999.0}


def _unit_from_name(name: str) -> Optional[str]:
    n = name.lower().strip()
    for suf, u in (("_km", "km"), ("(km)", "km"), (" km", "km"), ("_sm", "sm"), ("(sm)", "sm"), ("_mi", "mi"),
                   ("_ft", "ft"), ("(ft)", "ft"), ("_m", "m"), ("(m)", "m"), (" m", "m")):
        if n.endswith(suf):
            return u
    return None


def load_visibility_csv(path, time_col: Optional[str] = None, vis_col: Optional[str] = None,
                        unit: Optional[str] = None, max_gap_s: float = 3600.0,
                        min_vis_m: float = 10.0, max_vis_m: float = 100000.0) -> VisibilitySeries:
    """Load a visibility / MOR time series from CSV with strict QC.

    Units are read from the column name (visibility_m, vis_km, ...) or from `unit=`; they are NEVER
    guessed. Raises ValueError on an unreadable unit, missing columns or fewer than 2 valid rows."""
    df = pd.read_csv(path, comment="#", skipinitialspace=True)
    cols = {c.lower().strip(): c for c in df.columns}
    tc = time_col or next((cols[c] for c in _TIME_COLS if c in cols), None)
    vc = vis_col or next((cols[c] for c in _VIS_COLS if c in cols), None)
    if tc is None or vc is None:
        raise ValueError(f"cannot find time/visibility columns in {list(df.columns)}; pass time_col= / vis_col=")
    u = (unit or _unit_from_name(vc))
    if u is None:
        raise ValueError(f"unit of column '{vc}' is not stated; pass unit='m'|'km'|'sm'|'ft' "
                         f"(units are never guessed: a km/m mix-up shifts fog severity 1000x)")
    u = u.lower()
    if u not in _UNIT_TO_M:
        raise ValueError(f"unsupported unit '{u}'")

    qc: Dict[str, object] = {"n_rows": int(len(df)), "unit": u, "warnings": []}
    raw_t = df[tc]
    if pd.api.types.is_numeric_dtype(raw_t):
        t = raw_t.astype(float).to_numpy()
        if np.nanmax(np.abs(t)) > 1e9:      # epoch seconds
            qc["warnings"].append("time looked like epoch seconds; rebased to first sample")
    else:
        # Resolution-independent (pandas >= 2 may store datetimes in s/ms/us, not only ns).
        ts = pd.to_datetime(raw_t, errors="coerce", utc=True)
        t = (ts - pd.Timestamp("1970-01-01", tz="UTC")).dt.total_seconds().to_numpy(dtype=float)
    v = pd.to_numeric(df[vc], errors="coerce").to_numpy(dtype=float)

    bad_sentinel = np.isin(v, list(_SENTINELS))
    nan = np.isnan(v) | np.isnan(t) | bad_sentinel
    neg = (~nan) & (v <= 0)
    qc["n_dropped_missing_or_sentinel"] = int(nan.sum())
    qc["n_dropped_nonpositive"] = int(neg.sum())
    keep = ~(nan | neg)
    t, v = t[keep], v[keep] * _UNIT_TO_M[u]

    order = np.argsort(t, kind="stable")
    qc["n_out_of_order"] = int((np.diff(t) < 0).sum())
    t, v = t[order], v[order]
    if len(t) and np.any(np.diff(t) == 0):
        _, idx = np.unique(t[::-1], return_index=True)    # keep the LAST of duplicated timestamps
        idx = len(t) - 1 - idx
        qc["n_duplicate_times"] = int(len(t) - len(idx))
        idx = np.sort(idx); t, v = t[idx], v[idx]
    else:
        qc["n_duplicate_times"] = 0

    # Only the METAR reporting cap itself (9999 / 10000 m) is a lower bound; a visibility SENSOR legitimately
    # reports above 10 km, so values beyond the cap are measurements.
    lower = np.isin(np.round(v), [9999.0, 10000.0]) if u == "m" else np.zeros(len(v), bool)
    qc["n_clipped_low"] = int((v < min_vis_m).sum()); qc["n_clipped_high"] = int((v > max_vis_m).sum())
    v = np.clip(v, min_vis_m, max_vis_m)
    if len(t) < 2:
        raise ValueError(f"fewer than 2 valid visibility samples after QC: {qc}")
    t = t - t[0]
    gaps = np.diff(t)
    qc["max_gap_s"] = float(gaps.max()); qc["median_dt_s"] = float(np.median(gaps)); qc["n_valid"] = int(len(t))
    if gaps.max() > max_gap_s:
        qc["warnings"].append(f"gap of {gaps.max():.0f}s exceeds max_gap_s={max_gap_s:.0f}s: interpolation across it is unreliable")
    if lower.any():
        qc["warnings"].append(f"{int(lower.sum())} values equal the METAR cap (9999/10000 m) and are lower bounds, not measurements")
    return VisibilitySeries(t, v, qc, lower)


_TS_PREFIX = re.compile(r"^\s*(\d{4}[/-]?\d{2}[/-]?\d{2}[ T]?\d{2}:?\d{2})")
_DHM = re.compile(r"\b(\d{2})(\d{2})(\d{2})Z\b")


def load_metar_file(path) -> VisibilitySeries:
    """Read a text file of METARs, one per line. A leading 'YYYY/MM/DD HH:MM' (or ISO) stamp is used when
    present; otherwise the DDHHMMZ group (day rollover to the next month is assumed when the day decreases).
    Observations without a parsable visibility group are dropped and counted."""
    ts, vs, lows, n, nvis = [], [], [], 0, 0
    month_off, last_day = 0, None
    for line in open(path, encoding="utf8", errors="replace"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        n += 1
        obs = parse_metar_visibility(line)
        if obs is None:
            continue
        m = _TS_PREFIX.match(line)
        if m:
            t = pd.to_datetime(m.group(1).replace("/", "-"), utc=True, errors="coerce")
            if pd.isna(t):
                continue
            tsec = t.timestamp()
        else:
            d = _DHM.search(line)
            if not d:
                continue
            day, hh, mm = int(d.group(1)), int(d.group(2)), int(d.group(3))
            if last_day is not None and day < last_day:
                month_off += 1
            last_day = day
            tsec = ((month_off * 30 + day) * 24 + hh) * 3600 + mm * 60
        nvis += 1
        ts.append(tsec); vs.append(obs.meters); lows.append(obs.qualifier in ("greater_than", "cavok"))
    qc = {"n_lines": n, "n_with_visibility": nvis, "n_dropped": n - nvis, "unit": "m", "warnings": []}
    if nvis < 2:
        raise ValueError(f"fewer than 2 METARs with visibility in {path}: {qc}")
    order = np.argsort(ts, kind="stable")
    t = np.asarray(ts)[order]; v = np.asarray(vs)[order]; lo = np.asarray(lows)[order]
    qc["n_valid"] = int(len(t)); qc["max_gap_s"] = float(np.diff(t - t[0]).max())
    if lo.any():
        qc["warnings"].append(f"{int(lo.sum())} observations are lower bounds (>=10 km / CAVOK / P6SM)")
    if qc["max_gap_s"] > 3600:
        qc["warnings"].append("METAR spacing >1 h: visibility between observations is interpolated, not measured")
    return VisibilitySeries(t - t[0], np.clip(v, 10.0, 100000.0), qc, lo)


# --------------------------------------------------------------------------------------
# Calibration and time-varying profile
# --------------------------------------------------------------------------------------
@dataclass
class FogCalibration:
    """Maps visibility to the sensor-model degradation factors.

    Defaults are the ORIGINAL qualitative anchors of the prototype (100/500/2000/20000 m). They are
    assumptions, not measurements: replace them with values fitted from real sensor logs
    (falcon_eye.calibrate). Interpolation is linear in log10(visibility), constant beyond the ends."""
    visibility_m: Tuple[float, ...] = (100.0, 500.0, 2000.0, 20000.0)
    noise_multiplier: Tuple[float, ...] = (5.0, 2.5, 1.5, 1.0)
    false_return_multiplier: Tuple[float, ...] = (8.0, 3.5, 1.8, 1.0)
    wavelength_nm: float = 550.0
    extinction_model: str = "koschmieder"
    extinction_scale: float = 1.0
    source: str = "default-qualitative-anchors (assumption, not measured)"

    def scenario_for(self, visibility_m: float, name: Optional[str] = None) -> FogScenario:
        order = np.argsort(self.visibility_m)
        lx = np.log10(np.asarray(self.visibility_m, float)[order])
        x = np.log10(max(float(visibility_m), 1.0))
        nm = float(np.interp(x, lx, np.asarray(self.noise_multiplier, float)[order]))
        fr = float(np.interp(x, lx, np.asarray(self.false_return_multiplier, float)[order]))
        return FogScenario(name or f"vis {visibility_m:.0f} m", float(visibility_m), nm, fr,
                           self.wavelength_nm, self.extinction_model, self.extinction_scale)

    def to_json(self, path):
        json.dump(self.__dict__, open(path, "w"), indent=1)

    @staticmethod
    def from_json(path) -> "FogCalibration":
        d = json.load(open(path))
        for k in ("visibility_m", "noise_multiplier", "false_return_multiplier"):
            d[k] = tuple(d[k])
        return FogCalibration(**d)


@dataclass
class FogProfile:
    """Time-varying fog for falcon_eye.simulation (SimulationConfig.fog_profile)."""
    t_s: np.ndarray
    vis_m: np.ndarray
    calibration: FogCalibration = field(default_factory=FogCalibration)

    @staticmethod
    def from_series(series: VisibilitySeries, calibration: Optional[FogCalibration] = None) -> "FogProfile":
        return FogProfile(series.t_s, series.vis_m, calibration or FogCalibration())

    @staticmethod
    def from_schedule(points: Sequence[Tuple[float, float]], calibration: Optional[FogCalibration] = None) -> "FogProfile":
        p = sorted(points)
        return FogProfile(np.array([a for a, _ in p], float), np.array([b for _, b in p], float),
                          calibration or FogCalibration())

    def visibility_at(self, t: float) -> float:
        return float(10.0 ** np.interp(t, self.t_s, np.log10(self.vis_m)))

    def at(self, t: float) -> FogScenario:
        return self.calibration.scenario_for(self.visibility_at(t))
