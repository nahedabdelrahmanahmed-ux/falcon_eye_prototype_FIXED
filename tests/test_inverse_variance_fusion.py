"""Tests for inverse-variance fusion (offline, deterministic seeds)."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from falcon_eye.sensor import FalconEyeSensor, SensorMeasurement
from falcon_eye.radar import RadarSensor
from falcon_eye.fog import get_fog_scenario
from falcon_eye.tracking import fuse_measurements

def M(r, s_r, a=0.0, s_a=0.01, v=100.0, s_v=1.0, conf=0.9, false=False):
    return SensorMeasurement(True, r, a, v, conf, is_false_detection=false,
                             range_sigma_m=s_r, angle_sigma_rad=s_a, velocity_sigma_mps=s_v)

fails = 0
def check(name, cond):
    global fails
    print(("PASS: " if cond else "FAIL: ") + name); fails += (not cond)

# (A) sigma_fused <= min(sigma) for many random sigma pairs and for every fog level
rng = np.random.default_rng(0)
ok = True
for _ in range(2000):
    sl, sr = rng.uniform(0.5, 50, 2)
    f = fuse_measurements(M(1000, sl), M(1000, sr))
    ok &= f.range_sigma_m <= min(sl, sr) + 1e-12
check("sigma_fused <= min(sigma) (2000 random pairs)", ok)
for fogk in ("clear", "light", "moderate", "dense"):
    lm = FalconEyeSensor(rng=np.random.default_rng(1)); rm = RadarSensor(rng=np.random.default_rng(2))
    fog = get_fog_scenario(fogk)
    pair = None
    for _ in range(20000):          # short range: LiDAR is nearly blind further out in thick fog
        a, b = lm.measure(50, 0.0, 100.0, fog), rm.measure(50, 0.0, 100.0, fog)
        if a.target_detected and b.target_detected and not a.is_false_detection and not b.is_false_detection:
            pair = (a, b); break
    assert pair is not None, f"no joint detection found for {fogk}"
    a, b = pair; f = fuse_measurements(a, b)
    assert f.range_sigma_m <= min(a.range_sigma_m, b.range_sigma_m) + 1e-12
    print(f"   {fogk:9s} sigma_L={a.range_sigma_m:5.2f}  sigma_R={b.range_sigma_m:5.2f}  sigma_fused={f.range_sigma_m:5.2f}")
check("sigma_fused <= min(sensor sigma) with real sensor models, all fog levels", True)

# (B) equal sigma -> 50/50
f = fuse_measurements(M(1000, 10), M(1040, 10))
check("equal sigma -> exactly the midpoint (weights 50%)", abs(f.estimated_range - 1020) < 1e-9)
check("equal sigma -> sigma_fused = sigma/sqrt(2)", abs(f.range_sigma_m - 10/np.sqrt(2)) < 1e-9)

# Document's worked example: sigma 3 vs 15 -> weights 96% / 4%
f = fuse_measurements(M(1000, 3), M(1020, 15))
w_l = (1/9)/((1/9)+(1/225))
check(f"worked example: lidar weight {w_l:.3f} (~96%), fused ~1001 m", abs(w_l-0.9615) < 1e-3 and abs(f.estimated_range - 1000.77) < 0.05)

# (D) fallback: no sigma -> identical to confidence weighting
a = SensorMeasurement(True, 1000, 0.0, 100.0, 0.9); b = SensorMeasurement(True, 1020, 0.0, 100.0, 0.7)
f = fuse_measurements(a, b)
check("no sigma -> falls back to confidence weighting (1008.75)", abs(f.estimated_range - (1000*.9+1020*.7)/1.6) < 1e-9 and f.range_sigma_m is None)
f2 = fuse_measurements(M(1000, 3, conf=0.9), M(1020, 15, conf=0.7), weighting="confidence")
check("weighting='confidence' reproduces the earlier scheme", abs(f2.estimated_range - (1000*.9+1020*.7)/1.6) < 1e-9)

# (E) the consistency gate still rejects contradicting pairs (ghost protection unchanged)
f = fuse_measurements(M(1000, 3, conf=0.3), M(4000, 15, conf=0.9), reference_range=1005)
check("contradicting pair -> picks the one closest to the track, no averaging", abs(f.estimated_range - 1000) < 1e-9)

# (C) RMSE through the REAL fuse_measurements
print("\n=== RMSE of range estimate (m), both sensors detect (true detections only) ===")
N = 6000
print(f"{'fog':9s}{'R':>6s}{'both%':>8s}{'LiDAR':>9s}{'Radar':>9s}{'conf-wt':>9s}{'inv-var':>9s}{'gain':>8s}")
rows = {}
for fogk in ("clear", "light", "moderate", "dense"):
    fog = get_fog_scenario(fogk)
    for R in (50, 3000):
        lm = FalconEyeSensor(rng=np.random.default_rng(11)); rm = RadarSensor(rng=np.random.default_rng(22))
        L, Rr, C, I = [], [], [], []
        for _ in range(N):
            a, b = lm.measure(R, 0.0, 100.0, fog), rm.measure(R, 0.0, 100.0, fog)
            if a.target_detected and b.target_detected and not a.is_false_detection and not b.is_false_detection:
                L.append(a.estimated_range); Rr.append(b.estimated_range)
                C.append(fuse_measurements(a, b, weighting="confidence").estimated_range)
                I.append(fuse_measurements(a, b, weighting="inverse_variance").estimated_range)
        n = len(L)
        if n < 30:
            print(f"{fogk:9s}{R:6d}{100*n/N:7.1f}%   (too few joint detections; excluded)"); continue
        rm_ = lambda x: float(np.sqrt(np.mean((np.array(x)-R)**2)))
        rows[(fogk, R)] = (n, rm_(L), rm_(Rr), rm_(C), rm_(I))
        print(f"{fogk:9s}{R:6d}{100*n/N:7.1f}%{rm_(L):9.2f}{rm_(Rr):9.2f}{rm_(C):9.2f}{rm_(I):9.2f}{100*(rm_(I)/rm_(C)-1):7.0f}%")
ok = all(v[4] <= v[3] + 1e-9 for v in rows.values())
check("inverse-variance RMSE <= confidence-weighted RMSE in every measurable cell", ok)
ok = all(v[4] <= min(v[1], v[2]) * 1.05 for v in rows.values() if v[0] >= 200)
check("fused RMSE not worse than the better single sensor (5% tolerance, n>=200)", ok)
print("\n" + ("All inverse-variance fusion tests passed." if not fails else f"{fails} test(s) FAILED."))
sys.exit(1 if fails else 0)
