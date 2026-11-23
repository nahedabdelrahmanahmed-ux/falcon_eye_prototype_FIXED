"""Tests for the optional uncertainty-driven (adaptive) tracker gains. Offline, deterministic."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from falcon_eye.sensor import SensorMeasurement
from falcon_eye.tracking import AlphaBetaTrack

fails = 0
def check(name, cond):
    global fails
    print(("PASS: " if cond else "FAIL: ") + name); fails += (not cond)

def M(r, s_r=10.0, v=100.0, s_v=2.0, ang=0.0):
    return SensorMeasurement(True, r, ang, v, 0.9, range_sigma_m=s_r, angle_sigma_rad=0.01, velocity_sigma_mps=s_v)

def run(track, sigma, n=40, dt=0.1, seed=0):
    rng = np.random.default_rng(seed); r0 = 5000.0
    for k in range(n):
        track.update(M(r0 - 100.0 * dt * k + rng.normal(0, sigma), sigma, 100.0 + rng.normal(0, 2.0)), dt)
    return track

# 1) variances shrink as measurements accumulate (gain falls as the track becomes certain)
t = run(AlphaBetaTrack(adaptive_gain=True), 10.0)
check("range variance falls below a single measurement's variance after 40 updates", t.range_var < 100.0)
check("variances stay positive and finite", t.range_var > 0 and t.rate_var > 0 and np.isfinite(t.range_var + t.rate_var))

# 2) an accurate measurement is trusted more than a noisy one (same prior)
def first_gain(sigma):
    tr = AlphaBetaTrack(adaptive_gain=True)
    tr.update(M(5000.0, 10.0), 0.1); tr.update(M(4990.0, 10.0), 0.1)        # confirm
    prior = tr.range_est + tr.range_rate_est * 0.1
    tr.update(M(prior + 30.0, sigma), 0.1)                                   # +30 m residual
    return (tr.range_est - prior) / 30.0
g_acc, g_noisy = first_gain(3.0), first_gain(30.0)
check(f"accurate sensor (sigma 3) gets a larger gain than noisy (sigma 30): {g_acc:.2f} > {g_noisy:.2f}", g_acc > g_noisy)

# 3) fallback: no sigma on the measurement -> identical to the fixed-gain track
a = AlphaBetaTrack(adaptive_gain=True); b = AlphaBetaTrack(adaptive_gain=False)
for k in range(30):
    m = SensorMeasurement(True, 5000.0 - 10.0 * k + (k % 3), 0.0, 100.0, 0.9)    # no sigma fields
    a.update(m, 0.1); b.update(m, 0.1)
check("no sigma -> adaptive track equals fixed-gain track exactly", a.range_est == b.range_est and a.range_rate_est == b.range_rate_est)

# 4) default is OFF, and with it OFF the sigma fields change nothing
check("adaptive_gain defaults to False (shipped behaviour unchanged)", AlphaBetaTrack().adaptive_gain is False)
c = run(AlphaBetaTrack(adaptive_gain=False), 10.0); d = run(AlphaBetaTrack(), 10.0)
check("adaptive off == default track", c.range_est == d.range_est)

# 5) smoother range than fixed gain on a noisy stream (the benefit measured in the benchmark)
errs = {}
for name, kw in (("fixed", {}), ("adaptive", {"adaptive_gain": True})):
    tr = AlphaBetaTrack(**kw); rng = np.random.default_rng(5); e = []
    for k in range(200):
        true = 5000.0 - 100.0 * 0.1 * k
        tr.update(M(true + rng.normal(0, 15.0), 15.0, 100.0 + rng.normal(0, 2.0), 2.0), 0.1)
        if k > 50 and tr.is_valid(): e.append(tr.range_est - true)
    errs[name] = float(np.sqrt(np.mean(np.square(e))))
check(f"adaptive range RMSE below fixed on a sigma=15 m stream ({errs['adaptive']:.1f} < {errs['fixed']:.1f} m)", errs["adaptive"] < errs["fixed"])

# 6) a sudden manoeuvre: the track must re-converge (no divergence from over-confidence)
tr = AlphaBetaTrack(adaptive_gain=True); rng = np.random.default_rng(9); r = 5000.0; v = 100.0
for k in range(300):
    if k > 100: v = 160.0                                                    # abrupt speed-up
    r -= v * 0.1
    tr.update(M(r + rng.normal(0, 5.0), 5.0, v + rng.normal(0, 1.0), 1.0), 0.1)
check("after an abrupt closing-speed change the tracked speed re-converges (<5 m/s error)", abs(tr.closing_velocity() - 160.0) < 5.0)

print("\n" + ("All adaptive-gain tests passed." if not fails else f"{fails} test(s) FAILED."))
sys.exit(1 if fails else 0)
