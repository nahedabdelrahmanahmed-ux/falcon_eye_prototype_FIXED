import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from falcon_eye.sensor import FalconEyeSensor
from falcon_eye.radar import RadarSensor
from falcon_eye.fog import get_fog_scenario

lidar = FalconEyeSensor(rng=np.random.default_rng(11))
radar = RadarSensor(rng=np.random.default_rng(22))
N = 4000
FOGS = ("clear", "light", "moderate", "dense")
RANGES = (50, 100, 300, 1000, 3000)
GAMMAS = (1, 10, 100)

print("=== A. Proposed weights W_LiDAR = exp(-gamma*alpha), W_Radar = 1 - W_LiDAR ===")
print("alpha = 3.912/V from our fog levels. gamma is not defined anywhere in the proposal, so several are tried.")
print(f"{'fog':<9}{'alpha':>9}" + "".join(f"{'W_L g='+str(g):>13}" for g in GAMMAS))
for f in FOGS:
    a = get_fog_scenario(f).extinction_coefficient()
    print(f"{f:<9}{a:>9.5f}" + "".join(f"{np.exp(-g*a):>13.3f}" for g in GAMMAS))

print("\n=== B. Actual two-way LiDAR transmittance exp(-2*alpha*R) in our model ===")
print(f"{'fog':<9}" + "".join(f"{'R='+str(r)+'m':>12}" for r in RANGES))
for f in FOGS:
    a = get_fog_scenario(f).extinction_coefficient()
    print(f"{f:<9}" + "".join(f"{np.exp(-2*a*r):>12.4f}" for r in RANGES))

print(f"\n=== C. Range-estimate RMSE (m) when BOTH sensors detect; N = {N} trials per cell ===")
methods = ["LiDAR only", "Radar only", "Ours(conf)"] + [f"Theirs g={g}" for g in GAMMAS] + ["InvVar(opt)"]
print(f"{'fog':<9}{'R(m)':>6}{'both%':>7}" + "".join(f"{m:>13}" for m in methods))
for f in FOGS:
    fog = get_fog_scenario(f)
    a = fog.extinction_coefficient()
    sig_l, sig_r = max(2.5, 3.0 * fog.noise_multiplier), 15.0
    for R in RANGES:
        rl, rr, cl, cr = [], [], [], []
        for _ in range(N):
            lm = lidar.measure(R, 0.0, 100.0, fog)
            rm = radar.measure(R, 0.0, 100.0, fog)
            if lm.target_detected and rm.target_detected and not lm.is_false_detection and not rm.is_false_detection:
                rl.append(lm.estimated_range); rr.append(rm.estimated_range)
                cl.append(lm.confidence); cr.append(rm.confidence)
        n = len(rl)
        if n < 30:
            print(f"{f:<9}{R:>6}{100*n/N:>6.1f}%   (too few joint detections to estimate)")
            continue
        rl, rr, cl, cr = map(np.array, (rl, rr, cl, cr))
        est = [rl, rr, (rl*cl + rr*cr)/(cl+cr)]
        for g in GAMMAS:
            w = np.exp(-g*a); est.append(w*rl + (1-w)*rr)
        wl, wr = 1/sig_l**2, 1/sig_r**2
        est.append((wl*rl + wr*rr)/(wl+wr))
        print(f"{f:<9}{R:>6}{100*n/N:>6.1f}%" + "".join(f"{np.sqrt(np.mean((e-R)**2)):>13.2f}" for e in est))
