"""Tests for the visibility-adaptive TTC margin (HEXA-TPU-RT report, Section 3). Offline, deterministic."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from falcon_eye.collision import CollisionRiskConfig, RiskLevel, assess_risk, ttc_margin_factor
from falcon_eye.fog import FOG_SCENARIOS

fails = 0
def check(name, cond):
    global fails
    print(("PASS: " if cond else "FAIL: ") + name); fails += (not cond)

on, off = CollisionRiskConfig(visibility_margin_enabled=True), CollisionRiskConfig()
check("feature is OFF by default", off.visibility_margin_enabled is False)
check("factor == 1.0 when disabled, whatever the visibility", ttc_margin_factor(off, 100.0) == 1.0)
check("factor == 1.0 when visibility unknown", ttc_margin_factor(on, None) == 1.0)
check("factor == 1.0 at/above the clear-visibility threshold", ttc_margin_factor(on, 10000.0) == 1.0 and ttc_margin_factor(on, 50000.0) == 1.0)
vis = [20000, 2000, 500, 100, 50]
fs = [ttc_margin_factor(on, v) for v in vis]
check("monotonic: lower visibility -> larger (or equal) factor", all(a <= b for a, b in zip(fs, fs[1:])))
check("capped at 1 + max_margin_extra", abs(max(fs) - 1.6) < 1e-9)
print("   factors:", {k: round(ttc_margin_factor(on, f.visibility_m), 2) for k, f in FOG_SCENARIOS.items()})

# behaviour: a target at TTC = 32 s (range 3200 m, closing 100 m/s) on the corridor axis
args = dict(estimated_range=3200.0, relative_angle=0.0, relative_velocity=100.0)
check("TTC 32 s is NOT a warning with fixed margin", assess_risk(**args, config=off).risk_level != RiskLevel.WARNING)
check("TTC 32 s IS a warning in dense fog with the adaptive margin (threshold 40 s)",
      assess_risk(**args, config=on, visibility_m=100.0).risk_level == RiskLevel.WARNING)
check("TTC 32 s is still NOT a warning in clear air with the adaptive margin",
      assess_risk(**args, config=on, visibility_m=20000.0).risk_level != RiskLevel.WARNING)
check("visibility_m ignored when feature off",
      assess_risk(**args, config=off, visibility_m=100.0).risk_level == assess_risk(**args, config=off).risk_level)
check("signature backward compatible (no visibility argument)", assess_risk(**args, config=off).ttc == 32.0)
print("\n" + ("All visibility-margin tests passed." if not fails else f"{fails} test(s) FAILED."))
sys.exit(1 if fails else 0)
