"""Tests for the miss-distance (closest-point-of-approach) threat test. Offline, deterministic."""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from falcon_eye.collision import CollisionRiskConfig, RiskLevel, assess_risk
from falcon_eye.sensor import SensorMeasurement
from falcon_eye.tracking import AlphaBetaTrack

fails = 0
def check(name, cond):
    global fails
    print(("PASS: " if cond else "FAIL: ") + name); fails += (not cond)

cone, cpa0, cpa2 = CollisionRiskConfig(), CollisionRiskConfig(miss_distance_mode=True, miss_sigma_k=0.0), CollisionRiskConfig(miss_distance_mode=True, miss_sigma_k=2.0)
check("miss_distance_mode is OFF by default (shipped behaviour unchanged)", cone.miss_distance_mode is False)
R, vc = 3000.0, 150.0                                    # TTC = 20 s -> inside the 25 s warning threshold
def A(miss, cfg, sig=None, sign=1):
    th = sign * np.arcsin(miss / R)
    return assess_risk(R, th, vc, cfg, angle_sigma_rad=sig).risk_level
check("target passing at 300 m (inside the 500 m zone) is MISSED by the 5 deg cone (bearing 5.7 deg)", A(300, cone) != RiskLevel.WARNING)
check("...and is flagged by the miss-distance test", A(300, cpa0) == RiskLevel.WARNING)
check("target passing at 800 m is not flagged (k=0)", A(800, cpa0) != RiskLevel.WARNING)
check("bearing sign does not matter", A(300, cpa0, sign=-1) == RiskLevel.WARNING and A(800, cpa0, sign=-1) != RiskLevel.WARNING)
check("bearing uncertainty widens the zone: sigma 0.1 rad -> 2*3000*0.1 = +600 m -> 800 m is flagged", A(800, cpa2, sig=0.1) == RiskLevel.WARNING)
check("no sigma available -> no inflation (k ignored)", A(800, cpa2, sig=None) != RiskLevel.WARNING)
check("small sigma (0.3 deg) barely changes the 800 m decision (+31 m)", A(800, cpa2, sig=np.radians(0.3)) != RiskLevel.WARNING)
check("zone is range-aware: same bearing, 600 m miss at R=3000 is outside, a 450 m miss at R=3000 is inside",
      A(600, cpa0) != RiskLevel.WARNING and A(450, cpa0) == RiskLevel.WARNING)
check("TTC logic still applies: target inside the zone but TTC 40 s -> advisory, not warning",
      assess_risk(6000.0, np.arcsin(300 / 6000.0), 150.0, cpa0).risk_level == RiskLevel.ADVISORY)
check("receding target never alarms", assess_risk(3000.0, 0.0, -50.0, cpa0).risk_level == RiskLevel.NONE)
check("legacy signature still works (no angle_sigma argument)", assess_risk(R, 0.0, vc, cone).risk_level == RiskLevel.WARNING)
t = AlphaBetaTrack()
m = SensorMeasurement(True, 3000.0, 0.01, 100.0, 0.9, range_sigma_m=3.0, angle_sigma_rad=0.005, velocity_sigma_mps=1.0)
t.update(m, 0.1)
check("tracker carries the bearing sigma of its latest measurement", t.angle_sigma_rad == 0.005)
print("\n" + ("All miss-distance tests passed." if not fails else f"{fails} test(s) FAILED."))
sys.exit(1 if fails else 0)
