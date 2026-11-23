"""Tests for the sensing-health monitor ("sensing degraded"). Offline, deterministic."""
import os, sys, tempfile
import numpy as np
import pandas as pd
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from falcon_eye.fog import FOG_SCENARIOS
from falcon_eye.health import (HealthConfig, SensingHealthMonitor, SensingState, SensorObs, acquisition_probability)
from falcon_eye.radar import RadarSensor
from falcon_eye.sensor import FalconEyeSensor
from falcon_eye.simulation import SimulationConfig, run_scenario, make_head_on_scenario
from falcon_eye.failures import FailureConfig
from falcon_eye.collision import CollisionRiskConfig
from falcon_eye.replay import replay_files, ReplayConfig, load_measurements, load_truth
from falcon_eye.calibrate import prepare
from falcon_eye.fogdata import load_visibility_csv
from falcon_eye.synthetic_data import generate_dataset

fails = 0
def check(name, cond):
    global fails
    print(("PASS: " if cond else "FAIL: ") + name); fails += (not cond)
SENS = lambda: {"lidar": FalconEyeSensor(), "radar": RadarSensor()}
OK = SensorObs(); BAD = SensorObs(failed=True)
def mon(sensors=None, **kw): return SensingHealthMonitor(sensors or SENS(), HealthConfig(**kw))
def run_ticks(m, n, dt=0.1, t0=0.0, fog=None, lidar=OK, radar=OK, **kw):
    r = None
    for k in range(n):
        t = t0 + k * dt
        r = m.update(t, {"lidar": lidar if k % 1 == 0 else None, "radar": radar if k % 2 == 0 else None}, fog, **kw)
    return r, t0 + n * dt

# ---- math ----
check("P(>=2 hits in 2 cycles, p=0.5) = 0.25", abs(acquisition_probability(0.5, 2, 2) - 0.25) < 1e-12)
check("P(>=1 hit in 5 cycles, p=0.2) = 1-0.8^5", abs(acquisition_probability(0.2, 5, 1) - (1 - 0.8 ** 5)) < 1e-12)
check("acquisition probability is monotone in p", all(acquisition_probability(a, 20) <= acquisition_probability(b, 20) for a, b in zip(np.linspace(0, 1, 21)[:-1], np.linspace(0, 1, 21)[1:])))

# ---- capability model (uses the sensors' own detection model) ----
m = mon(); F = FOG_SCENARIOS
rl = [m.effective_range_m("lidar", F[k]) for k in ("clear", "light", "moderate", "dense")]
rr = [m.effective_range_m("radar", F[k]) for k in ("clear", "light", "moderate", "dense")]
check(f"LiDAR effective range falls steeply with fog ({rl[0]:.0f} > {rl[1]:.0f} > {rl[2]:.0f} > {rl[3]:.0f} m)", rl[0] > rl[1] > rl[2] > rl[3])
check("LiDAR in dense fog is effectively blind (< 50 m)", rl[3] < 50)
check("radar effective range is nearly fog-independent (> 5 km everywhere)", min(rr) > 5000 and max(rr) - min(rr) < 200)
check("detection probability is shared by measure() and the monitor (single source of truth)",
      abs(FalconEyeSensor().detection_probability(1000.0, 0.0, F["light"]) - FalconEyeSensor()._p_detect(1000.0, 0.0, F["light"])[0]) < 1e-15)
rng = np.random.default_rng(0); s = FalconEyeSensor(rng=rng); n = 20000
hit = sum(s.measure(1500.0, 0.0, 50.0, F["clear"], extra_failure_probability=0.0).target_detected for _ in range(n))
p_model = s.detection_probability(1500.0, 0.0, F["clear"])
check(f"empirical detection rate matches the model (p={p_model:.3f}, measured {hit / n:.3f})", abs(hit / n - p_model) < 0.02 + s.base_false_detection_probability)

# ---- state logic ----
r, _ = run_ticks(mon(), 20, fog=F["clear"])
FAULTY = ("SENSOR_FAULT", "SENSOR_STALE", "SENSOR_DATA_AGE", "SENSOR_DISAGREEMENT", "TRACK_LOST_THREAT")
check("fusion, clear air, healthy -> NOMINAL with no fault reason", r.state == SensingState.NOMINAL and not any(x.startswith(FAULTY) for x in r.reasons))
check("...the LiDAR is 7% short of the 2.5 km design range: reported as an informational SENSOR_REDUCED, not a fault",
      any(x.startswith("SENSOR_REDUCED:lidar") for x in r.reasons))
r, _ = run_ticks(mon(), 20, fog=F["light"])
check("fusion, light fog: still NOMINAL (radar carries the warning) but REDUNDANCY_LOST is reported", r.state == SensingState.NOMINAL and "REDUNDANCY_LOST" in r.reasons)
check("...and the LiDAR is called out as the reduced/incapable sensor", any(x.startswith(("SENSOR_REDUCED:lidar", "SENSOR_INCAPABLE:lidar")) for x in r.reasons))
r, _ = run_ticks(mon(required_ok_sensors=2), 20, fog=F["light"]); check("required_ok_sensors=2 demands redundancy -> DEGRADED", r.state == SensingState.DEGRADED)
r, _ = run_ticks(SensingHealthMonitor({"lidar": FalconEyeSensor()}, HealthConfig()), 20, fog=F["light"], radar=None)
check("LiDAR alone in light fog -> BLIND (incapable at the design encounter)", r.state == SensingState.BLIND)
r, _ = run_ticks(SensingHealthMonitor({"lidar": FalconEyeSensor()}, HealthConfig()), 20, fog=F["clear"], radar=None)
check("LiDAR alone in clear air -> DEGRADED (reduced: 2.3 km < 2.5 km required), with the numbers in the reason",
      r.state == SensingState.DEGRADED and any("SENSOR_REDUCED:lidar" in x and "required 2500" in x for x in r.reasons))
r, _ = run_ticks(SensingHealthMonitor({"lidar": FalconEyeSensor()}, HealthConfig(design_closing_speed_mps=80.0)), 20, fog=F["clear"], radar=None)
check("a slower design encounter (80 m/s -> 2.0 km) makes clear-air LiDAR NOMINAL: capability is relative to the design encounter", r.state == SensingState.NOMINAL)
r, _ = run_ticks(mon(), 20, fog=None); check("visibility unknown -> capability not assessed (no INCAPABLE claim invented)", r.capability_known is False and not any("INCAPABLE" in x or "REDUCED" in x for x in r.reasons))

# ---- data faults ----
m = mon(); r, t = run_ticks(m, 20, fog=F["clear"]); check("single failed cycle does not raise a fault", m.update(t, {"lidar": BAD, "radar": OK}, F["clear"]).state == SensingState.NOMINAL)
m = mon(); r, t = run_ticks(m, 10, fog=F["clear"])
for k in range(12): r = m.update(t + 0.1 * k, {"lidar": None, "radar": OK if k % 2 == 0 else None}, F["clear"])
check("LiDAR silent for > 0.5 s -> SENSOR_STALE:lidar", any(x.startswith("SENSOR_STALE:lidar") for x in r.reasons))
m = mon(); r, t = run_ticks(m, 10, fog=F["clear"], lidar=BAD)
check("mostly-failed cycles -> SENSOR_FAULT:lidar", any(x.startswith("SENSOR_FAULT:lidar") for x in r.reasons))
r, t = run_ticks(m, 20, t0=t, fog=F["clear"]); check("fault clears after the sensor recovers", not any("lidar" in x and "FAULT" in x for x in r.reasons))
m = mon(); r, t = run_ticks(m, 12, fog=F["clear"], lidar=SensorObs(age_s=1.0))
check("data older than max_data_age_s -> SENSOR_DATA_AGE:lidar", any(x.startswith("SENSOR_DATA_AGE:lidar") for x in r.reasons))
r, t = run_ticks(mon(), 12, fog=F["clear"], lidar=SensorObs(age_s=0.1)); check("data of normal age is fine", not any("AGE" in x for x in r.reasons))
r, _ = run_ticks(SensingHealthMonitor({"lidar": FalconEyeSensor(), "radar": RadarSensor()}, HealthConfig()), 12, fog=F["clear"], lidar=BAD, radar=BAD)
check("all sensors faulted -> BLIND", r.state == SensingState.BLIND)

# ---- consistency + lost threat ----
m = mon(); r = None
for k in range(8): r = m.update(0.1 * k, {"lidar": OK, "radar": OK}, F["clear"], disagree=True)
check("persistent LiDAR/radar disagreement -> SENSOR_DISAGREEMENT and at least DEGRADED", "SENSOR_DISAGREEMENT" in r.reasons and r.state != SensingState.NOMINAL)
for k in range(12): r = m.update(1.0 + 0.1 * k, {"lidar": OK, "radar": OK}, F["clear"], disagree=False)
check("disagreement clears once the sensors agree again", "SENSOR_DISAGREEMENT" not in r.reasons)
m = mon(track_lost_hold_s=2.0); m.update(0.0, {"lidar": OK, "radar": OK}, F["clear"], track_valid=True, threat_active=True)
r = m.update(0.1, {"lidar": OK, "radar": OK}, F["clear"], track_valid=False, threat_active=False)
check("a threatening track that disappears -> TRACK_LOST_THREAT, state DEGRADED", "TRACK_LOST_THREAT" in r.reasons and r.state == SensingState.DEGRADED)
r = m.update(2.5, {"lidar": OK, "radar": OK}, F["clear"], track_valid=False)
check("...held for track_lost_hold_s and then released", "TRACK_LOST_THREAT" not in r.reasons)
m = mon(); m.update(0.0, {"lidar": OK, "radar": OK}, F["clear"], track_valid=True, threat_active=False)
r = m.update(0.1, {"lidar": OK, "radar": OK}, F["clear"], track_valid=False)
check("a non-threatening track that disappears raises nothing", "TRACK_LOST_THREAT" not in r.reasons)

# ---- simulation integration ----
def sim(mode, fog, health, fm="none", seed=3, **kw):
    h, t = make_head_on_scenario(initial_range_m=9000, closing_speed_mps=100)
    return run_scenario(h, t, SimulationConfig(duration_s=60, dt_s=0.1, fog_key=fog, mode=mode, seed=seed, health=health,
                                               failure=FailureConfig(mode=fm, start_time_s=20.0, end_time_s=40.0), **kw))
a = sim("fusion", "moderate", None); b = sim("fusion", "moderate", HealthConfig())
check("monitor OFF by default: no health arrays", a.sensing_state is None)
check("monitor ON never changes risk levels or the track (annunciation only)", a.risk_level == b.risk_level and np.array_equal(a.tracked_range, b.tracked_range, equal_nan=True) and a.warning_time == b.warning_time)
r = sim("lidar", "dense", HealthConfig()); check("simulation: LiDAR-only in dense fog -> BLIND throughout", set(r.sensing_state) == {"blind"})
r = sim("fusion", "clear", HealthConfig()); check("simulation: fusion in clear air is NOMINAL >= 90% of the time", np.mean(np.array(r.sensing_state) == "nominal") >= 0.9)
r = sim("fusion", "clear", HealthConfig(), fm="no_detection"); t = r.time; w = (t >= 20) & (t <= 40)
fault = np.array([any(x.startswith(("SENSOR_STALE", "SENSOR_FAULT")) for x in rs) for rs in r.sensing_reasons])
check("simulation: injected sensor outage is flagged within 1 s of its start", fault[w].mean() > 0.9 and t[np.argmax(fault & w)] - 20.0 <= 1.0)
check("...and the flag clears after the outage", not fault[t > 45].any())
r = sim("fusion", "clear", HealthConfig(), fm="delayed_measurement")
check("simulation: delayed measurements are flagged through the data-age signal", any(any(x.startswith("SENSOR_DATA_AGE") for x in rs) for rs in r.sensing_reasons))
r = sim("fusion", "clear", HealthConfig(), fm="biased_range")
check("DOCUMENTED LIMIT: a range bias common to both sensors is NOT detected as a fault", not any(any(x.startswith(("SENSOR_FAULT", "SENSOR_STALE", "SENSOR_DATA_AGE", "SENSOR_DISAGREEMENT")) for x in rs) for rs in r.sensing_reasons))

# ---- replay integration ----
tmp = tempfile.mkdtemp()
P = generate_dataset(os.path.join(tmp, "ds"), n_episodes=6, seed=7, vis_range_m=(15000, 20000))
base = replay_files(P["measurements"], P["truth"], P["visibility"], cfg=ReplayConfig(health=HealthConfig()))
tk = base["ticks"]
check("replay: health columns present and clear-air fusion is mostly NOMINAL", "sensing_state" in tk.columns and (tk.sensing_state == "nominal").mean() > 0.85)
check("replay: scores include state shares", "state_nominal_frac" in base["scores"]["overall"])
off = replay_files(P["measurements"], P["truth"], P["visibility"])
check("replay: health OFF leaves the tick table without health columns (unchanged behaviour)", "sensing_state" not in off["ticks"].columns)
m = pd.read_csv(P["measurements"], comment="#")
gap = m[(m.episode == 0) & (m.time_s >= 40) & (m.time_s < 50)].index
p2 = os.path.join(tmp, "outage.csv"); open(p2, "w").write("# SYNTHETIC\n"); m.drop(gap).to_csv(open(p2, "a", newline=""), index=False)
o = replay_files(p2, P["truth"], P["visibility"], cfg=ReplayConfig(health=HealthConfig()))["ticks"]
g = o[(o.episode == 0) & (o.time_s > 41) & (o.time_s < 49)]; after = o[(o.episode == 0) & (o.time_s > 52) & (o.time_s < 58)]
check("replay: a 10 s total data outage yields heartbeat ticks (no silent gap)", len(g) >= 30)
check("replay: ...reported as BLIND (all sensors stale) during the outage", (g.sensing_state == "blind").mean() > 0.9)
check("replay: ...and NOMINAL again after data resume", (after.sensing_state == "nominal").mean() > 0.8)
mm, qc = load_measurements(P["measurements"]); tr, _ = load_truth(P["truth"]); vis = load_visibility_csv(P["visibility"])
d = pd.read_csv(P["measurements"], comment="#"); dd = os.path.join(tmp, "inv.csv"); d.loc[d.index[d.detected == 1][:50], "range_m"] = -5.0
open(dd, "w").write("# SYNTHETIC\n"); d.to_csv(open(dd, "a", newline=""), index=False)
mi, qci = load_measurements(dd)
check("invalid rows are kept as failed cycles (flag), not silently deleted", qci["n_dropped_invalid_detections"] == 50 and int(mi["invalid"].sum()) == 50 and len(mi) == len(mm))
n_bad_lidar = int((d.loc[d.index[d.detected == 1][:50], "sensor"].str.strip().str.lower() == "lidar").sum())
check(f"calibration excludes exactly the {n_bad_lidar} corrupted LiDAR rows (invalid rows are not genuine cycles)",
      len(prepare(mi, tr, vis, "lidar", 9000, 1.0, 300)) == len(prepare(mm, tr, vis, "lidar", 9000, 1.0, 300)) - n_bad_lidar and n_bad_lidar > 0)
fog_ds = generate_dataset(os.path.join(tmp, "fog"), n_episodes=10, seed=9, vis_range_m=(80, 400))
lid = replay_files(fog_ds["measurements"], fog_ds["truth"], fog_ds["visibility"], cfg=ReplayConfig(mode="lidar", health=HealthConfig()))
sc = lid["scores"]["overall"]
check("replay: LiDAR-only in dense fog is BLIND and every unwarned threat is annunciated (no silent misses)",
      sc["state_blind_frac"] > 0.9 and sc.get("unwarned_silent_frac", 0.0) == 0.0)

print("\n" + ("All sensing-health tests passed." if not fails else f"{fails} test(s) FAILED."))
sys.exit(1 if fails else 0)
