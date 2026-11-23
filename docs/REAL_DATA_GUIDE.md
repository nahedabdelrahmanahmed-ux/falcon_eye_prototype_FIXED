# Testing Falcon Eye with real fog data: what to request and how to run it

This guide is written so it can be forwarded to a data provider (airport, airline, UAV operator, LiDAR/radar
vendor, meteorological service). **No real data ships with this project.** `falcon_eye.synthetic_data` produces
files in the same formats, stamped `# SYNTHETIC`, only to demonstrate the format and exercise the code.

## 1. What we need, and why

| # | Data | Used for | Required? |
|---|------|----------|-----------|
| 1 | **Sensor measurement log** (LiDAR and/or radar), one row per sensor cycle, *including cycles with no detection* | replay through tracker/fusion/risk | yes |
| 2 | **Reference ("truth") track** of the target relative to the host (ADS-B, RTK-GPS, or tracking range) | accuracy, detection probability, false-return rate | yes for scoring/calibration |
| 3 | **Visibility time series** from the same place/time (visibility sensor MOR, or METAR) | to bin results by fog density; calibration | yes |
| 4 | Sensor metadata: wavelength, field of view, max range, mounting, firmware, scan rate | model parameters | strongly recommended |
| 5 | Optional: RVR, extinction/backscatter logs, camera frames | cross-checks | no |

Without #2 the replay still runs, but only reports availability/warning behaviour, **not accuracy or false alarms**:
the correct answer is unknown.

## 2. File formats (CSV, UTF-8, header row, `#` comment lines allowed)

### 2.1 Measurement log: `measurements.csv`
| column | unit | notes |
|---|---|---|
| `time_s` | s | same clock as the other files (see 3.2) |
| `sensor` | `lidar` or `radar` | case-insensitive |
| `detected` | 0/1 | **log the 0 rows too**: detection probability cannot be computed without them |
| `range_m` | m | required when `detected=1` |
| `angle_rad` *or* `angle_deg` | rad / deg | bearing off the host nose, + = same sign as truth |
| `closing_speed_mps` *or* `range_rate_mps` | m/s | closing positive; range-rate negative = closing |
| `confidence`, `range_sigma_m`, `angle_sigma_rad`, `velocity_sigma_mps` | optional | 1-sigma accuracy per measurement if the sensor reports it |
| `episode` | int, optional | separates independent passes/flights; the tracker resets between episodes |
| `latency_s` | s, optional | age of the data when it reached the logger; used by the health monitor to flag stale data |

### 2.2 Truth: `truth.csv`
`time_s, true_range_m, true_closing_mps` plus optional `true_angle_rad|true_angle_deg`, `target_present` (0/1), `episode`.
Provide `target_present=0` rows for stretches where no target is in the scene; these measure false returns.

### 2.3 Visibility: `visibility_*.csv` **or** a METAR text file
* CSV: a time column (`time_s`, or ISO-8601 `timestamp`) and a visibility column **whose header carries the unit**
  (`visibility_m`, `vis_km`, `visibility_sm`, `vis_ft`) or `unit=` must be passed. **Units are never guessed**: a km/m
  mix-up shifts fog severity 1000-fold.
* METAR file: one report per line, optionally prefixed `YYYY/MM/DD HH:MM`. ICAO metres (`0800`, `9999`, `CAVOK`)
  and US statute miles (`1 1/2SM`, `P6SM`, `M1/4SM`) are parsed. `9999`, `CAVOK` and `P6SM` are **lower bounds**
  (>= 10 km / > 6 SM) and are flagged, not treated as measurements. RVR groups are parsed separately and are
  never used as visibility.

## 3. Things that silently ruin results (please check before sending)

1. **Visibility must be representative of the sensor's line of sight.** An airport METAR describes the airfield, not
   a fog bank 3 km away. Prefer a visibility sensor co-located with the host, or state the distance. The simulator
   can model a bank (`LayeredFogScenario`), but real data decide whether that matters.
2. **Clock synchronisation.** A 0.1 s offset at 100 m/s closing speed is a 10 m range error, larger than the
   sensor's own noise in clear air. Please state the sync method and its accuracy.
3. **Truth quality.** State the truth source's accuracy. It is subtracted in quadrature (`truth_sigma_m`) but cannot
   be corrected if unknown.
4. **Include clear-air passes.** Noise and false-return multipliers are *relative to the clearest qualifying band*;
   with no clear data they are relative to whatever was clearest.
5. **Include the sensor's own post-processing state** (filtering, thresholds, "high-sensitivity" modes): they change
   what "detected" means.
6. Remove anything that identifies aircraft or people that you are not permitted to share.

## 4. How much data

Each visibility band is used only if it has at least 200 sensor cycles and 30 genuine detections and the estimated
contamination by chance false returns is below 20 %. Bands that fail are **skipped and listed, never extrapolated**.
Consequence: **dense-fog LiDAR noise cannot be calibrated from the LiDAR alone**, because it barely detects. Close-range
passes (target within a few hundred metres) are what populate those bands.
In our synthetic test, 60 passes x 100 s recovered the extinction scale to +-2.5 %. Few passes give wider intervals, so
always read the confidence interval, not just the point value.

## 5. Running it

```bash
python -m falcon_eye.replay --measurements measurements.csv --truth truth.csv \
       --visibility visibility_m.csv            # or: --metar reports.txt  /  --visibility-unit km
       [--mode lidar|radar|fusion] [--miss-distance --miss-sigma-k 0] [--adaptive-gain] --out results/
```
Outputs: `results/ticks.csv` (per-tick track and risk) and `results/scores.json` containing **the data-quality report**
(rows dropped, duplicates, out-of-order, sentinel values, unit, gaps) followed by overall and per-visibility-band
scores: availability, range / closing-speed RMSE, ghost-track fraction, false-warning fraction, reaction budget.
Read the QC section first: it states what was removed.

### 5.1 Sensing-health ("sensing degraded") annunciation
Add `--health` to get, for every tick, a state `nominal | degraded | blind` and the reasons. It answers a different
question from the risk level: **can the absence of a warning be trusted?**
Evidence used: failed or invalid cycles (rows that fail QC are kept as failed cycles, never silently dropped), silent
sensors (heartbeat ticks are inserted across data gaps so a total outage is visible), data older than 0.5 s
(`latency_s`), persistent LiDAR/radar disagreement, a threatening track that vanishes, and the sensor's **effective range
in the current visibility** compared with the range a warning needs for a *design encounter*
(`--design-closing-speed`, default 100 m/s x 25 s = 2500 m; this is an assumption, so state it with the results).
For real data pass sensor models carrying the REAL sensor's maximum range and field of view
(`ReplayConfig(health_sensors=...)`); the default capability model is the project's simulated sensors.
With truth supplied the scores add `unwarned_threat_ticks` and `unwarned_silent_frac`: the share of threats that went
unwarned **while the state was still nominal**. That number, on real data, is the real test of the monitor.
It cannot detect: a range bias common to both sensors, bursts of false returns, or interference (see FIX_REPORT).

Calibration from the same files:
```python
from falcon_eye.replay import load_measurements, load_truth
from falcon_eye.fogdata import load_visibility_csv
from falcon_eye.calibrate import estimate_sensor_statistics, fit_fog_calibration, fit_extinction_scale
meas,_ = load_measurements("measurements.csv"); tr,_ = load_truth("truth.csv"); vis = load_visibility_csv("visibility_m.csv")
stats = estimate_sensor_statistics(meas, tr, vis, "lidar", max_range_m=..., fov_rad=...)   # use the SENSOR's real limits
cal   = fit_fog_calibration(stats);  cal.to_json("fog_calibration.json")
k     = fit_extinction_scale(meas, tr, vis, "lidar", wavelength_nm=905, extinction_model="kim")
```
Pass `FogCalibration.from_json(...)` to `FogProfile` to drive the simulator with the fitted model, and compare the simulated
results with the real replay to see how far the model still is from reality.

## 6. What the results can and cannot show

* They show how this tracker / fusion / warning logic behaves on **that sensor, that fog, that geometry**.
* Fitted multipliers describe that sensor. They do not transfer to another sensor model or mounting.
* The model assumes the host-axis (head-on) geometry for the miss-distance test; crossing traffic needs lateral
  velocity that these sensors do not provide.
* Nothing here is a certified detect-and-avoid algorithm; do not use it for operational decisions.
