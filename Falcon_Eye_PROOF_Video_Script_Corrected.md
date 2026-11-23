# Falcon Eye: PROOF Video Script (corrected)

Same four-act structure as before, rewritten so every claim matches the technical report and the Monte Carlo data.
**Spoken words: about 130, which fits the 60 to 90 second window.**

---

## TITLE CARD (0:00 to 0:03, on screen)
**PROOF Aviation Futures Challenge**
"Don't Pitch the Future. Prove It."

## ACT 1: THE PROBLEM (0:03 to 0:15)
*Visual: fog rolling over a runway.*

> "When fog rolls in, sensors that rely on light can go blind. Weather is a factor in about a third of fatal general-aviation accidents."

## ACT 2: THE SOLUTION (0:15 to 0:35)
*Visual: the Sense, Detect, Track, Triage, Warn slide.*

> "Falcon Eye is an independent forward-awareness layer that fuses LiDAR and radar. Unlike single-sensor systems, it keeps detecting when one sensor fails."

## ACT 3: HOW IT WORKS, AND THE PROOF (0:35 to 1:05)
*Visual: the detection-rate chart, then the dashboard's Charts tab. Burn this label onto the screen the whole time:*
**[SIMULATION: 2,400 Monte Carlo trials]**

> "In our simulation of two thousand four hundred randomized trials, LiDAR alone drops to zero percent in dense fog. Fusion held a hundred percent detection in every condition we tested. A reflex path ranks multiple threats and is unit-tested never to delay a real alert. Hardware calibration is the next step."

## ACT 4: WHAT'S NEXT (1:05 to 1:25)
*Visual: split screen, a pilot in fog on one side and the triage figure on the other. Then a campus or test-range image with this label:*
**[CONCEPT: proposed first pilot site]**

> "Next: bench calibration with real LiDAR and radar, and an FPGA prototype of our ten-core accelerator, now in RTL. We're looking for a test-range partner and sensor access. Don't pitch the future. Prove it."

## END CARD (last 5 seconds, on screen)
**Falcon Eye: [Team name] · PROOF Aviation Futures Challenge · Oct 25, 2026 · #PROOFBoston #HTCJ #AviationFutures**

---

## What changed from the draft, and why

| Draft said | Now says | Reason |
|---|---|---|
| "Traditional aviation safety systems relying solely on LiDAR and cameras" | "Sensors that rely on light" | TCAS, ADS-B and terrain warning systems do not rely on LiDAR or cameras. |
| "77 GHz FMCW radar" | "radar" | Our radar model is generic with a 12 km range, so 77 GHz was never part of it. Automotive 77 GHz radar also cannot see aircraft at that range. |
| "sub-8ms reaction times", "radar instantly locks" | removed | Not measured anywhere. Our modeled sensor latency is 50 to 100 ms. |
| "engineered a custom RTL TPU in Verilog" | "ten-core accelerator, now in RTL" | The RTL is in progress, and the language is not confirmed. |
| "cycle-accurate simulation proves it cuts 47 down to zero" | removed the word "proves" in the video; the deck says an optimized scheduler cut it from up to 47 of 480 to zero | It is a scheduler comparison in simulation. |
| "moving from Phase 0 simulation to FPGA bench testing" | bench calibration plus an FPGA prototype | Phase 0 in our plan is sensor bench calibration, not simulation. |
| "from atmospheric physics to silicon" | removed | There is no silicon. |
| "We don't pitch the future. We prove it." | "Don't pitch the future. Prove it." | The challenge guide says not to reword the tagline. |
| "solo deep-tech founder" | removed | Keep this only if it is true and matches your team page. |
