# Falcon Eye — Miramar Pitch Night Script
**Target: 4:30–4:50 spoken, leaving buffer under the 5:00 limit for live pacing/nerves.**
~680 words at a natural, slightly-slower-than-reading pitch pace (~145 wpm) = about 4:40.

Practice this out loud twice with a timer before the event. If you're running long,
cut the bracketed *[trim if short on time]* lines first.

---

### SLIDE 1 — Title [0:00–0:20]
**SAY:**
"Good morning — I'm [your name], and this is Falcon Eye.
Every year, weather is a factor in over a third of fatal aviation accidents.
And in January 2020, a helicopter carrying Kobe Bryant and eight others flew
into fog, the pilot lost outside visual reference, became spatially
disoriented, and lost control. Nine people died because there was no
independent system on board to catch what his eyes and instruments together
couldn't."

---

### SLIDE 2 — The Problem [0:20–1:00]
**SAY:**
"That's not a one-off. Controlled flight into terrain is 17% of all general
aviation fatalities, and over 90% of those accidents are fatal when they
happen. The pattern is always the same: visibility degrades quietly, with no
alarm, and there's no independent backup watching when the primary system —
usually the pilot's own eyes — fails.
*[trim if short on time]* And this isn't just a general aviation problem
anymore. The FAA's new Part 108 framework is making detect-and-avoid a
regulatory requirement for beyond-visual-line-of-sight drone and advanced air
mobility operations. This gap is becoming a compliance problem, not just a
safety one."

---

### SLIDE 3 — The Solution [1:00–1:30]
**SAY:**
"Falcon Eye is an independent forward-looking collision-awareness layer.
Unlike systems that depend on one sensor, we fuse LiDAR and radar — so
detection doesn't disappear the moment visibility does. It's built to be the
backup that's actually still there when the primary system fails."

---

### SLIDE 4 — How It Works [1:30–2:10]
**SAY:**
"The pipeline is simple: sense with LiDAR and radar, detect using a model
that's honest about how fog actually degrades optical sensing, track every
target in real time, triage the most urgent one first, and warn. We didn't
assume LiDAR sees through dense fog — we modeled exactly how and when it
fails, because a system that quietly stops working is worse than one that
tells you it's degraded."

---

### SLIDE 5 — Proof [2:10–3:10] — the most important 60 seconds
**SAY:**
"Here's the proof, not just the pitch. We ran 2,400 randomized simulation
trials across four fog conditions and three sensing modes. LiDAR alone held
100% detection in clear air — and dropped to zero percent in dense fog.
Radar alone, and LiDAR-plus-radar fusion, held 100% detection in every single
condition we tested. That's the whole argument for fusion in one chart.

We also built a multi-target triage engine that ranks simultaneous threats by
urgency and instantly flags the most critical one — validated on three
targets at once with zero false alarms and zero missed threats. And when we
found a bug in our own tracker that could have caused a false alert, we didn't
hide it — we fixed it, and wrote a test that proves it can't happen again."

---

### SLIDE 6 — Roadmap [3:10–3:50]
**SAY:**
"We're at the prototype stage — fully validated in simulation, not yet on
real hardware. The path forward is concrete: bench-calibrate our sensor
models against real LiDAR and radar hardware, validate against real moving
targets on the ground, then a small UAS flight test with a test-range
partner, then degraded-visibility testing. Every phase has a specific,
measurable success criterion — this isn't 'someday we'll test it,' it's a
phased plan with numbers attached."

---

### SLIDE 7 — Ask [3:50–4:25]
**SAY:**
"What we need right now: a university UAS test range or flight-test partner
to run that first real flight test. Access to real LiDAR and radar hardware
for bench calibration. And connections to people who fly — UAM operators, GA
pilots, drone operators — who can tell us if what we built actually matches
what they need in the cockpit."

---

### SLIDE 8 — Close [4:25–4:45]
**SAY:**
"Aviation's next hundred years needs systems that are honest about their own
limits — not ones that quietly assume everything still works. That's Falcon
Eye. Thank you — I'd love to talk to anyone here who's headed to Miramar."

---

## Delivery notes
- **Slide 5 (Proof) is where you slow down**, not speed up. That chart is your
  entire competitive argument — let the "zero percent to one hundred percent"
  line land before moving on.
- Say **"seventeen percent"** and **"two thousand four hundred trials"** out
  loud as words, not read as digits — it's easier to say smoothly live.
- If a name/date slips your mind live (Kobe Bryant's date, Part 108), that's
  fine — the number and the consequence matter more than the citation. Don't
  stop to correct yourself.
- End by literally inviting a conversation ("I'd love to talk to anyone headed
  to Miramar") — this is a room built for exactly that connection, per the
  event description, so make the ask explicit rather than just saying thank you.
