# Falcon Eye: Final Pitch Script

**Format:** under 5 minutes, spoken in English to match the deck.
**Core path:** about 4:15 at a natural pace. With the optional beat, about 4:40. Practice it out loud twice with a timer.

**Slides to show, in order:** 1, 2, 3, 6, 8, 11, 12, (13 optional), 14, 15, 17.
If time allows, add slide 16 (impact) after slide 15. Slides 4, 5, 7, 9 and 10 stay in the deck as backup if someone asks.

Say numbers as words ("two thousand four hundred"), not digits. It is easier to say smoothly and lands harder.

---

## SLIDE 1: Title (0:00 to 0:15)

> "Hi, I'm [your name], and this is Falcon Eye: an independent backup layer for aircraft that keeps detecting threats when visibility, or a single sensor, fails."

*Delivery:* Slow and confident. Do not explain anything technical yet. Move on.

---

## SLIDE 2: The problem (0:15 to 0:55)

> "Weather is a factor in about a third of fatal general-aviation accidents, and controlled flight into terrain is seventeen percent of all general-aviation fatalities.
>
> In January 2020, a helicopter carrying nine people flew into thick fog. The pilot lost visual reference, became spatially disoriented, and lost control. All nine were killed. There was no independent system to catch what the pilot's eyes and instruments together couldn't."

*Delivery:* Pause for a full second after "All nine were killed." Let the room sit with it.

---

## SLIDE 3: The solution (0:55 to 1:20)

> "Falcon Eye removes the single point of failure. LiDAR and radar fail for different physical reasons: LiDAR struggles in fog, radar barely notices it. Fused together, detection survives either one failing."

---

## SLIDE 5: Four technical advantages (OPTIONAL, 15 seconds; backup slide)

> "Four advantages: fused sensing with independent failure modes, edge-native processing with no cloud in the loop, deadline-first hardware acceleration, and slack-priority triage that puts the most critical alert first."

*Delivery:* Use it only if you want a features recap. It repeats ideas from slides 3, 8 and 11, so skipping it costs you nothing.

---

## SLIDE 6: The proof (1:20 to 2:20). Slow down here.

> "Here's the evidence. Two thousand four hundred randomized simulation trials, across four fog levels and three sensing modes.
>
> LiDAR alone: a hundred percent detection in clear air, seventy-three in light fog, fourteen in moderate fog, and zero in dense fog. Radar alone, and fusion, held a hundred percent in every condition we tested.
>
> This is simulation, not flight data, and we label it that way everywhere. We also injected eight failure modes. And when we found a bug in our own tracker that could trigger a false alert, we fixed it and wrote a test so it can't come back."

*Delivery:* This is your strongest slide. Point at the dense-fog bars when you say "zero." Do not rush the sentence "this is simulation, not flight data": it is what makes everything else believable.

---

## SLIDE 8: What's different (2:20 to 2:55)

> "Fusion alone isn't our claim. LiDAR plus radar already ships in commercial drones.
>
> What's different: those systems solve short-range obstacle avoidance. We're built for long-range aircraft detection with real warning lead time. We publish quantified failure curves instead of 'works in harsh conditions.' We triage multiple threats with a reflex path that's unit-tested never to delay a real alert. And we're an independent layer, not tied to one airframe."

*Delivery:* Opening with "fusion isn't our claim" disarms the obvious objection before anyone raises it.

---

## SLIDE 11: The processor (2:55 to 3:35)

> "Underneath this, we're designing our own deadline-first AI accelerator. In 2026, AI chips are hitting a memory wall, with high-bandwidth memory output described as sold out, and a power wall, while edge devices live in a one-to-five watt budget and need answers in milliseconds.
>
> Our answer: ten cores, each with an eight-by-eight systolic array and ping-pong memory, built to meet the deadline, not just the benchmark. We built a cycle-accurate simulator first, and it paid off: it cut missed emergency deadlines from up to forty-seven out of four hundred eighty to zero, and turned a performance cliff into a gradual decline. The design is in RTL now, with an FPGA prototype next."

*Delivery:* Say "in RTL now, FPGA prototype next" with pride. That is real progress, and you earned it. If someone asks about silicon or measured power, the honest answer is in the Q&A section below.

---

## SLIDE 12: The market (3:35 to 4:00)

> "The market is moving ahead of the rules. Advanced air mobility is estimated at fourteen to eighteen billion dollars in 2026. The FAA's Part 108 beyond-line-of-sight rule is still a proposal, not law, but industry reports say more than sixty-five million drone flights have already been logged this year. The safety gap exists today."

*Delivery:* Say "still a proposal, not law" out loud. Overstating this is the easiest way to lose credibility with an informed listener.

---

## SLIDE 13: Early signal (4:00 to 4:15). OPTIONAL, use only if this audience is new to you

> "We presented this at HTCJ's Miramar Pitch Night. Investors and founders asked three sharp questions, and one attendee asked for our technical report. No funding or partnership yet, but it's a real starting point."

---

## SLIDE 14: Roadmap and ask (4:15 to 4:35)

> "Today we're a prototype. Next: bench calibration against real LiDAR and radar, ground tests with real moving targets, then a small UAS flight test through an authorized test-range partner. We need three things: a test-range partner, bench access to LiDAR and radar units, and introductions to operators."

---

## SLIDE 15: How we earn (4:35 to 4:50)

> "We're pre-revenue. Near-term: integration and validation services. Mid-term: licensing our software stack. Long-term, and our largest upside: licensing the processor as IP, an upfront fee plus per-chip royalty, once the RTL is verified and proven on an FPGA."

---

## SLIDE 16: Impact (OPTIONAL, 20 seconds, between slides 15 and 17)

> "What it can change: it targets the failure behind seventeen percent of general-aviation fatalities, it aims to be the missing safety layer for drones and air taxis, and it raises the bar for independent, platform-agnostic safety."

*Delivery:* Say "aims to" and "can." These are projected benefits, and that wording keeps you credible with informed listeners.

---

## SLIDE 17: Close (4:50 to 5:00)

> "Aviation's next hundred years need systems honest about their limits. That's Falcon Eye. I'm [your name]. Let's talk."

*Delivery:* Stop talking. Do not add "thank you" or "any questions." Let the slide sit.

---

## If you're running long

Cut in this order: slide 16 and the optional slide 13, then the second half of slide 11 (keep "in RTL now, FPGA prototype next"), then the first sentence of slide 12. Never cut the "this is simulation, not flight data" line on slide 6.

---

## Q&A cheat sheet: honest answers to the likely questions

**"How is this different from DJI and others who already fuse LiDAR and radar?"**
Fusion itself isn't our claim. Their systems target short-range obstacle avoidance; ours targets long-range aircraft detection with warning lead time, with published failure curves, a tested multi-threat triage path, and an airframe-independent design. The comparison is based on public product descriptions.

**"Radar-only also hit a hundred percent. Isn't that suspiciously good?"**
Fair question. Radar's fog robustness is a modeled assumption: physically motivated, because radar wavelengths barely attenuate in fog, but not measured. Real radar has its own limits, like clutter and coarse resolution. That is exactly why our first phase is calibrating the models against real hardware.

**"What's the catch with fusion?"**
Our first fusion logic accepts a detection from either sensor, and in dense fog that raised false alarms compared to radar alone. We documented it rather than hiding it. Requiring corroboration between sensors, track confirmation, is our next fusion upgrade.

**"Is Part 108 law?"**
No. It's a proposed rule, under federal review since July 2026, with the FAA aiming to publish by the end of 2026. That could slip. Check the latest status the morning you present.

**"Has any of this run on real hardware or in flight?"**
No. Everything so far is simulation, and we say so. The plan is bench calibration first, then ground tests, then a small UAS flight test with an authorized partner. We don't run unauthorized flights.

**"Can it run at the edge, offline?"**
The logic already computes locally with no cloud dependency, and it's computationally light: a physics model, an alpha-beta filter and rule-based triage. Today it's Python in simulation. Porting it to C++ or Rust on a small companion computer is the next engineering step.

**"Could this go into robots?"**
Plausibly. The fusion, triage and hysteresis logic are domain-general. But aviation is our entry point, and robotics is an adjacent market we haven't validated.

**"Why build a processor? Does Falcon Eye need it?"**
Falcon Eye's reflex path is a hard-deadline problem, and the accelerator is designed deadline-first, so it is a natural fit and a natural first workload to target; benchmarking it is on our roadmap. It is also a standalone IP opportunity. Today Falcon Eye's algorithms are light enough to run on ordinary embedded hardware, so the processor is upside, not a dependency.

**"Chip design is capital-intensive. How do you fund it?"**
That is why our path is capital-light: RTL, then an FPGA prototype, then licensing the architecture as IP instead of manufacturing chips ourselves. Those first milestones do not require a tape-out budget.

**"Is there silicon yet? Measured power?"**
Not yet. It is in RTL, and the FPGA prototype is the next step. Power and timing are modeled in our simulator, and the FPGA is where we validate them.

**"Who is your first customer?"**
We don't have one yet. Our targets are UAS and advanced-air-mobility operators and OEMs, and our immediate need is a test-range partner and operator conversations.

---

## Before you go on stage

- [ ] Replace [your name] in your head with your actual name. Practice saying it.
- [ ] Fill in your name and email on slide 17 of the deck.
- [ ] Re-check the Part 108 status one more time, the morning of.
- [ ] Time yourself once with the real slides, not just reading the page.
- [ ] If the audience already saw Miramar, skip slide 13.
