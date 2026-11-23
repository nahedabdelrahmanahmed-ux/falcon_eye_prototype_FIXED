# Falcon Eye: The Storytelling Script

**Format:** under 5 minutes, spoken in English. Same slides as the main script.
**Slides in order:** 1, 2, 3, 6, 8, 11, 12, 14, 15, 17. (Slide 16, Impact, is optional.)

**The story in one line:** a pilot loses the horizon, we ask what could back them up, we build it honestly, we get challenged, we go deeper, and now we need partners to fly it.

**How to tell it:** talk to one person in the room, not to the crowd. Slow down at the scene and at the failure. Speed up slightly through the facts. The pauses are part of the script.

---

## ACT 1: THE SCENE

### SLIDE 1 (0:00 to 0:30)

> "Imagine you're flying. The horizon you've trusted the whole flight starts to fade into white. No ground. No sky. Just your eyes, your instruments, and a few seconds.
>
> I'm [your name], and this is Falcon Eye: the story of building an independent backup for exactly that moment."

*Delivery:* Start before the clicker, before the slide matters. Quiet voice. Pause after "a few seconds."

*Optional personal line (use only if true):* "I've cared about this since [one honest sentence about why]."

### SLIDE 2 (0:30 to 1:05)

> "In January 2020, that moment happened to a helicopter carrying nine people. The pilot flew into thick fog, lost visual reference, became spatially disoriented, and lost control. All nine were killed. There was no independent system to catch what his eyes and instruments together couldn't.
>
> And it isn't rare. Weather is a factor in about a third of fatal general-aviation accidents, and controlled flight into terrain is seventeen percent of all general-aviation fatalities."

*Delivery:* Gentle and factual. Do not dramatize beyond what the investigators found. Pause a full second after "All nine were killed."

---

## ACT 2: THE QUESTION

### SLIDE 3 (1:05 to 1:30)

> "So we asked one question: what if the backup didn't share the same weakness? LiDAR struggles in fog. Radar barely notices it. Two sensors, two different ways to fail, and fused together, one of them is still watching."

*Delivery:* Lift your energy here. This is the idea arriving.

---

## ACT 3: THE STRUGGLE

### SLIDE 6 (1:30 to 2:25). The heart of the story. Slow down.

> "So we built a simulator, and we made it honest. We didn't assume LiDAR sees through fog. We modeled how fog really weakens it.
>
> The first result stung. In dense fog, LiDAR alone detected nothing. Zero percent. Across two thousand four hundred randomized trials, radar alone and fusion held a hundred percent in every condition we tested. All simulation, not flight data, and we say so every time.
>
> But the simulator embarrassed us too. Our own tracker once raised a critical alert about a target almost a minute away. We could have hidden that. Instead we traced it, fixed it, and wrote a test so it can never come back. That's the culture we think aviation safety deserves."

*Delivery:* "Zero percent" lands best if you stop for a beat before and after it. Say "embarrassed us" with a small smile; it earns trust.

---

## ACT 4: THE TURN

### SLIDE 8 (2:25 to 3:00)

> "A few weeks ago, at a pitch night, someone told us: 'LiDAR and radar fusion already exists.' They were right. Drones ship with it.
>
> So what are we? Not a drone dodging a wire. Long-range detection of another aircraft, with real warning lead time. Published failure curves instead of 'works in harsh conditions.' A triage engine that ranks threats and is unit-tested never to delay a real alert. And an independent layer, not tied to any one airframe."

*Delivery:* Say "They were right" plainly, with no defensiveness. The honesty is the persuasion.

### SLIDE 11 (3:00 to 3:40)

> "Then we asked ourselves a harder question: what should run this, at the edge, with no cloud, on a deadline?
>
> In 2026, AI chips are fighting a memory wall and a power wall, with high-bandwidth memory output described as sold out. So we started designing our own accelerator: ten cores, eight-by-eight systolic arrays, built deadline-first. Our simulator found the flaws before we wrote RTL: missed emergency deadlines dropped from up to forty-seven out of four hundred eighty to zero. It's in RTL now, with an FPGA prototype next."

*Delivery:* Confident and a little proud. "In RTL now, FPGA next" is real progress. Say it that way.

---

## ACT 5: THE ROAD AHEAD

### SLIDE 12 (3:40 to 4:05)

> "The timing is real. Industry reports count more than sixty-five million drone flights already. The FAA's Part 108 beyond-line-of-sight rule is still only a proposal. The flights aren't waiting for it."

*Delivery:* Say "still only a proposal" clearly. It keeps you credible with anyone who follows the regulation.

### SLIDE 14 (4:05 to 4:30)

> "Where are we? A prototype that has never touched a real aircraft, and we're telling you that. Next: bench calibration, ground tests, then a small UAS flight through an authorized test-range partner. To get there we need three things: a test-range partner, bench access to LiDAR and radar hardware, and introductions to operators."

### SLIDE 15 (4:30 to 4:45)

> "We're pre-revenue. First, validation services. Then software licensing. And our largest upside: licensing the processor as IP, once the RTL is verified and proven on an FPGA."

---

## CLOSE

### SLIDE 17 (4:45 to 5:00)

> "Remember the horizon fading into white? Our job is to make sure the pilot isn't the last line of defense when it does. Aviation's next hundred years need systems honest about their limits. That's Falcon Eye. I'm [your name]. Let's talk."

*Delivery:* Slow on the callback. Then stop talking. Do not add "thank you." Let the slide sit.

---

## OPTIONAL: SLIDE 16, Impact (adds about 20 seconds; use only if you are under time)

> "What it can change: it targets the failure behind seventeen percent of general-aviation fatalities, it aims to be the missing safety layer for drones and air taxis, and it raises the bar for independent, platform-agnostic safety."

---

## Rules that keep the story honest

- Never say what the real pilot was thinking or feeling. Say only what investigators found.
- Say "simulation" whenever you give a result. It is what makes the story believable.
- Say "proposal" for Part 108, "aims to" for impact, and "in RTL, FPGA next" for the processor.
- If you run long, cut slide 12 first, then the second half of slide 11.

## If you get questions

Use the Q&A cheat sheet in `Falcon_Eye_Final_Pitch_Script.md`. The answers there still apply word for word.

## Before you go on stage

- [ ] Say your name out loud once. Practice the first thirty seconds until you can do them without looking at the page.
- [ ] Fill in your name and email on slide 17 of the deck.
- [ ] Re-check the Part 108 status the morning of.
- [ ] Time one full run with the real slides.
