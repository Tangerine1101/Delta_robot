# Video Operation Analysis Report #2 — Multi-Object Pick Lag (second capture)

This report analyzes the physical operation behavior of the robot based on the second video recording [debug_video_2.mp4](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/debug_video_2.mp4) (36.7 s, 848×480, 30 fps). It is a companion to the first analysis ([video_analysis_report.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/video_analysis_report.md)) and the root-cause document ([bug_report_final.md](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/bug_report_final.md)).

---

## 1. Scope Exclusions

As with the first report, the following are recognized as intentional/known conditions and are **excluded** from the bug evaluation in this report:
*   **Robot target alignment vs. Actual lift:** The robot aligns the suction cup over a PCB but does not physically lift it. This is intentional — the physical parameters are not yet precisely calibrated, so `pre_pick_height` and `pickup_height` are deliberately set higher than the real belt surface for safety. "Success" here therefore means **the suction cup descends onto the geometric center of the PCB** (correct alignment/timing), not that the part is physically removed.
*   **Drop / place location:** Where a (notionally) grabbed part is released is not evaluated, since the sorting-bin coordinates have not been adjusted.
*   **Measurement precision:** The camera is handheld and shaky (the whole frame translates between successive frames). Absolute pixel offsets are therefore not reliable for measuring exact millimeter-level lead/lag. The report relies on the qualitative spatial relationship between the suction cup and the nearest PCB at the moment of lowest descent.

---

## 2. Setup & Belt Direction

*   The camera looks across the green conveyor belt with the delta arm / suction cup entering from the top.
*   **No painted boundary lines** are visible in this video (unlike the first video report which had red lines), so the workspace window limits cannot be explicitly referenced.
*   The conveyor transports products **right → left** (products are placed on the right/upstream side and travel to the left/downstream side).
*   Throughout the run, all missed PCBs are still within the visible belt span and reachable pick area at the moment the arm attempts each pick — the misses are purely due to timing/lag, not because the parts traveled out of the physical reach of the arm.

---

## 3. Timeline and Behavior Analysis

Timestamps are approximate (±0.3 s) and refer to the video clock.

### Phase A — First object, isolated (≈0:02 – 0:06)
*   A single PCB enters from the right at ≈0:02. The arm leads to a point in the pick zone, the suction cup descends low and waits, and the PCB drifts into the cup — by ≈0:06 the cup and the PCB center coincide. **Result: Aligned / on-center.**

### Phase B — Small groups, 2–3 objects (≈0:08 – 0:22)
*   The operator feeds further PCBs, now arriving closer together (2–3 in the pick area at once).
*   The robot attempts to process them: after each descent, the arm lifts, returns, and comes back. During that return, there is a visible pause during which the next incoming object keeps moving.
*   Within each small group, the lead object is typically aligned, but the trailing object that arrived during the arm's return passes the cup. The suction cup descends onto the **empty belt behind / next to** that part (e.g., at ≈0:08 the cup descends in the gap between an already-passed part on the left and an incoming part on the right).

### Phase C — Dense burst, 4–5 objects (≈0:24 – 0:31)
*   The operator places 4–5 PCBs in quick succession, travelling as a tight, almost touching cluster.
*   The arm services them strictly one at a time (lift → return → descend). The per-pick return pause means the cluster advances substantially between attempts, and the cup repeatedly comes down **between or behind** the clustered parts rather than on a center.
*   Most of the burst drifts past unpicked and exits the downstream/left edge. By ≈0:31, the belt has cleared with the parts having moved through unhandled.

### Phase D — Belt clears (≈0:32 – 0:37)
*   The remaining parts exit to the left. The arm makes a final descent on the empty belt, and the run ends.

---

## 4. Characteristics of the Bug Symptom

*   **First vs. Subsequent Objects:** The first/lead object in any calm interval is led and aligned accurately. Subsequent objects in a multi-object burst are consistently missed.
*   **Serialization Pause:** There is a distinct lift-return-reapproach pause between consecutive picks during which the robot does not track or plan for the incoming parts in real-time.
*   **Under-lead / Landing Behind:** When the arm returns for a drifted part, the cup comes down upstream of/behind the part rather than on its center.
*   **Density Sensitivity:** Throughput collapses as object spacing decreases; a tight cluster of parts produces cascading misses.
*   **Silent Miss:** The system runs the full motion sequence for each attempt, and a miss is not surfaced as an error.

---

## 5. Limitations of this Capture

*   **Handheld Camera:** Only qualitative spatial judgments are possible; no exact millimeter-level lead/lag measurement.
*   **No Boundary Markers:** The explicit workspace window cannot be referenced; reachable area is judged purely from the belt span and the arm's descent footprint.
*   **No Telemetry Overlay:** The analysis is purely visual without on-screen telemetry logs (`[REPREDICT]`, `[SPEED]`, `pos_EE`).
