# Video Operation Analysis Report — Multi-Object Pick Lag

This report analyzes the physical operation behavior of the robot based on the video recording `debug_video.mp4` and the workspace boundary image `workspace_video.png` located in the `doc/` directory.

---

## 1. Scope Exclusions

As requested by the operator, the following mechanical and spatial configuration factors are recognized as intentional behaviors and are **excluded** from the bug evaluation in this report:
*   **Robot target alignment vs. Failure to lift the PCB:** The robot aligns with the target but does not physically pick up the PCB. This is intentional because the physical parameters of the robot have not yet been precisely calibrated. For safety reasons, the pre-pick height (`pre_pick_height`) and pick height (`pick_height`) were deliberately configured higher than actual values.
*   **Improper drop location (Place point):** The fact that the PCB is dropped back onto the conveyor belt or falls outside the receiving box is expected/normal, as the spatial coordinates of the box have not yet been adjusted.

---

## 2. Workspace Boundary Analysis

Based on the image [workspace_video.png](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/workspace_video.png):
*   The two red lines drawn on the conveyor protective glass mark the active workspace of the robot on the conveyor.
*   The conveyor transports products from **right to left** (from upstream into the pick area and towards downstream).
*   During the entire test in the video, all PCBs that were missed by the robot **remained fully within the boundary between these two red lines**, confirming that the products did not drift outside the robot's physical working workspace.

---

## 3. Timeline and Behavior Analysis of the Three Drops

Below is a detailed chronological breakdown of the robot's actions for each drop event in [debug_video.mp4](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/doc/debug_video.mp4):

### Drop 1: Single Object (0:02 - 0:15)
*   **Description:** A single PCB is placed on the conveyor at `0:02`.
*   **Behavior:** The robot detects the target from a distance, traverses to lead the target, tracks it, and descends the gripper **exactly at the geometric center** of the PCB at `0:11`. The single-object pick cycle is fully successful in terms of positioning.

### Drop 2: Three Consecutive Objects (0:18 - 0:40)
*   **Description:** The operator places three PCBs in quick succession on the conveyor from `0:18` to `0:20`.
*   **Behavior per object:**
    *   **PCB #1:** The robot arm leads, tracks, and descends **exactly on the center** of the PCB at `0:28`.
    *   **PCB #2:** After completing the place phase for PCB #1, the robot returns to pick PCB #2. The gripper descends at `0:31` but is significantly **delayed/lagging behind** (offset to the right/upstream side). It lands on the empty conveyor belt because PCB #2 has already drifted past.
    *   **PCB #3:** The robot executes the next pick cycle with an accumulated lag. The gripper descends at `0:38` and again lands **behind** PCB #3 on the empty belt.

### Drop 3: Two Consecutive Objects (0:42 - 0:58)
*   **Description:** The operator places two PCBs in succession at `0:42` and `0:44`.
*   **Behavior per object:**
    *   **PCB #1:** Since the robot was at its idle (home) position and had sufficient preparation time, the gripper descends **exactly on the center** of PCB #1 at `0:45`.
    *   **PCB #2:** After completing the place phase for PCB #1, the robot returns to the conveyor to handle PCB #2. Similar to Drop 2, the gripper descends at `0:55` but lags behind the conveyor speed, landing on the belt **behind** PCB #2.

---

## 4. Characteristics of the Bug Symptom

*   **First vs. Subsequent Object Behavior:**
    *   The first object in any batch (or a single running object) is always led accurately and targeted precisely at its center.
    *   Any subsequent objects in a multi-object burst (the 2nd and 3rd objects) are consistently missed, with the robot landing upstream/to the right of the object.
*   **Response Lag:** There is a noticeable delay/pause from the moment the robot completes the cycle for the previous object to when it starts moving to lead the next object.
*   **Under-lead Offset:** The physical touchdown point of the gripper for subsequent objects fails to match the real-time position of the object on the conveyor, even though the objects are still within the workspace boundaries (between the two red lines).
