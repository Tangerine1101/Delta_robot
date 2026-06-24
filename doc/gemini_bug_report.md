# Bug Report: Multi-Object Picking Latency and Misses

## 1. Symptom Description
During conveyor belt operations with the real robot:
* **Single object picking** is highly reliable and almost always succeeds.
* When **multiple objects (2 or more) appear on the belt simultaneously**, the first object is picked successfully. However, the second and subsequent objects are missed because the robot arm arrives **too late** at the pick coordinates, despite the objects still physically residing within the workspace boundaries.

---

## 2. Root Cause Analysis
The issue stems from a combination of a physical/PLC Z-axis limit clamp, a config mismatch, and a boundary condition race in the PC-side phase completion gating logic.

### 2.1. Z-axis Height Mismatch & PLC Clamping
* In [config.json](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/config.json#L159), the pre-pick height is configured as:
  ```json
  "pre_pick_height": -295.0
  ```
* However, actual telemetry logs show that the robot's physical Z position never goes below `-290.0` mm during the `goto` phase. This indicates that the Omron PLC has a software axis limit or mechanical constraint that clamps the Cartesian Z-coordinate at **`-290.0` mm**.

### 2.2. Gating Gaps in `_wait_for_phase_completion`
In [scheduler.py](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/modules/scheduler.py#L871-L922), the function `_wait_for_phase_completion` blocks until the robot arm physically arrives at the commanded waypoint.
* The position tolerance is retrieved from configuration or defaults to **`5.0` mm**.
* At the end of the `goto` phase, the target Z coordinate is `_pre_pick_z = -295.0` mm, but the robot is physically clamped at `pos_EE[2] ≈ -290.0` mm.
* The 3D distance between the actual position and the target is calculated as:
  $$\text{distance} = \sqrt{\Delta x^2 + \Delta y^2 + ((-290.0) - (-295.0))^2} \approx 5.0002\text{ mm}$$
* Because the distance is slightly larger than the tolerance threshold ($5.0002\text{ mm} > 5.0\text{ mm}$), the arrival condition is never satisfied (`distance <= position_tolerance_mm` evaluates to `False`).
* The function hangs and waits until the hard deadline (timeout) expires:
  $$\text{hard\_deadline} = \text{expected\_duration} + \text{wait\_margin\_s}$$
  Since the nominal XY speed is set conservatively to $120.0\text{ mm/s}$ in `config.json`, the calculated expected duration is around $1.5\text{ seconds}$, which, combined with a `wait_margin_s = 1.0`, results in a **silent 2.5-second timeout** for every `goto` phase.

### 2.3. Why Single Picks Succeed but Multi-Picks Fail
* **Single/First Object**: When the first object is planned, it is far upstream. The robot executes the `goto` phase and reaches the pre-pick position within 0.3s. The PC waits for the 2.5s timeout. Since the object is far upstream, it still takes more than 2.5s to reach the workspace. The robot waits in `_wait_until_pick_dispatch` after the timeout expires and eventually executes a successful pick.
* **Second/Subsequent Objects**: When the second object is planned (immediately after placing the first), it is already in or near the workspace. The PC dispatches the `goto` phase and enters the 2.5s timeout. Because 2.5s is extremely long, the second object floats past the pick position while the PC thread is blocked in the timeout loop. The PC dispatches the `pick` phase 2.5s too late, causing a miss.

---

## 3. Log Evidence
From the live execution log [test_conveyor.log](file:///home/tangerine/Share/Global%20Share/Documents/Delta_robot/test_conveyor.log):

### 3.1. Constant Clamping at `-290.0` mm
At the end of the `goto` phase for every plan, the logged position `pos_EE` Z-coordinate is clamped at `-290.0` mm:
* **Plan 1 (Pick Phase Start)**:
  ```json
  [PLC] {"task_doing": 3, "task_state": 2, "pos_angular": ..., "pos_EE": [-30.7757, -117.2629, -290.0002]}
  ```
* **Plan 2 (Pick Phase Start)**:
  ```json
  [PLC] {"task_doing": 3, "task_state": 2, "pos_angular": ..., "pos_EE": [-33.5296, -118.7280, -289.9998]}
  ```
* **Plan 3 (Pick Phase Start)**:
  ```json
  [PLC] {"task_doing": 3, "task_state": 2, "pos_angular": ..., "pos_EE": [-47.3034, -126.0509, -290.0001]}
  ```

### 3.2. Elapsed Time Gaps
Comparing timestamps before and after plan execution reveals a consistent $\approx 6.2\text{ second}$ total duration per plan (2.5s timeout for `goto` + wait time + 2.5s timeout for `pick` if placing Z also runs into tolerance limits):
* Plan 2 starts planning around $t = 34991.202$ seconds and the next speed sample only resumes at $t = 34997.805$ seconds, showing an execution delay of **$6.6$ seconds**.
* Plan 3 shows a delay of **$7.32$ seconds** ($35009.983 - 35002.660$).

---

## 4. Recommendations

### Option 1: Adjust heights in `config.json` (Recommended)
Raise `pre_pick_height` to a value that the robot can physically reach without hitting the Z-axis software limit (i.e. above `-290.0` mm):
```json
"pre_pick_height": -285.0
```
*This is the cleanest fix as it maintains the tight $5.0\text{ mm}$ arrival tolerance and ensures the robot physically completes its motion.*

### Option 2: Increase the position tolerance on the PC
If the heights in the config must remain as they are, increase the position tolerance to absorb the Z clamping offset:
```json
"pick_arrival_tolerance_mm": 8.0
```
This allows the $5.0002\text{ mm}$ distance discrepancy to be accepted as arrived, instantly terminating the `goto` phase check.

### Option 3: Extend PLC Axis Limits
If the robot must go lower to perform picks on thinner objects, adjust the software limits of the Z-axis in the Omron NX1P2 PLC configuration (e.g. down to `-315.0` mm) using Sysmac Studio.
