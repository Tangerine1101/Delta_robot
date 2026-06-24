# Real-Time Pick/Place Redesign

## Purpose

The real conveyor pick path is rebuilt as a fresh PC-side scheduler because the
old flow parked the arm from one noisy belt-speed sample, then waited on timing
derived from that same sample. At dense object spacing this produced the observed
"arrive, wait, miss" behavior: the arm could reach a downstream park point that
the object would never align with at the expected time.

The new flow keeps the camera tracker, Siemens belt position, and Omron
end-effector pose live while the robot is moving. The arm still plans a stable
straight-down pick point, but the pick command is fired by a live position gate:
the object must reach the parked gripper position.

## Thread Model

The real `production` and `test_conveyor` paths use two scheduler threads.

The perception/state thread runs at about 25 ms. It performs the only regular
PLC status read, updates belt position/speed and `pos_EE`, polls vision
detections, refreshes the `BeltTracker`, prunes only unclaimed stale objects,
and emits dashboard status/detection events. It never renders the OpenCV GUI.

The main thread selects an object, predicts the pick point, builds goto/pick
packets, claims the object, dispatches commands, and waits on shared state. It
does not issue status reads inside wait loops; arm arrival and object arrival are
both checked from the latest state published by the perception thread. The
OpenCV GUI pump remains on the main thread because Qt requires that.

All scheduler PLC round trips share one IPC lock. The perception thread locks it
around status reads through `ConveyorSpeedSource`; the main thread locks it
around Omron/Siemens dispatches through `RealtimePickExecutor`. This prevents
two IPC callers from consuming each other's queued responses.

## Planning And Priority

The iterative `_predict_pick_position` solver is kept as the reachability model.
The real-time caller uses it to find the earliest feasible intercept, then
applies the configured minimum lead:

- `intercept_lead_time_s = 1.6`
- `final_pick_time = max(earliest_time, now + intercept_lead_time_s)`
- If this lead pushes a danger-zone object past `u_max`, clamp the pick point to
  the downstream workspace edge and verify the arm can still arrive before the
  object reaches that edge.

Selection uses two tiers:

- Tier 1: objects in the downstream danger zone, defined as the final third of
  the workspace in `u`, sorted most-downstream first.
- Tier 2: all other catchable objects, sorted by estimated cycle distance:
  current arm position to predicted pick point plus predicted pick point to the
  object's configured sorting bin.

Unknown object types, claimed objects, lateral out-of-window objects, and
objects that cannot be reached before their pick point are skipped.

## Execution

Each selected object remains in the tracker while claimed. This lets live camera
updates keep refreshing the object anchor until the pick gate fires, and it also
prevents the perception thread from pruning the in-flight target.

Execution sequence:

1. Dispatch the goto packet.
2. Wait until shared `pos_EE` reaches the goto endpoint.
3. Wait until the claimed object's live `u` reaches the parked pick `u`.
4. Dispatch the Siemens `rotate_absolute` command.
5. Dispatch the original pick packet, which descends straight down at the parked
   pick point and transfers to the object's own bin.
6. Wait until shared `pos_EE` reaches the place endpoint.
7. Remove the object from the tracker, remember its id briefly so vision does not
   recreate it, and advance the scheduler's current arm position to the bin.

The positional pick gate includes an intentionally empty production hook:

```python
def _belt_lead_offset_mm(belt_speed_mm_s: float) -> float:
    return 0.0
```

It is reserved for future compensation of command, network, or mechanism
latency. At the current 50-100 mm/s belt speeds this offset is intentionally
zero. Implementing a speed-dependent model is tracked as a roadmap item — see
`README.md` §5 Future Roadmap (the single source of truth for that work).

## PLC Impact

There are no PLC changes. Siemens DB offsets, Omron packet layout,
`interpolar_points = 7`, command ids, and packet padding remain unchanged.

## Verification

Required checks after scheduler edits:

```bash
python3 -m unittest tests.test_trajectory_planning -v
python3 -m py_compile main.py modules/cli.py modules/EthernetCom.py modules/image_processing.py modules/scheduler.py modules/test_module.py modules/conveyor.py modules/interface.py
python3 main.py --scheduler --scenario test_throughput --duration 12.0 --simulate-executor
python3 main.py --scheduler --scenario test_accuracy --duration 5.0 --simulate-executor
```

Real-belt verification:

```bash
python3 main.py --scheduler --scenario test_conveyor --interface --duration 30
```

Expected behavior: `[SPEED]` and `[DETECT]` continue during robot motion, danger
zone objects are selected first, the arm parks downstream with the configured
lead, and the pick packet fires when the live tracked object reaches the parked
pick position.
