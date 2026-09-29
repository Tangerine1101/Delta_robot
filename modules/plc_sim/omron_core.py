"""Scan-accurate port of the deployed Omron program ``Matching_Code_10``.

Source of truth: the Sysmac project ``OMRON/matching code/Matching_Code_10.smc2``
(rung-by-rung description in ``doc/plc/program-old.md``). Every rung of
``Program0/Section0`` that affects the PC-visible behaviour is executed once per
4 ms scan, in rung order, including the program's defects — the point of this
module is to behave like the real PLC, not like an ideal robot.

Defects reproduced on purpose (IDs from ``doc/plc/version-diff-and-defects.md``):
O1 silent joint-limit IK, O2 command 3 on a running chain, O3 zero-length stall,
O5 stale pump-release time, P2 sticky IK error, P5 no completion report, P7 no
backward pass / braking sign, P11 State-10 ramp, P12 junction stalls.

:class:`Patches` switches on the proposed PLC fixes for A/B runs; :class:`Quirks`
holds the two Sysmac run-time behaviours the project file cannot settle.
"""
from __future__ import annotations

import math
import re
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Any

from modules.core.kinematics import calc_forward_kinematic, calc_inverse_kinematics, f32
from modules.plc_sim.plant import ServoAxis

SCAN_S = 0.004
N_POINTS = 7

# Homing (MC_Home_Delta + rung 6): switch angles expressed in the final joint frame,
# search speed, calibration move and the TP window that gates rung 6.
HOME_SWITCH_DEG = (-28.9, -27.5, 27.7)
HOME_SEARCH_DEG_S = 12.0
HOME_CALIB_DEG_S, HOME_CALIB_ACC = 10.0, 20.0
HOME_DONE_WINDOW_S = 5.0
# Goto_Absolute: MC_MoveAbsolute per axis.
GOTO_DEG_S, GOTO_ACC = 15.0, 30.0


@dataclass
class Quirks:
    # Value a Sysmac FUNCTION output takes when the body RETURNs before writing it
    # (O1 / O9). IEC functions have no memory, so the default initial value 0.0 is
    # the expected behaviour; bench check B1 confirms it.
    unassigned_fun_output: float = 0.0
    # TP instruction: True = PT is re-read every scan; False = latched at the
    # rising edge (O5).
    tp_pt_live: bool = False


@dataclass
class Patches:
    """Proposed PLC fixes (version-diff-and-defects.md §3a); all off = as deployed."""

    o1_limit_is_error: bool = False  # IK returns TRUE when the −20° limit trips
    o2_reject_busy: bool = False     # command 3 rejected (task_state 4) while the chain runs
    p2_recover: bool = False         # State 99 clears itself once Execute is FALSE


# ---------------------------------------------------------------------------
# Motion-control stand-ins
# ---------------------------------------------------------------------------
class SyncMoveAbsolute:
    """MC_SyncMoveAbsolute: a rising Execute takes the axis (multi-execution aborts
    the previous owner); while Busy the axis setpoint is ``Position`` every scan.
    With Execute FALSE the axis holds its last setpoint."""

    def __init__(self, axis: ServoAxis) -> None:
        self.axis = axis
        self._prev = False
        self.busy = False

    def __call__(self, execute: bool, position: float) -> None:
        if execute and not self._prev:
            self.axis.owner = self
        self._prev = execute
        self.busy = execute and self.axis.owner is self
        if self.busy:
            self.axis.command(position)


class MoveAbsolute:
    """MC_MoveAbsolute with a trapezoidal profile (Execute edge-triggered)."""

    def __init__(self, axis: ServoAxis, velocity: float, accel: float) -> None:
        self.axis, self.v, self.a = axis, velocity, accel
        self._prev = False
        self._plan: tuple[float, float, float, float] | None = None  # start, dist, T, t
        self.done = False

    def __call__(self, execute: bool, position: float) -> None:
        if execute and not self._prev:
            start = self.axis.setpoint
            dist = position - start
            d = abs(dist)
            v = min(self.v, math.sqrt(d * self.a)) if d > 0 else 0.0
            t_total = (d / v + v / self.a) if v > 0 else 0.0
            self._plan = (start, dist, t_total, 0.0)
            self.axis.owner = self
            self.done = False
        self._prev = execute
        if self._plan is not None and self.axis.owner is self:
            start, dist, t_total, t = self._plan
            t = min(t + SCAN_S, t_total)
            self._plan = (start, dist, t_total, t)
            self.axis.command(start + math.copysign(_trap_s(abs(dist), self.v, self.a, t), dist))
            if t >= t_total:
                self._plan = None
                self.done = True
        elif self._plan is not None:
            self._plan = None  # aborted by another instruction
        if not execute:
            self.done = False


def _trap_s(d: float, v: float, a: float, t: float) -> float:
    if d <= 0.0:
        return 0.0
    v = min(v, math.sqrt(d * a))
    t_a = v / a
    t_total = d / v + t_a
    if t < t_a:
        return 0.5 * a * t * t
    if t < t_total - t_a:
        return 0.5 * a * t_a * t_a + v * (t - t_a)
    return d - 0.5 * a * (t_total - t) ** 2


class TP:
    """IEC pulse timer."""

    def __init__(self, live_pt: bool) -> None:
        self.live_pt = live_pt
        self.q = False
        self.et = 0.0
        self._prev = False
        self._pt = 0.0

    def __call__(self, inp: bool, pt: float) -> None:
        if inp and not self._prev and not self.q:
            self.q, self.et, self._pt = True, 0.0, pt
        self._prev = inp
        limit = pt if self.live_pt else self._pt
        if self.q:
            self.et += SCAN_S
            if self.et >= limit:
                self.q = False
        elif not inp:
            self.et = 0.0


# ---------------------------------------------------------------------------
# MC_Inter_Curve_Vel
# ---------------------------------------------------------------------------
@dataclass
class ICVInputs:
    a: tuple[float, float, float]
    b: tuple[float, float, float]
    c: tuple[float, float, float]
    v_max: float
    a_max: float
    d_max: float
    v_start_req: float
    blend_mode: bool


class InterCurveVel:
    """Port of the MC_Inter_Curve_Vel ST body (States 0/10/1/2/3/99)."""

    def __init__(self, name: str, axes: list[ServoAxis], quirks: Quirks, patches: Patches) -> None:
        self.name = name
        self.axes = axes
        self.quirks = quirks
        self.patches = patches
        self.sync = [SyncMoveAbsolute(ax) for ax in axes]
        self.state = 0
        self.theta = [ax.actual for ax in axes]  # idle scans since power-up: Theta = Act.Pos
        self.target = [0.0, 0.0, 0.0]
        self.start = [0.0, 0.0, 0.0]
        self.cycle_count = 0
        self.enable_motion = False
        self.use_standard_scurve = False
        self.actual_vmax = 100.0
        self.L = self.t = self.T_total = 0.0
        self.t_acc = self.t_dec = self.t_run = 0.0
        self.S_acc = self.S_dec = self.S_run = 0.0
        # outputs
        self.out_error_ik = False
        self.out_done_inter = False
        self.out_done_blend = False
        self.out_v_end = 0.0
        self.t_total_estimate = 0.0
        self.t5_out = 0.0
        self.ik_limit_trips = 0

    def _ik(self, p: tuple[float, float, float]) -> tuple[bool, tuple[float, float, float]]:
        ret, th, limit = calc_inverse_kinematics(
            *p, unassigned=self.quirks.unassigned_fun_output,
            limit_is_error=self.patches.o1_limit_is_error,
        )
        if limit and not ret:
            self.ik_limit_trips += 1
        return ret, th

    def call(self, execute: bool, inp: ICVInputs) -> bool:
        """One FB call. ``execute`` is the VAR_IN_OUT value; returns its new value."""
        act = [ax.actual for ax in self.axes]
        if self.state == 0:
            if execute:
                self.out_error_ik = False
                self.out_done_inter = False
                err, th = self._ik(inp.a)
                if err:
                    self.out_error_ik = True
                    self.state = 99
                else:
                    self.target = list(th)
                    self.enable_motion = True
                    if inp.v_start_req > 0.0:
                        self.theta = list(self.target)
                        self.state = 1
                    else:
                        self.start = list(act)
                        self.cycle_count = 1
                        self.state = 10
            else:
                self.out_done_blend = False
                self.enable_motion = False
                self.theta = list(act)
        elif self.state == 10:
            if self.cycle_count <= 20:
                k = float(self.cycle_count) / 20.0
                self.theta = [s + (g - s) * k for s, g in zip(self.start, self.target)]
                self.cycle_count += 1
            else:
                self.state = 1
        elif self.state == 1:
            execute = self._plan(execute, inp)
        elif self.state == 2:
            if self.t <= self.T_total:
                s_t = self._s_of_t(inp)
                a, b = inp.a, inp.b
                p = tuple(a[i] + (s_t / self.L) * (b[i] - a[i]) for i in range(3))
                err, th = self._ik(p)
                if err:
                    self.out_error_ik = True
                    self.state = 99
                    self._drive()
                    return execute
                self.theta = list(th)
                self.t += SCAN_S
            else:
                if inp.blend_mode and self.out_v_end > 0.0:
                    self.out_done_blend = True
                    execute = False
                    self.state = 0
                else:
                    self.state = 3
        elif self.state == 3:
            self.enable_motion = False
            self.out_done_inter = all(abs(self.theta[i] - act[i]) <= 0.0005 for i in range(3))
            if not any(s.busy for s in self.sync):
                execute = False
                self.state = 0
        elif self.state == 99:
            if self.patches.p2_recover and not execute:
                self.out_error_ik = False
            execute = False
            self.enable_motion = False
            if not self.out_error_ik:
                self.state = 0
        self._drive()
        return execute

    def _drive(self) -> None:
        for s, th in zip(self.sync, self.theta):
            s(self.enable_motion, th)

    def _plan(self, execute: bool, inp: ICVInputs) -> bool:
        a, b, c = inp.a, inp.b, inp.c
        V, A, D, V0 = inp.v_max, inp.a_max, inp.d_max, inp.v_start_req
        L = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        self.L = L
        if L <= 0.0:
            self.state = 0
            return False
        if not inp.blend_mode:
            self.out_v_end = 0.0
            self.use_standard_scurve = V0 == 0.0
            vm = V
            if self.use_standard_scurve:
                if L < 0.75 * vm * vm * (1.0 / A + 1.0 / D):
                    vm = math.sqrt(L / (0.75 * (1.0 / A + 1.0 / D)))
                self.t_acc = 1.5 * vm / A
                self.t_dec = 1.5 * vm / D
                self.S_acc = 0.5 * vm * self.t_acc
                self.S_dec = 0.5 * vm * self.t_dec
                self.S_run = L - self.S_acc - self.S_dec
                self.t_run = self.S_run / vm
            else:
                s_limit = abs(vm * vm - V0 * V0) / (2.0 * A) + vm * vm / (2.0 * D)
                if s_limit > L:
                    vm = math.sqrt((2.0 * A * D * L + D * V0 * V0) / (A + D))
                self.t_acc = abs(vm - V0) / A
                self.t_dec = abs(vm) / D
                self.S_acc = (V0 + vm) * 0.5 * self.t_acc
                self.S_dec = vm * 0.5 * self.t_dec
                self.S_run = max(L - self.S_acc - self.S_dec, 0.0)
                self.t_run = self.S_run / vm
        else:
            self.use_standard_scurve = False
            v1 = [b[i] - a[i] for i in range(3)]
            v2 = [c[i] - b[i] for i in range(3)]
            len2 = math.sqrt(sum(x * x for x in v2))
            if L > 0.0 and len2 > 0.0:
                cos_t = sum(v1[i] * v2[i] for i in range(3)) / (L * len2)
                cos_t = max(-1.0, min(1.0, cos_t))
            else:
                cos_t = 1.0
            v_corner = V * math.sqrt((cos_t + 1.0) / 2.0)
            v_reach = math.sqrt(V0 * V0 + 2.0 * A * L)
            v_end = min(v_corner, v_reach, V)
            self.out_v_end = v_end
            vm = V
            s_limit = abs(vm * vm - V0 * V0) / (2.0 * A) + abs(vm * vm - v_end * v_end) / (2.0 * D)
            if s_limit > L:
                vm = math.sqrt((2.0 * A * D * L + D * V0 * V0 + A * v_end * v_end) / (A + D))
            vm = max(vm, V0, v_end)
            self.t_acc = abs(vm - V0) / A
            self.t_dec = abs(vm - v_end) / D
            self.S_acc = (V0 + vm) * 0.5 * self.t_acc
            self.S_dec = (v_end + vm) * 0.5 * self.t_dec
            self.S_run = max(L - self.S_acc - self.S_dec, 0.0)
            self.t_run = self.S_run / vm
        self.actual_vmax = vm
        self.T_total = self.t_acc + self.t_run + self.t_dec
        self.t_total_estimate = self.T_total + 0.08 if V0 == 0.0 else self.T_total
        self.t5_out = self.t_total_estimate
        self.t = 0.0
        self.state = 2
        return execute

    def _s_of_t(self, inp: ICVInputs) -> float:
        t, vm = self.t, self.actual_vmax
        if self.use_standard_scurve:
            if t <= self.t_acc:
                tau = t / self.t_acc
                return vm * self.t_acc * (tau ** 3 - 0.5 * tau ** 4)
            if t <= self.t_acc + self.t_run:
                return self.S_acc + vm * (t - self.t_acc)
            tau = (t - (self.t_acc + self.t_run)) / self.t_dec
            return self.S_acc + self.S_run + vm * self.t_dec * (tau - tau ** 3 + 0.5 * tau ** 4)
        if t <= self.t_acc:
            s = inp.v_start_req * t + 0.5 * inp.a_max * t * t
        elif t <= self.t_acc + self.t_run:
            s = self.S_acc + vm * (t - self.t_acc)
        else:
            tp = t - (self.t_acc + self.t_run)
            s = self.S_acc + self.S_run + vm * tp - 0.5 * inp.d_max * tp * tp
        return min(s, self.L)


# ---------------------------------------------------------------------------
# Program0
# ---------------------------------------------------------------------------
def _zero_pc() -> dict[str, Any]:
    return {
        "commandID": 0, "argument_number": 0,
        "argument_x": [0.0] * N_POINTS, "argument_y": [0.0] * N_POINTS,
        "argument_z": [0.0] * N_POINTS, "argument_time": [0.0] * N_POINTS,
        "argument_e": [0] * N_POINTS, "bit_doing": 0,
    }


def _zero_plc() -> dict[str, Any]:
    return {
        "pos_angular": [0.0] * 3, "pos_EE": [0.0] * N_POINTS,
        "task_doing": 0, "task_state": 0, "Total_Time_Estimate": [0.0] * N_POINTS,
    }


_REAL_FIELDS = {"argument_x", "argument_y", "argument_z", "argument_time"}
_TAG_RE = re.compile(r"^(pc_package|plc_package)\.(\w+)(?:\[(\d+)\])?$")


@dataclass
class SimEvent:
    t: float
    kind: str
    detail: dict[str, Any] = field(default_factory=dict)


class OmronPLC:
    """Program0 of Matching_Code_10, one :meth:`scan` = one 4 ms primary task."""

    def __init__(
        self,
        quirks: Quirks | None = None,
        *,
        patches: Patches | None = None,
        start_angles_deg: tuple[float, float, float] = (0.0, 0.0, 0.0),
        servo_tau_s: float = 0.0,
        servo_on: bool = True,
        max_joint_speed_deg_s: float = 1800.0,
    ) -> None:
        self.quirks = quirks or Quirks()
        self.patches = patches or Patches()
        self.axes = [ServoAxis(a, servo_tau_s) for a in start_angles_deg]
        self.max_joint_speed_deg_s = max_joint_speed_deg_s
        self.time_s = 0.0
        self.pc_package = _zero_pc()
        self.plc_package = _zero_plc()
        self.events: deque[SimEvent] = deque(maxlen=2000)
        self.event_counts: Counter[str] = Counter()
        # I/O
        self.home_switch = [False, False, False]
        self.mc_on_btn_flag = servo_on
        self.pump_out = False
        self.valve_out = True
        # Program0 variables
        self.icv = [InterCurveVel(f"MC_Inter_Curve_Vel_{k}", self.axes, self.quirks, self.patches)
                    for k in range(6)]
        self.icv_pos = [[0.0, 0.0, 0.0] for _ in range(N_POINTS)]
        self.icv_e = [0] * N_POINTS
        self.icv_start_new_turn = False
        self.blend_done = [False] * 6
        self.vend = [0.0] * 6
        self.icv_t = [0.0] * N_POINTS
        self.icv_vmax, self.icv_amax, self.icv_dmax = 300.0, 1000.0, 1000.0
        self.k_t = 0.0
        self.mem_icv_t5 = 0.0
        self.current_step = 0
        self.pump_ext = False
        self.pump_timer = TP(self.quirks.tp_pt_live)
        self.mc_goto_abs = False
        self.goto_xyz = (0.0, 0.0, 0.0)
        self._goto_moves = [MoveAbsolute(ax, GOTO_DEG_S, GOTO_ACC) for ax in self.axes]
        self._goto_done = False
        self._goto_prev = False
        self.mc_home_ext = False
        self._home: _Homing | None = None
        self.home_done = False
        self._limit_prev = False
        self._fk_xyz = (0.0, 0.0, 0.0)
        self._last_setpoint = [ax.setpoint for ax in self.axes]
        self._rung9_fk()
        self._rung21()

    def _emit(self, kind: str, **detail: Any) -> None:
        self.events.append(SimEvent(round(self.time_s, 4), kind, detail))
        self.event_counts[kind] += 1

    # ---- tag access (pylogix-style names) ---------------------------------------
    def write_tag(self, tag: str, value: Any) -> bool:
        m = _TAG_RE.match(tag)
        if not m or m.group(1) != "pc_package" or m.group(2) not in self.pc_package:
            return False
        member, idx = m.group(2), m.group(3)
        cur = self.pc_package[member]
        if isinstance(cur, list):
            if idx is None or not 0 <= int(idx) < len(cur):
                return False
            cur[int(idx)] = f32(value) if member in _REAL_FIELDS else int(value)
        else:
            if idx is not None:
                return False
            self.pc_package[member] = int(value)
        return True

    def read_tag(self, tag: str) -> tuple[bool, Any]:
        m = _TAG_RE.match(tag)
        if not m:
            return False, None
        pkg = self.pc_package if m.group(1) == "pc_package" else self.plc_package
        member, idx = m.group(2), m.group(3)
        if member not in pkg:
            return False, None
        cur = pkg[member]
        if isinstance(cur, list):
            if idx is None or not 0 <= int(idx) < len(cur):
                return False, None
            return True, cur[int(idx)]
        return (idx is None), (cur if idx is None else None)

    # ---- scan ---------------------------------------------------------------------
    def scan(self) -> None:
        self._rung1_limit_stop()
        self._rung4_dispatch()
        self._rungs5_8_homing()
        self._rung9_fk()
        self._rungs11_12_goto()
        self._rungs13_18_chain()
        self._rung19_pump()
        self.pump_out = self.pump_ext
        self.valve_out = not self.pump_ext
        self._rung21()
        for ax in self.axes:
            ax.step(SCAN_S)
        self._watch_setpoints()
        self.time_s += SCAN_S

    def run_for(self, seconds: float) -> None:
        for _ in range(int(round(seconds / SCAN_S))):
            self.scan()

    # ---- rungs --------------------------------------------------------------------
    def _rung1_limit_stop(self) -> None:
        active = any(self.home_switch) and not self.mc_home_ext
        if active:
            for ax in self.axes:
                ax.owner = "MC_Stop"
            self.mc_goto_abs = False
            self.icv_start_new_turn = False
            self.blend_done[0] = False
            self.blend_done[1] = False
            if not self._limit_prev:
                self._emit("limit_stop")
        self._limit_prev = active

    def _rung4_dispatch(self) -> None:
        pc, plc = self.pc_package, self.plc_package
        if pc["bit_doing"] == 0:
            return
        cmd = pc["commandID"]
        if cmd == 2:
            self.goto_xyz = (pc["argument_x"][0], pc["argument_y"][0], pc["argument_z"][0])
            self.mc_goto_abs = True
            plc["task_doing"], plc["task_state"] = 2, 2
            pc["commandID"] = -1
        elif cmd == 3:
            busy = [k for k, inst in enumerate(self.icv) if inst.state not in (0, 99)]
            if busy and self.patches.o2_reject_busy:
                plc["task_doing"], plc["task_state"] = 3, 4
                self._emit("cmd3_rejected", busy=busy)
            else:
                if any(k <= 4 for k in busy):
                    self._emit("cmd3_while_busy", busy=busy)
                self.icv_start_new_turn = True
                for i in range(N_POINTS):
                    self.icv_pos[i] = [pc["argument_x"][i], pc["argument_y"][i], pc["argument_z"][i]]
                    self.icv_e[i] = pc["argument_e"][i]
                plc["task_doing"], plc["task_state"] = 3, 2
            pc["commandID"] = -1
        elif cmd == 4:
            self.mc_home_ext = True
            plc["task_doing"], plc["task_state"] = 4, 2
            pc["commandID"] = -1
        elif cmd == 5:
            self.pump_ext = True  # overwritten by rung 19 in the same scan
            plc["task_doing"], plc["task_state"] = 5, 1
            pc["commandID"] = -1
        elif cmd == 6:
            self.pump_ext = False
            plc["task_doing"], plc["task_state"] = 6, 1
            pc["commandID"] = -1
        pc["bit_doing"] = 0

    def _rungs5_8_homing(self) -> None:
        if self.mc_on_btn_flag and self.mc_home_ext:
            if self._home is None:
                self._home = _Homing(self.axes)
            self.home_done = self._home.step()
        else:
            self._home = None
            self.home_done = False
        if self.home_done:  # Section1
            self.mc_home_ext = False
            if self.plc_package["task_doing"] == 4:
                self.plc_package["task_state"] = 1

    def _rung9_fk(self) -> None:
        err, xyz = calc_forward_kinematic(*(ax.actual for ax in self.axes),
                                          unassigned=self.quirks.unassigned_fun_output)
        if err:
            self._emit("fk_error")
        self._fk_xyz = xyz

    def _rungs11_12_goto(self) -> None:
        # Goto_Absolute FB: IK every scan into FB locals (Out_Angle1..3).
        _, th, _ = calc_inverse_kinematics(
            *self.goto_xyz, unassigned=self.quirks.unassigned_fun_output,
            limit_is_error=self.patches.o1_limit_is_error,
        )
        execute = self.mc_goto_abs
        for mv, ang in zip(self._goto_moves, th):
            mv(execute, ang)
        if execute and all(mv.done for mv in self._goto_moves):
            self._goto_done = True
        if self._goto_prev and not execute:
            self._goto_done = False
        self._goto_prev = execute
        if self._goto_done:  # Section2
            self.mc_goto_abs = False
            if self.plc_package["task_doing"] == 2:
                self.plc_package["task_state"] = 1

    def _rungs13_18_chain(self) -> None:
        p = self.icv_pos
        for k, inst in enumerate(self.icv):
            last = k == 5
            inp = ICVInputs(
                a=tuple(p[k]), b=tuple(p[k + 1]),
                c=(0.0, 0.0, 0.0) if last else tuple(p[k + 2]),
                v_max=self.icv_vmax, a_max=self.icv_amax, d_max=self.icv_dmax,
                v_start_req=0.0 if k == 0 else self.vend[k - 1],
                blend_mode=not last,
            )
            was = inst.state
            if k == 0:
                self.icv_start_new_turn = inst.call(self.icv_start_new_turn, inp)
            else:
                self.blend_done[k - 1] = inst.call(self.blend_done[k - 1], inp)
            if inst.state == 99 and was != 99:
                self._emit("ik_error", inst=k)
            if was == 1 and inst.state == 0:
                self._emit("zero_length_segment", inst=k)
            self.blend_done[k] = inst.out_done_blend
            self.vend[k] = inst.out_v_end
            self.icv_t[k] = inst.t_total_estimate
            if last:
                self.k_t = inst.t5_out

    def _rung19_pump(self) -> None:
        if self.k_t > 0.0:
            self.mem_icv_t5 = self.k_t
        if self.icv_start_new_turn:
            self.current_step = 0
        for k in range(6):
            if self.blend_done[k]:
                self.current_step = k + 1
        timer_in = self.current_step == 5
        self.pump_timer(timer_in, 0.5 * self.mem_icv_t5)
        step = self.current_step
        if step <= 4:
            self.pump_ext = self.icv_e[step] == 1
        elif step == 5:
            self.pump_ext = (self.icv_e[5] == 1) and self.pump_timer.q
        else:
            self.pump_ext = False

    def _rung21(self) -> None:
        plc = self.plc_package
        for i, ax in enumerate(self.axes):
            plc["pos_angular"][i] = f32(ax.actual)
        for i in range(3):
            plc["pos_EE"][i] = f32(self._fk_xyz[i])
        for i in range(N_POINTS):
            plc["Total_Time_Estimate"][i] = f32(self.icv_t[i])

    def _watch_setpoints(self) -> None:
        for i, ax in enumerate(self.axes):
            speed = abs(ax.setpoint - self._last_setpoint[i]) / SCAN_S
            if speed > self.max_joint_speed_deg_s:
                self._emit("setpoint_overspeed", axis=i, deg_s=round(speed, 1))
            self._last_setpoint[i] = ax.setpoint

    # ---- convenience --------------------------------------------------------------
    @property
    def pos_ee(self) -> tuple[float, float, float]:
        return tuple(self.plc_package["pos_EE"][:3])  # type: ignore[return-value]

    @property
    def chain_busy(self) -> bool:
        return any(inst.state not in (0, 99) for inst in self.icv)

    @property
    def chain_states(self) -> list[int]:
        return [inst.state for inst in self.icv]


class _Homing:
    """MC_Home_Delta + rung 6/7, compressed to its timing: search at 12°/s to each
    switch, then the calibration move back to 0 at 10°/s; Done must arrive inside
    the 5 s TP window opened when the last switch is reached (O10)."""

    def __init__(self, axes: list[ServoAxis]) -> None:
        self.axes = axes
        self.phase = "search"
        self.t_window = 0.0
        self.calib = [MoveAbsolute(ax, HOME_CALIB_DEG_S, HOME_CALIB_ACC) for ax in axes]

    def step(self) -> bool:
        if self.phase == "search":
            reached = 0
            for ax, sw in zip(self.axes, HOME_SWITCH_DEG):
                ax.owner = self
                delta = sw - ax.setpoint
                stepv = HOME_SEARCH_DEG_S * SCAN_S
                if abs(delta) <= stepv:
                    ax.command(sw)
                    reached += 1
                else:
                    ax.command(ax.setpoint + math.copysign(stepv, delta))
            if reached == 3:
                self.phase = "calib"
            return False
        if self.phase == "calib":
            self.t_window += SCAN_S
            if self.t_window > HOME_DONE_WINDOW_S:
                self.phase = "stuck"
                return False
            for mv in self.calib:
                mv(True, 0.0)
            if all(mv.done for mv in self.calib):
                self.phase = "done"
                return True
            return False
        return self.phase == "done"
