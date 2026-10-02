#!/usr/bin/env python3
"""Zero-dependency web dashboard and operator console for the Delta-robot cell.

The scheduler pushes structured events into a ``DashboardServer`` via a callback and
(for camera scenarios) registers the vision pipeline so the annotated frame can be
streamed as MJPEG. With an API handler attached (``set_api_handler``, done by
``modules.ui.supervisor`` in ``main.py --interface`` console mode) the page also offers
manual control, belt/rotation commands and scenario start/stop.

Endpoints
---------
* ``GET /``            — dashboard HTML (vanilla JS, no CDN).
* ``GET /events``      — Server-Sent Events stream of scheduler / console events.
* ``GET /stream.mjpg`` — ``multipart/x-mixed-replace`` MJPEG of the annotated frame.
* ``GET|POST /api/*``  — JSON commands, forwarded to the API handler (404 without one).

Only the Python standard library is used. Run standalone for a smoke test::

    python3 -m modules.ui.dashboard     # serves dummy events on :8000
"""
from __future__ import annotations

import base64
import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable

# A tiny 1x1 dark-gray JPEG, used as the MJPEG placeholder when no camera is
# attached (e.g. simulated scenarios). Stretched by the browser via CSS.
_PLACEHOLDER_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAP//////////////////////////////////"
    "////////////////////////////////////////////////////2wBDAf//////////"
    "////////////////////////////////////////////////////////////////////"
    "//////////wAARCAABAAEDASIAAhEBAxEB/8QAFAABAAAAAAAAAAAAAAAAAAAAAP/EABQQ"
    "AQAAAAAAAAAAAAAAAAAAAAD/xAAUAQEAAAAAAAAAAAAAAAAAAAAA/8QAFBEBAAAAAAAAAA"
    "AAAAAAAAAAAP/aAAwDAQACEQMRAD8AfwD/2Q=="
)

_MAX_BODY = 64 * 1024

_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Delta Robot Console</title>
<style>
  :root { color-scheme: dark; --bg:#15171c; --panel:#1b1f26; --line:#2c313a; --text:#e6e6e6;
          --muted:#8a93a3; --ok:#2dc653; --warn:#ffb703; --bad:#ff5a5f; --info:#6aa8ff; }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: ui-sans-serif, system-ui, "Segoe UI", Roboto, sans-serif;
         background:var(--bg); color:var(--text); font-size:14px; }
  .mono { font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace; }
  header { padding:8px 14px; background:#1e2229; border-bottom:1px solid var(--line);
           display:flex; align-items:center; gap:10px; flex-wrap:wrap; position:sticky; top:0; z-index:5; }
  header h1 { font-size:15px; margin:0; font-weight:650; letter-spacing:.3px; }
  .pill { display:inline-block; padding:2px 9px; border-radius:10px; font-size:12px; background:#2a2f38; color:#cfd6e2; }
  .pill.ok { background:#1f3320; color:#8aff9a; } .pill.bad { background:#3a2020; color:#ff8a8a; }
  .pill.idle { background:#1f3320; color:#8aff9a; } .pill.manual { background:#1d2c45; color:#9cc4ff; }
  .pill.running { background:#3a3016; color:#ffd166; } .pill.stopping { background:#3a2020; color:#ff8a8a; }
  .pill.sim { background:#35204a; color:#d9b3ff; }
  #stopbtn { margin-left:auto; font-weight:800; font-size:15px; letter-spacing:1px; padding:8px 22px;
             border-radius:8px; border:2px solid #ff8a8a; background:#b3261e; color:#fff; cursor:pointer; }
  #stopbtn:active { transform:translateY(1px); }
  .strip { display:flex; gap:18px; flex-wrap:wrap; padding:8px 14px; background:#181b21;
           border-bottom:1px solid var(--line); font-size:13px; }
  .strip b { color:var(--muted); font-weight:500; margin-right:4px; }
  nav { display:flex; gap:6px; padding:8px 14px 0; flex-wrap:wrap; }
  nav button { font:inherit; font-size:13px; padding:6px 14px; border-radius:6px 6px 0 0; cursor:pointer;
               background:#222730; color:#9aa4b2; border:1px solid var(--line); border-bottom:none; }
  nav button.active { background:var(--panel); color:#e8f0ff; }
  .tabpage { display:none; } .tabpage.active { display:block; }
  .wrap { display:grid; grid-template-columns: minmax(360px, 1.3fr) 1fr; gap:14px; padding:14px; }
  .wrap.one { grid-template-columns:1fr; }
  @media (max-width: 900px){ .wrap{ grid-template-columns:1fr; } }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:12px; }
  .card h2 { font-size:12px; text-transform:uppercase; letter-spacing:.6px; color:var(--muted); margin:0 0 10px; }
  .stack > .card + .card { margin-top:14px; }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; margin:6px 0; }
  label.f { display:flex; flex-direction:column; font-size:11px; color:var(--muted); gap:3px; }
  input, select { font:inherit; background:#11141a; color:var(--text); border:1px solid #394150; border-radius:6px;
                  padding:6px 8px; min-width:0; }
  input[type=number] { width:88px; }
  button.b { font:inherit; padding:7px 12px; border-radius:6px; border:1px solid #3d5170; background:#2d3a4d;
             color:#e3eeff; cursor:pointer; min-width:44px; }
  button.b:hover { background:#34465e; } button.b.pri { background:#1f6f43; border-color:#2dc653; }
  button.b.warn { background:#5a4413; border-color:var(--warn); } button.b.danger { background:#5a1f1f; border-color:var(--bad); }
  button:disabled, input:disabled, select:disabled { opacity:.38; cursor:not-allowed; }
  .jog { display:grid; grid-template-columns:repeat(3, 56px); gap:6px; }
  .jog button { height:44px; }
  .note { font-size:12px; color:var(--muted); line-height:1.45; }
  .kv { display:grid; grid-template-columns:auto 1fr; gap:4px 12px; font-size:13px; }
  .kv b { color:#9aa4b2; font-weight:500; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th,td { text-align:left; padding:4px 6px; border-bottom:1px solid #262b33; }
  th { color:var(--muted); font-weight:500; }
  canvas { width:100%; height:240px; display:block; background:#111419; border-radius:6px; }
  #cam { width:100%; border-radius:6px; background:#000; display:block; object-fit:contain; }
  #planlog, #teachjson { font-size:11px; max-height:220px; overflow:auto; white-space:pre-wrap; color:#b8c0cc; line-height:1.5; }
  .lvl-info { color:#b8c0cc; } .lvl-warn { color:var(--warn); } .lvl-error { color:var(--bad); }
  #toast { position:fixed; right:14px; bottom:14px; max-width:420px; padding:10px 14px; border-radius:8px;
           background:#23324a; border:1px solid #3d5170; display:none; z-index:10; }
  #toast.err { background:#3a2020; border-color:var(--bad); }
  .counters { display:grid; grid-template-columns:repeat(4, 1fr); gap:8px; }
  .counters div { background:#141821; border:1px solid var(--line); border-radius:6px; padding:6px; text-align:center; }
  .counters span { display:block; font-size:18px; font-weight:650; }
  .counters small { color:var(--muted); font-size:11px; }
  .counters.six { grid-template-columns:repeat(6, 1fr); }
  @media (max-width: 700px){ .counters.six{ grid-template-columns:repeat(3, 1fr); } }
  canvas#cellcanvas { height:540px; }
  @media (max-width: 900px){ canvas#cellcanvas{ height:380px; } }
  canvas#chart_scatter { height:320px; }
  .legend { display:flex; gap:14px; flex-wrap:wrap; margin-top:8px; }
  .legend i { display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; vertical-align:middle; }
  .o-picked { color:var(--ok); } .o-miss_grip { color:var(--bad); } .o-miss_late { color:var(--warn); }
  .o-lost_track, .o-on_belt { color:var(--muted); }
</style>
</head>
<body>
<header>
  <h1>DELTA ROBOT</h1>
  <span id="conn" class="pill bad">connecting…</span>
  <span id="mode" class="pill">—</span>
  <span id="scn" class="pill">—</span>
  <span id="simbadge" class="pill sim" style="display:none">SIMULATOR</span>
  <button id="stopbtn" title="Stop the running scenario and the belt">STOP</button>
</header>
<div class="strip mono">
  <span><b>PLC</b><span id="s_link">—</span></span>
  <span><b>arm</b><span id="s_pos">—</span></span>
  <span><b>arm state</b><span id="s_arm">—</span></span>
  <span><b>pump (cmd)</b><span id="s_pump">—</span></span>
  <span><b>belt</b><span id="s_belt">—</span></span>
  <span><b>cup</b><span id="s_rot">—</span></span>
</div>
<nav>
  <button id="tabbtn-operate" class="active" onclick="showTab('operate')">Operate</button>
  <button id="tabbtn-cell" onclick="showTab('cell')">Cell</button>
  <button id="tabbtn-flow" onclick="showTab('flow')">Throughput</button>
  <button id="tabbtn-live" onclick="showTab('live')">Live</button>
  <button id="tabbtn-charts" onclick="showTab('charts')">Charts (30s)</button>
  <button id="tabbtn-log" onclick="showTab('log')">Log</button>
</nav>

<div id="tab-operate" class="tabpage active">
<div id="noconsole" class="note" style="padding:14px 14px 0; display:none">
  Monitoring only: this run was started from the command line. Start <span class="mono">python3 main.py --interface</span>
  on its own for manual control and scenario switching.</div>
<div class="wrap">
  <div class="stack">
    <div class="card">
      <h2>Manual control</h2>
      <div class="note" id="manual_note">Available when no scenario is running and the arm is still.</div>
      <div class="row">
        <label class="f">X (mm)<input id="gx" type="number" step="1" class="man"></label>
        <label class="f">Y (mm)<input id="gy" type="number" step="1" class="man"></label>
        <label class="f">Z (mm)<input id="gz" type="number" step="1" class="man"></label>
        <button class="b pri man" onclick="gotoXYZ()">Go to</button>
        <button class="b man" onclick="fillHere()" title="Copy the current position into the fields">Use current</button>
      </div>
      <div class="row" style="align-items:flex-start; gap:18px; margin-top:12px">
        <div>
          <div class="note">Jog (XY)</div>
          <div class="jog">
            <span></span><button class="b man" onclick="jog('y',1)">Y+</button><span></span>
            <button class="b man" onclick="jog('x',-1)">X−</button><span></span><button class="b man" onclick="jog('x',1)">X+</button>
            <span></span><button class="b man" onclick="jog('y',-1)">Y−</button><span></span>
          </div>
        </div>
        <div>
          <div class="note">Jog (Z)</div>
          <div class="jog" style="grid-template-columns:56px">
            <button class="b man" onclick="jog('z',1)">Z↑</button>
            <button class="b man" onclick="jog('z',-1)">Z↓</button>
          </div>
        </div>
        <label class="f">step (mm)
          <select id="jogstep" class="man"><option>1</option><option selected>5</option><option>10</option><option>20</option></select>
        </label>
      </div>
      <div class="row" id="presets"></div>
      <div class="row">
        <button class="b pri man" onclick="pump(1)">Pump ON</button>
        <button class="b man" onclick="pump(0)">Pump OFF</button>
        <button class="b warn man" onclick="homeAxes()">Home axes…</button>
      </div>
      <div class="note">Moves use the PLC's joint-space goto (slow, curved path). Targets are checked against
        the PLC's own inverse kinematics before sending. The pump holds its state until the next trajectory.</div>
    </div>
    <div class="card">
      <h2>Conveyor &amp; cup</h2>
      <div class="row">
        <input id="beltslider" type="range" min="0" max="200" step="5" value="0" class="own" style="flex:1"
               oninput="$('beltval').textContent=this.value+' mm/s'">
        <span id="beltval" class="mono">0 mm/s</span>
        <button class="b own" onclick="belt(+$('beltslider').value)">Set</button>
        <button class="b danger own" onclick="belt(0)">Stop belt</button>
      </div>
      <div class="row">
        <label class="f">cup angle (°)<input id="rotdeg" type="number" step="5" value="0" class="own"></label>
        <button class="b own" onclick="rotate()">Rotate</button>
      </div>
    </div>
  </div>
  <div class="stack">
    <div class="card">
      <h2>Scenario</h2>
      <div class="row">
        <label class="f">scenario<select id="scnsel" onchange="scnChanged()"></select></label>
        <label class="f">duration (s, blank = until stopped)<input id="scndur" type="number" min="0" step="10" placeholder="∞"></label>
      </div>
      <div class="row">
        <label class="f">planner<select id="plsel" class="runopt" onchange="showDocs()"></select></label>
        <label class="f">speed law<select id="lawsel" class="runopt" onchange="showDocs()"></select></label>
        <label class="f">constant speed (mm/s)<input id="staticv" type="number" step="5" min="0" class="runopt"></label>
      </div>
      <div class="note" id="plandoc"></div>
      <div id="feedopts" style="display:none; margin-top:6px">
        <div class="note">Virtual feeder: parts are drawn from the seed and ride the real belt. The same seed gives the same arrivals.</div>
        <div class="row">
          <label class="f">feeder<select id="fdkind" class="runopt" onchange="feedKindChanged()"></select></label>
          <label class="f">seed<input id="fdseed" type="number" step="1" class="runopt"></label>
          <label class="f">rate (parts/min)<input id="fdrate" type="number" step="1" min="1" class="runopt"></label>
          <label class="f">parts (0 = no limit)<input id="fdmax" type="number" step="1" min="0" class="runopt"></label>
        </div>
      </div>
      <div class="row">
        <button class="b pri" id="startbtn" onclick="startScenario()">Start</button>
        <button class="b danger" id="stopscn" onclick="stopScenario()">Stop scenario</button>
      </div>
      <div class="kv mono" style="margin-top:8px">
        <b>running</b><span id="r_name">—</span>
        <b>elapsed</b><span id="r_elapsed">—</span>
        <b>plans</b><span id="r_plans">—</span>
      </div>
      <div class="note" style="margin-top:6px">Stop ends the run after the pick in flight. The belt is stopped when a run ends.
        Run settings apply to that run only; blank or unchanged fields keep the config value.</div>
    </div>
    <div class="card" id="simcard" style="display:none">
      <h2>Simulated cell</h2>
      <div class="counters mono">
        <div><span id="c_placed">0</span><small>placed</small></div>
        <div><span id="c_lost">0</span><small>lost</small></div>
        <div><span id="c_dropped">0</span><small>dropped</small></div>
        <div><span id="c_wrong">0</span><small>wrong bin</small></div>
      </div>
      <div class="note" id="c_events" style="margin-top:8px">PLC defect events: none</div>
    </div>
    <div class="card">
      <h2>Teach points</h2>
      <div class="row">
        <input id="teachname" placeholder="name, e.g. bin_QFP" style="flex:1">
        <button class="b" onclick="teach()">Save current position</button>
      </div>
      <table><tbody id="teachlist"></tbody></table>
      <div class="note" style="margin-top:6px">Saved in <span class="mono">config.yaml</span>
        › <span class="mono">interface.teach_points</span> (robot frame, mm).</div>
    </div>
  </div>
</div>
</div>

<div id="tab-cell" class="tabpage">
<div class="wrap one">
  <div class="card">
    <h2>Cell view — top, belt frame (belt runs left to right)</h2>
    <canvas id="cellcanvas"></canvas>
    <div class="legend note" id="cell_legend"></div>
    <div class="note" style="margin-top:6px">Parts are drawn where the tracker places them: camera fixes inside the camera window,
      then dead-reckoned on the belt encoder. Yellow outline = claimed by the planner; a part on the cup follows the arm.
      A cross marks a miss: red = grip failed, amber = not reached in time. The bar on the right is the cup height.</div>
  </div>
</div>
</div>

<div id="tab-flow" class="tabpage">
<div class="wrap one">
  <div class="card">
    <h2>Run record</h2>
    <div class="counters six mono">
      <div><span id="f_input">0</span><small>input (seen)</small></div>
      <div><span id="f_picked" class="o-picked">0</span><small>picked</small></div>
      <div><span id="f_grip" class="o-miss_grip">0</span><small>miss: grip failed</small></div>
      <div><span id="f_late" class="o-miss_late">0</span><small>miss: too late</small></div>
      <div><span id="f_lost">0</span><small>lost track</small></div>
      <div><span id="f_open">0</span><small>on belt</small></div>
    </div>
    <div class="kv mono" style="margin-top:10px">
      <b>run</b><span id="f_run">—</span>
      <b>input / throughput</b><span id="f_rates">—</span>
      <b>pick rate</b><span id="f_pickrate">—</span>
    </div>
  </div>
  <div class="card"><h2>Input and throughput (parts/min, trailing 60 s)</h2><canvas id="chart_rates"></canvas></div>
  <div class="card"><h2>Cumulative parts</h2><canvas id="chart_cum"></canvas></div>
  <div class="card"><h2>Throughput vs input — one point per 30 s of this run (parts/min)</h2><canvas id="chart_scatter"></canvas>
    <div class="note" style="margin-top:6px">Dashed line: throughput = input. Every run writes parts.csv, flow.csv and summary.json into
      its log folder; <span class="mono">python3 -m modules.tools.flow_report log/</span> plots all runs together.</div></div>
  <div class="card">
    <h2>Misses</h2>
    <table class="mono"><thead><tr><th>t (s)</th><th>part</th><th>type</th><th>outcome</th><th>reason</th><th>error (mm)</th></tr></thead>
    <tbody id="misslist"><tr><td colspan="6" style="color:#666">no misses</td></tr></tbody></table>
  </div>
  <div class="card"><h2>Last run summary</h2><div id="summary" class="mono note" style="white-space:pre-wrap">—</div></div>
</div>
</div>

<div id="tab-live" class="tabpage">
<div class="wrap">
  <div class="card">
    <h2>Camera (annotated overlay)</h2>
    <img id="cam" src="/stream.mjpg" alt="camera stream"/>
  </div>
  <div class="stack">
    <div class="card">
      <h2>Conveyor &amp; performance</h2>
      <div class="kv mono">
        <b>speed</b><span id="belt_speed">—</span>
        <b>vx, vy</b><span id="belt_v">—</span>
        <b>position</b><span id="belt_pos">—</span>
        <b>PLC round-trip</b><span id="perf_rtt">—</span>
        <b>pick cycle (avg)</b><span id="perf_cycle">—</span>
      </div>
    </div>
    <div class="card">
      <h2>Objects on belt (ROI → workspace)</h2>
      <table class="mono"><thead><tr><th>id</th><th>type</th><th>zone</th><th>u (mm)</th><th>x</th><th>y</th></tr></thead>
      <tbody id="objs"><tr><td colspan="6" style="color:#666">no detections yet</td></tr></tbody></table>
    </div>
    <div class="card">
      <h2>Plan log</h2>
      <div id="planlog" class="mono">—</div>
    </div>
  </div>
</div>
</div>

<div id="tab-charts" class="tabpage">
  <div class="wrap one">
    <div class="card"><h2>Conveyor speed — last 30s (mm/s)</h2><canvas id="chart_belt"></canvas></div>
    <div class="card"><h2>Object density — last 30s (count in workspace)</h2><canvas id="chart_density"></canvas></div>
    <div class="card"><h2>End-effector position — last 30s (mm)</h2><canvas id="chart_pose"></canvas>
      <div id="pose_note" class="note" style="margin-top:6px;"></div></div>
    <div class="card"><h2>End-effector horizontal speed (XY) — last 30s (mm/s)</h2><canvas id="chart_ee_xy_speed"></canvas></div>
    <div class="card"><h2>End-effector vertical speed (Z) — last 30s (mm/s)</h2><canvas id="chart_ee_z_speed"></canvas></div>
  </div>
</div>

<div id="tab-log" class="tabpage">
  <div class="wrap one">
    <div class="card">
      <h2>Console log</h2>
      <table class="mono"><thead><tr><th style="width:90px">time</th><th style="width:60px">level</th><th>message</th></tr></thead>
      <tbody id="loglist"><tr><td colspan="3" style="color:#666">no entries</td></tr></tbody></table>
    </div>
  </div>
</div>

<div id="toast"></div>
<script>
const $ = (id) => document.getElementById(id);
const fmt = (v, d=2) => (v===undefined||v===null||v==="") ? "—" : (typeof v==="number" ? v.toFixed(d) : v);
const planlines = [];
let sup = null;          // last console state ("sup" event); null = monitoring-only run
let live = false;        // SSE connected
let lastEvent = 0;       // watchdog
let pose = null;

// ---------------------------------------------------------------- tabs, toast
function showTab(name){
  for(const t of ["operate","cell","flow","live","charts","log"]){
    $("tab-"+t).classList.toggle("active", t===name);
    $("tabbtn-"+t).classList.toggle("active", t===name);
  }
}
let toastTimer = null;
function toast(text, err){
  const el=$("toast"); el.textContent=text; el.className=err?"err":""; el.style.display="block";
  clearTimeout(toastTimer); toastTimer=setTimeout(()=>el.style.display="none", err?6000:2500);
}

// ---------------------------------------------------------------- API
async function api(path, body){
  try{
    const r = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"},
                                 body: JSON.stringify(body||{})});
    const j = await r.json().catch(()=>({ok:false, error:"bad response"}));
    if(!j.ok){ toast(j.error || ("HTTP "+r.status), true); return null; }
    return j;
  }catch(e){ toast("request failed: "+e, true); return null; }
}
function num(id){ const v=$(id).value; if(v===""){ throw new Error(id+" is empty"); } return +v; }
async function gotoXYZ(){
  try{ const j = await api("/api/manual/goto", {x:num("gx"), y:num("gy"), z:num("gz")});
       if(j) toast("goto "+j.target.join(", ")); }catch(e){ toast(e.message, true); }
}
function fillHere(){ if(pose){ $("gx").value=pose[0].toFixed(1); $("gy").value=pose[1].toFixed(1); $("gz").value=pose[2].toFixed(1);} }
async function jog(axis, sign){ const j = await api("/api/manual/jog", {axis, step: sign*(+$("jogstep").value)}); if(j) toast("jog "+axis); }
async function preset(name){ const j = await api("/api/manual/preset", {name}); if(j) toast("goto "+name); }
async function pump(on){ const j = await api("/api/manual/pump", {on}); if(j) toast("pump "+(on?"ON":"OFF")); }
async function homeAxes(){
  if(!confirm("Home all three axes? Each arm moves to its home switch, then to the calibration pose.")) return;
  const j = await api("/api/manual/home", {confirm:true}); if(j) toast("homing started");
}
async function belt(speed){ const j = await api("/api/belt", {speed}); if(j) toast("belt "+speed+" mm/s"); }
async function rotate(){ const j = await api("/api/rotate", {deg:+$("rotdeg").value}); if(j) toast("cup rotating"); }
async function startScenario(){
  const name=$("scnsel").value, dur=$("scndur").value, settings=runSettings();
  const described = Object.keys(settings).map(k=>k+"="+settings[k]).join(", ");
  if(!confirm("Start scenario '"+name+"'"+(dur?(" for "+dur+" s"):" until stopped")
              +(described?("\\nwith "+described):"")+"?")) return;
  const j = await api("/api/scenario/start", {name, duration: dur===""?null:+dur, settings}); if(j) toast("started "+name);
}

// ---------------------------------------------------------------- run settings form
const esc = (t) => String(t??"").replace(/[&<>"']/g, c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
function fillRunForm(d){
  if($("plsel").dataset.done) return;
  const rd=d.run_defaults||{}, pl=d.plugins||{};
  $("plsel").innerHTML=(pl.planners||[]).map(p=>`<option value="${esc(p.name)}">${esc(p.name)} (${esc(p.kind)})</option>`).join("");
  $("lawsel").innerHTML=(pl.speed_laws||[]).map(p=>`<option value="${esc(p.name)}">${esc(p.name)}</option>`).join("");
  $("fdkind").innerHTML=(d.feeders||[]).map(k=>`<option>${esc(k)}</option>`).join("");
  $("plsel").value=rd.planner; $("lawsel").value=rd.speed_law; $("staticv").value=rd.static_mm_s;
  $("fdkind").value=rd.feeder_kind; $("fdseed").value=rd.feeder_seed; $("fdmax").value=rd.feeder_max_parts;
  $("plsel").dataset.done="1";
  feedKindChanged(); scnChanged(); showDocs();
}
function showDocs(){
  if(!sup) return;
  const pl=sup.plugins||{}, find=(list,n)=>(list||[]).find(p=>p.name===n)||{};
  const p=find(pl.planners,$("plsel").value), l=find(pl.speed_laws,$("lawsel").value);
  $("plandoc").textContent = (p.name?("planner: "+p.doc):"") + (l.name?("  ·  speed law: "+l.doc+" (gate "+l.kind+")"):"");
}
function scnChanged(){
  if(!sup) return;
  $("feedopts").style.display = (sup.scenario_feeds||{})[$("scnsel").value]==="virtual" ? "block" : "none";
}
let lastKind=null;
function feedKindChanged(keepValue){
  if(!sup) return;
  const rd=sup.run_defaults||{}, kind=$("fdkind").value, field=(rd.feeder_rate_fields||{})[kind];
  const box=$("fdrate");
  if(field===null || field===undefined){ box.disabled=true; box.value=""; box.placeholder="n/a"; }
  else { box.disabled=!(sup.mode==="idle"); box.placeholder="";
         if(!keepValue || lastKind!==kind || box.value==="") box.value=(rd.feeder_rates||{})[kind]??""; }
  lastKind=kind;
}
function runSettings(){
  const rd=(sup&&sup.run_defaults)||{}, out={};
  const put=(key, raw, def, numeric)=>{
    if(raw===""||raw===null||raw===undefined) return;
    const v = numeric ? +raw : raw;
    if(String(v)!==String(def)) out[key]=v;
  };
  put("planner", $("plsel").value, rd.planner);
  put("speed_law", $("lawsel").value, rd.speed_law);
  put("static_mm_s", $("staticv").value, rd.static_mm_s, true);
  if((sup.scenario_feeds||{})[$("scnsel").value]==="virtual"){
    const kind=$("fdkind").value;
    put("feeder_kind", kind, rd.feeder_kind);
    put("feeder_seed", $("fdseed").value, rd.feeder_seed, true);
    put("feeder_max_parts", $("fdmax").value, rd.feeder_max_parts, true);
    if(!$("fdrate").disabled) put("feeder_rate", $("fdrate").value, (rd.feeder_rates||{})[kind], true);
  }
  return out;
}
async function stopScenario(){ const j = await api("/api/scenario/stop", {}); if(j) toast(j.stopping?"stopping…":"nothing running"); }
$("stopbtn").onclick = async () => { const j = await api("/api/stop", {}); if(j) toast("STOP sent: belt stopped, run ending"); };

// ---------------------------------------------------------------- teach points (config.yaml, via the console)
let teachPoints = {}, teachKeys = [], teachShown = null;
function teachRender(points){
  const json = JSON.stringify(points||{});
  if(json===teachShown) return;
  teachShown = json; teachPoints = points||{}; teachKeys = Object.keys(teachPoints);
  const esc = t => t.replace(/&/g,"&amp;").replace(/</g,"&lt;");
  $("teachlist").innerHTML = teachKeys.map((k,i)=>`<tr><td>${esc(k)}</td><td class="mono">${teachPoints[k].map(v=>(+v).toFixed(2)).join(", ")}</td>`
    +`<td><button class="b man" onclick="teachGo(${i})">Go</button> <button class="b" onclick="teachDel(${i})">✕</button></td></tr>`).join("");
  applyEnable();
}
async function teach(){
  const name=$("teachname").value.trim(); if(!name){ toast("give the point a name", true); return; }
  const j = await api("/api/teach/save", {name}); if(j){ $("teachname").value=""; toast("saved "+name); }
}
async function teachDel(i){ const k=teachKeys[i]; const j = await api("/api/teach/delete", {name:k}); if(j) toast("deleted "+k); }
async function teachGo(i){ const k=teachKeys[i], p=teachPoints[k]; if(!p) return;
  const j = await api("/api/manual/goto", {x:p[0], y:p[1], z:p[2]}); if(j) toast("goto "+k); }
// Points saved by older pages in this browser's localStorage are moved to the config once.
async function teachMigrate(){
  let old = {};
  try{ old = JSON.parse(localStorage.getItem("delta_teach")||"{}"); }catch(_){ return; }
  const names = Object.keys(old).filter(k=>!(k in teachPoints));
  for(const k of names){
    const p = old[k];
    if(!await api("/api/teach/save", {name:k, x:p[0], y:p[1], z:p[2]})) return;
  }
  try{ localStorage.removeItem("delta_teach"); }catch(_){}
  if(names.length) toast("moved "+names.length+" browser teach point(s) to config.yaml");
}

// ---------------------------------------------------------------- enable/disable
function applyEnable(){
  const console_ = sup!==null;
  $("noconsole").style.display = console_ ? "none" : "block";
  const idle = console_ && live && sup.mode==="idle";
  const armReady = idle && sup.arm_idle && sup.link_ok;
  document.querySelectorAll(".man").forEach(el => el.disabled = !armReady);
  document.querySelectorAll(".own").forEach(el => el.disabled = !idle);
  $("startbtn").disabled = !armReady; $("scnsel").disabled = !idle; $("scndur").disabled = !idle;
  document.querySelectorAll(".runopt").forEach(el => el.disabled = !idle);
  if(idle && $("fdkind").value) feedKindChanged(true);
  $("stopscn").disabled = !(console_ && live && sup.mode==="running");
  $("stopbtn").disabled = !(console_ && live);
  $("manual_note").textContent = !console_ ? "Monitoring only." :
     !live ? "Disconnected — controls locked." :
     sup.mode!=="idle" ? ("Locked: console is "+sup.mode+".") :
     !sup.arm_idle ? ("Waiting: "+sup.arm_note+".") : "Ready.";
}

// ---------------------------------------------------------------- charts history
const WINDOW_S = 30;
const hist = [];
let hasPose = false;
function pushHistory(d){
  const now = Date.now()/1000;
  const prev = hist.length ? hist[hist.length-1] : null;
  let vxy = null, vz = null;
  if(prev && prev.x!==undefined && d.x!==undefined){
    const dt = now - prev.t;
    if(dt > 0){ vxy = Math.hypot(d.x-prev.x, d.y-prev.y)/dt; vz = (d.z-prev.z)/dt; }
  }
  hist.push({t: now, speed: (d.speed_mm_s!==undefined? d.speed_mm_s : null),
             density: (d.object_density!==undefined? d.object_density : null),
             x: d.x, y: d.y, z: d.z, vxy: vxy, vz: vz});
  if(d.x!==undefined) hasPose = true;
  const cutoff = now - WINDOW_S - 1;
  while(hist.length && hist[0].t < cutoff) hist.shift();
}

// ---------------------------------------------------------------- events
function logRow(e){
  const tb=$("loglist"); if(tb.dataset.empty!=="0"){ tb.innerHTML=""; tb.dataset.empty="0"; }
  const t=new Date((e.t||Date.now()/1000)*1000).toLocaleTimeString();
  const tr=document.createElement("tr");
  tr.innerHTML=`<td>${t}</td><td class="lvl-${e.level}">${e.level}</td><td class="lvl-${e.level}"></td>`;
  tr.lastChild.textContent = e.text;
  tb.prepend(tr); while(tb.children.length>300) tb.lastChild.remove();
}
function apply(type, d){
  lastEvent = Date.now();
  if(type==="status"){
    if(d.speed_mm_s!==undefined && d.speed_mm_s!==null){ $("belt_speed").textContent = fmt(d.speed_mm_s)+" mm/s"; $("s_belt").textContent = fmt(d.speed_mm_s,1)+" mm/s"; }
    $("belt_v").textContent = fmt(d.vx)+", "+fmt(d.vy);
    if(d.position_mm!==undefined) $("belt_pos").textContent = fmt(d.position_mm,1)+" mm";
    if(d.scenario) $("scn").textContent = d.scenario;
    if(d.round_trip_latency_s!==undefined) $("perf_rtt").textContent = (d.round_trip_latency_s*1000).toFixed(1)+" ms";
    if(d.pick_cycle_s!==undefined) $("perf_cycle").textContent = fmt(d.pick_cycle_s)+" s";
    if(d.x!==undefined){ pose=[d.x,d.y,d.z]; $("s_pos").textContent = pose.map(v=>v.toFixed(1)).join(", "); }
    if(d.rotate_deg!==undefined && d.rotate_deg!==null) $("s_rot").textContent = fmt(d.rotate_deg,1)+"°";
    if(d.arm) cell.arm=d.arm;
    if(d.e!==undefined && d.e!==null) cell.pump=d.e;
    if(d.speed_mm_s!==undefined && d.speed_mm_s!==null) cell.speed=+d.speed_mm_s;
    if(d.position_mm!==undefined && d.position_mm!==null){ cell.beltPos=+d.position_mm; cell.beltAt=performance.now(); }
    pushHistory(d);
  } else if(type==="sup"){
    sup = d;
    const m=$("mode"); m.textContent = d.mode.toUpperCase(); m.className = "pill "+d.mode;
    $("simbadge").style.display = d.sim ? "inline-block" : "none";
    $("simcard").style.display = d.sim ? "block" : "none";
    $("s_link").textContent = d.link_ok ? "connected" : "NO LINK";
    $("s_link").style.color = d.link_ok ? "" : "var(--bad)";
    $("s_arm").textContent = d.arm_idle ? "still" : d.arm_note;
    $("s_pump").textContent = d.pump_cmd===null ? "— (scenario)" : (d.pump_cmd ? "ON" : "off");
    $("r_name").textContent = d.scenario || "—";
    $("r_elapsed").textContent = d.elapsed_s!==null && d.elapsed_s!==undefined ? d.elapsed_s+" s" : "—";
    $("r_plans").textContent = d.scenario ? d.plans : "—";
    $("scn").textContent = d.scenario || "console";
    const sel=$("scnsel");
    if(sel.options.length!==d.scenarios.length){
      sel.innerHTML = d.scenarios.map(s=>`<option ${s==="production"?"selected":""}>${s}</option>`).join("");
    }
    fillRunForm(d);
    const firstTeach = teachShown===null;
    teachRender(d.teach_points);
    if(firstTeach) teachMigrate();
    if(!$("presets").dataset.done){
      $("presets").innerHTML = Object.keys(d.presets).map(k=>`<button class="b man" onclick="preset('${k}')">${k}</button>`).join("");
      $("presets").dataset.done="1";
      $("beltslider").max = d.limits.belt_max;
    }
    applyEnable();
  } else if(type==="sim"){
    const w=d.world||{};
    $("c_placed").textContent=w.placed??0; $("c_lost").textContent=w.lost??0;
    $("c_dropped").textContent=w.dropped??0; $("c_wrong").textContent=w.wrong_bin??0;
    const ev=d.plc_events||{}, keys=Object.keys(ev);
    $("c_events").textContent = "PLC defect events: " + (keys.length ? keys.map(k=>k+"="+ev[k]).join(", ") : "none");
    $("c_events").style.color = keys.length ? "var(--warn)" : "";
  } else if(type==="log"){
    logRow(d);
    if(d.level==="error") toast(d.text, true);
  } else if(type==="layout"){
    cell.layout=d;
    if(d.run) resetFlow(d.run);
  } else if(type==="flow"){
    pushFlow(d);
  } else if(type==="outcome"){
    onOutcome(d);
  } else if(type==="run_summary"){
    $("summary").textContent = JSON.stringify(d, null, 2);
  } else if(type==="detect"){
    cell.parts=d.objects||[]; cell.partsAt=performance.now();
    for(const o of cell.parts) if(o.v!==undefined) cell.lastUV[o.id]=[o.u, o.v];
    const rows=(d.objects||[]).slice().sort((a,b)=>(b.u??-1)-(a.u??-1));
    const zoneColor={ROI:"#3a86ff", transit:"#ffb703", workspace:"#2dc653", past:"#888", upstream:"#888"};
    $("objs").innerHTML = rows.length ? rows.map(o=>
      `<tr><td>${o.id}</td><td>${o.type||""}</td><td style="color:${zoneColor[o.zone]||'#ccc'}">${o.zone||"—"}</td>`
      +`<td>${fmt(o.u,1)}</td><td>${fmt(o.x,1)}</td><td>${fmt(o.y,1)}</td></tr>`).join("")
      : '<tr><td colspan="6" style="color:#666">no detections</td></tr>';
  } else if(type==="plan"){
    planlines.unshift("["+(d.plan_id??"?")+"] obj="+(d.object_id??"?")+" "+JSON.stringify(d.predicted_pick_position_2d||d));
    if(planlines.length>40) planlines.pop();
    $("planlog").textContent = planlines.join("\\n");
  } else if(type==="accept_phase"){
    planlines.unshift("[ACCEPT] cycle="+(d.cycle??"?")+" obj="+(d.object_id??"?")+" phase="+(d.phase??"?")
      +" wall_s="+fmt(d.wall_s)+" dist_mm="+fmt(d.distance_mm));
    if(planlines.length>40) planlines.pop();
    $("planlog").textContent = planlines.join("\\n");
  } else if(type==="accept_summary"){
    planlines.unshift("[ACCEPT-SUMMARY] "+JSON.stringify(d));
    if(planlines.length>40) planlines.pop();
    $("planlog").textContent = planlines.join("\\n");
  }
}

// ---------------------------------------------------------------- charts (canvas, no libs)
function drawChart(canvas, series, opts){
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 600, cssH = canvas.clientHeight || 240;
  if(canvas.width !== cssW*dpr || canvas.height !== cssH*dpr){ canvas.width = cssW*dpr; canvas.height = cssH*dpr; }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,cssW,cssH);
  const padL=46, padR=58, padT=10, padB=22, w = cssW-padL-padR, h = cssH-padT-padB;
  const now = Date.now()/1000, t0 = now-WINDOW_S;
  let lo=Infinity, hi=-Infinity, any=false;
  for(const s of series) for(const p of hist){
    const v = p[s.key];
    if(v===null||v===undefined||isNaN(v)||p.t<t0) continue;
    any=true; if(v<lo)lo=v; if(v>hi)hi=v;
  }
  ctx.font="11px ui-monospace, monospace";
  if(!any){ ctx.fillStyle="#5b6472"; ctx.fillText(opts.empty||"waiting for data…", padL, padT+h/2); return; }
  if(lo===hi){ lo-=1; hi+=1; }
  const span=hi-lo; lo-=span*0.08; hi+=span*0.08;
  const X = (t)=> padL + (t-t0)/WINDOW_S * w, Y = (v)=> padT + (1-(v-lo)/(hi-lo)) * h;
  ctx.strokeStyle="#262b33"; ctx.fillStyle="#7c8696"; ctx.lineWidth=1;
  for(let g=0; g<=4; g++){ const yy = padT + h*g/4, val = hi-(hi-lo)*g/4;
    ctx.beginPath(); ctx.moveTo(padL,yy); ctx.lineTo(padL+w,yy); ctx.stroke(); ctx.fillText(val.toFixed(0), 4, yy+3); }
  for(let s=0; s<=6; s++){ const xx = padL + w*s/6; ctx.fillText("-"+(WINDOW_S-WINDOW_S*s/6).toFixed(0)+"s", xx-10, padT+h+16); }
  for(const s of series){
    ctx.strokeStyle=s.color; ctx.lineWidth=1.6; ctx.beginPath();
    let started=false, lastV=null;
    for(const p of hist){
      if(p.t<t0) continue;
      const v=p[s.key];
      if(v===null||v===undefined||isNaN(v)){ started=false; continue; }
      const px=X(p.t), py=Y(v);
      if(!started){ ctx.moveTo(px,py); started=true; } else { ctx.lineTo(px,py); }
      lastV=v;
    }
    ctx.stroke();
    if(lastV!==null){ ctx.fillStyle=s.color; ctx.fillText(s.label+" "+lastV.toFixed(1), padL+w+4, Y(lastV)+3); }
  }
}
function redraw(){
  if($("tab-cell").classList.contains("active")) drawCell();
  if($("tab-flow").classList.contains("active")) drawFlow();
  if($("tab-charts").classList.contains("active")){
    drawChart($("chart_belt"), [{key:"speed", color:"#39FF14", label:"v"}], {empty:"waiting for belt speed…"});
    drawChart($("chart_density"), [{key:"density", color:"#c792ea", label:"N"}], {empty:"waiting for density…"});
    drawChart($("chart_pose"), [{key:"x",color:"#00F0FF",label:"X"},{key:"y",color:"#FFB000",label:"Y"},
               {key:"z",color:"#FF007F",label:"Z"}], {empty:"no pos_EE"});
    drawChart($("chart_ee_xy_speed"), [{key:"vxy", color:"#00F0FF", label:"v_xy"}], {empty:"no pos_EE"});
    drawChart($("chart_ee_z_speed"), [{key:"vz", color:"#FF007F", label:"v_z"}], {empty:"no pos_EE"});
    $("pose_note").textContent = hasPose ? "" :
      "End-effector pose comes from the PLC; camera-only runs without a PLC (--no-plc) have none.";
  }
  requestAnimationFrame(redraw);
}
requestAnimationFrame(redraw);

// ---------------------------------------------------------------- canvas helpers
function prepCanvas(canvas){
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 600, h = canvas.clientHeight || 240;
  if(canvas.width !== w*dpr || canvas.height !== h*dpr){ canvas.width = w*dpr; canvas.height = h*dpr; }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
  ctx.font="11px ui-monospace, monospace";
  return [ctx, w, h];
}
function drawXY(canvas, rows, xkey, series, opts){
  const [ctx, cssW, cssH] = prepCanvas(canvas);
  const padL=46, padR=90, padT=10, padB=22, w=cssW-padL-padR, h=cssH-padT-padB;
  if(rows.length<2){ ctx.fillStyle="#5b6472"; ctx.fillText(opts.empty||"waiting for data…", padL, padT+h/2); return; }
  const x0=rows[0][xkey]; let x1=rows[rows.length-1][xkey]; if(x1<=x0) x1=x0+1;
  let lo=0, hi=1;
  for(const s of series) for(const r of rows){ const v=r[s.key]; if(v!==null&&v!==undefined&&!isNaN(v)){ if(v>hi)hi=v; if(v<lo)lo=v; } }
  hi*=1.08;
  const X=(x)=>padL+(x-x0)/(x1-x0)*w, Y=(v)=>padT+(1-(v-lo)/(hi-lo))*h;
  ctx.strokeStyle="#262b33"; ctx.fillStyle="#7c8696"; ctx.lineWidth=1;
  for(let g=0; g<=4; g++){ const yy=padT+h*g/4, val=hi-(hi-lo)*g/4;
    ctx.beginPath(); ctx.moveTo(padL,yy); ctx.lineTo(padL+w,yy); ctx.stroke(); ctx.fillText(val.toFixed(val<10?1:0), 4, yy+3); }
  for(let k=0; k<=6; k++){ const xv=x0+(x1-x0)*k/6; ctx.fillText(xv.toFixed(0)+"s", X(xv)-10, padT+h+16); }
  series.forEach((s, i)=>{
    ctx.strokeStyle=s.color; ctx.lineWidth=1.8; ctx.beginPath();
    let started=false, last=null;
    for(const r of rows){ const v=r[s.key]; if(v===null||v===undefined||isNaN(v)){ started=false; continue; }
      const px=X(r[xkey]), py=Y(v); if(!started){ ctx.moveTo(px,py); started=true; } else ctx.lineTo(px,py); last=v; }
    ctx.stroke();
    ctx.fillStyle=s.color; ctx.fillText(s.label+" "+(last===null?"—":last.toFixed(1)), padL+w+6, padT+12+14*i);
  });
}
function drawScatter(canvas, pts){
  const [ctx, cssW, cssH] = prepCanvas(canvas);
  const pad=46, w=cssW-pad-20, h=cssH-pad-10;
  let hi=10; for(const [a,b] of pts){ hi=Math.max(hi,a,b); } hi*=1.1;
  const side=Math.min(w,h), X=(v)=>pad+v/hi*side, Y=(v)=>10+side-v/hi*side;
  ctx.strokeStyle="#262b33"; ctx.fillStyle="#7c8696"; ctx.lineWidth=1;
  for(let g=0; g<=4; g++){ const val=hi*g/4;
    ctx.beginPath(); ctx.moveTo(X(0),Y(val)); ctx.lineTo(X(hi),Y(val)); ctx.stroke(); ctx.fillText(val.toFixed(0), 8, Y(val)+3);
    ctx.beginPath(); ctx.moveTo(X(val),Y(0)); ctx.lineTo(X(val),Y(hi)); ctx.stroke(); ctx.fillText(val.toFixed(0), X(val)-6, Y(0)+16); }
  ctx.fillText("input →", X(hi)-50, Y(0)-6); ctx.fillText("throughput ↑", X(0)+6, Y(hi)+12);
  ctx.setLineDash([5,4]); ctx.strokeStyle="#8a93a3"; ctx.beginPath(); ctx.moveTo(X(0),Y(0)); ctx.lineTo(X(hi),Y(hi)); ctx.stroke(); ctx.setLineDash([]);
  if(!pts.length){ ctx.fillStyle="#5b6472"; ctx.fillText("first point after 30 s of a run", X(hi*0.3), Y(hi*0.5)); return; }
  pts.forEach(([a,b], i)=>{ ctx.fillStyle = i===pts.length-1 ? "#ffd166" : "#6aa8ff";
    ctx.beginPath(); ctx.arc(X(a),Y(b),4,0,Math.PI*2); ctx.fill(); });
}

// ---------------------------------------------------------------- throughput record
const SCATTER_WIN_S = 30;
let flowHist=[], scatterPts=[], lastWin=null, missRows=[];
function resetFlow(run){
  flowHist=[]; scatterPts=[]; lastWin=null; missRows=[]; renderMisses();
  $("summary").textContent="—";
  $("f_run").textContent = run.scenario+" · "+run.planner+" / "+run.speed_law
    + (run.feeder ? (" · feeder "+run.feeder.kind+" seed "+run.feeder.seed) : " · camera");
}
function pushFlow(d){
  if(flowHist.length && d.t < flowHist[flowHist.length-1].t - 1){ flowHist=[]; scatterPts=[]; lastWin=null; }
  flowHist.push(d); if(flowHist.length>7200) flowHist.shift();
  if(lastWin===null) lastWin=d;
  else if(d.t-lastWin.t >= SCATTER_WIN_S){ const dt=d.t-lastWin.t;
    scatterPts.push([60*(d.input-lastWin.input)/dt, 60*(d.picked-lastWin.picked)/dt]); lastWin=d; }
  $("f_input").textContent=d.input; $("f_picked").textContent=d.picked; $("f_grip").textContent=d.miss_grip;
  $("f_late").textContent=d.miss_late; $("f_lost").textContent=d.lost_track; $("f_open").textContent=d.open+(d.on_belt||0);
  $("f_rates").textContent = fmt(d.input_per_min,1)+" / "+fmt(d.throughput_per_min,1)+" parts/min (last "+fmt(d.rate_window_s,0)+" s)";
  const done=d.picked+d.miss_grip+d.miss_late;
  $("f_pickrate").textContent = done ? (100*d.picked/done).toFixed(1)+" %  ("+d.picked+" of "+done+" finished)" : "—";
}
function renderMisses(){
  $("misslist").innerHTML = missRows.length ? missRows.map(o=>
    `<tr><td>${fmt(o.t,1)}</td><td>${esc(o.id)}</td><td>${esc(o.type)}</td><td class="o-${esc(o.outcome)}">${esc(o.outcome)}</td>`
    +`<td>${esc(o.reason)}</td><td>${o.error_mm===null||o.error_mm===undefined?"—":fmt(o.error_mm,1)}</td></tr>`).join("")
    : '<tr><td colspan="6" style="color:#666">no misses</td></tr>';
}
function onOutcome(o){
  if(o.outcome==="picked" || o.outcome==="on_belt") return;
  missRows.unshift(o); if(missRows.length>100) missRows.pop(); renderMisses();
  const at=cell.lastUV[o.id];
  if(at && (o.outcome==="miss_grip" || o.outcome==="miss_late"))
    cell.marks.push({u:at[0], v:at[1], kind:o.outcome, born:performance.now()});
}
function drawFlow(){
  drawXY($("chart_rates"), flowHist, "t", [{key:"input_per_min", color:"#6aa8ff", label:"input"},
         {key:"throughput_per_min", color:"#2dc653", label:"picked"}], {empty:"no run yet"});
  drawXY($("chart_cum"), flowHist, "t", [{key:"input", color:"#6aa8ff", label:"input"},
         {key:"picked", color:"#2dc653", label:"picked"}, {key:"miss_grip", color:"#ff5a5f", label:"grip"},
         {key:"miss_late", color:"#ffb703", label:"late"}], {empty:"no run yet"});
  drawScatter($("chart_scatter"), scatterPts);
}

// ---------------------------------------------------------------- animated cell view
const cell = {layout:null, arm:null, pump:null, speed:0, beltPos:0, beltAt:0, parts:[], partsAt:0, lastUV:{}, marks:[]};
const TYPE_COLORS = ["#ff9f43", "#54a0ff", "#c792ea", "#1dd1a1", "#ff6b81"];
function typeColor(t){ const names=Object.keys((cell.layout||{}).sizes||{}); const i=names.indexOf(t);
  return TYPE_COLORS[(i<0?names.length:i) % TYPE_COLORS.length]; }
function drawCell(){
  const canvas=$("cellcanvas"); const [ctx, cssW, cssH] = prepCanvas(canvas);
  const L=cell.layout;
  if(!L){ ctx.fillStyle="#5b6472"; ctx.fillText("waiting for the cell layout…", 20, 30); return; }
  const cam=L.camera_window_uv, ws=L.workspace_window_uv, now=performance.now();
  const us=[cam[0], cam[1], ws[0], ws[1], L.base[0]], vs=[L.belt_v[0], L.belt_v[1], L.base[1]];
  for(const k in L.bins){ us.push(L.bins[k][0]); vs.push(L.bins[k][1]); }
  // Elbows swing up to one bicep length (140 mm) around their shoulders.
  for(const p of L.shoulders){ us.push(p[0]-150, p[0]+150); vs.push(p[1]-150, p[1]+150); }
  const u0=Math.min(...us)-20, u1=Math.max(...us)+20, v0=Math.min(...vs)-20, v1=Math.max(...vs)+20;
  const gaugeW=64, W=cssW-gaugeW, H=cssH;
  const sc=Math.min(W/(u1-u0), H/(v1-v0)), ox=(W-(u1-u0)*sc)/2, oy=(H-(v1-v0)*sc)/2;
  const P=(u,v)=>[ox+(u-u0)*sc, oy+(v1-v)*sc];
  // belt band with stripes moving at the belt position
  const [bx0,by0]=P(u0, L.belt_v[1]+8), [bx1,by1]=P(u1, L.belt_v[0]-8);
  ctx.fillStyle="#262a31"; ctx.fillRect(bx0,by0,bx1-bx0,by1-by0);
  const beltNow = cell.beltPos + cell.speed*Math.min((now-cell.beltAt)/1000, 0.5);
  const period=40, off=((beltNow % period)+period)%period;
  ctx.strokeStyle="#30353e"; ctx.lineWidth=2;
  for(let u=u0-period+off; u<u1; u+=period){ const [x]=P(u,0); ctx.beginPath(); ctx.moveTo(x,by0); ctx.lineTo(x,by1); ctx.stroke(); }
  // windows
  const box=(r, color, label)=>{ const [x0,y0]=P(r[0],r[3]), [x1,y1]=P(r[1],r[2]);
    ctx.strokeStyle=color; ctx.lineWidth=1.5; ctx.strokeRect(x0,y0,x1-x0,y1-y0); ctx.fillStyle=color; ctx.fillText(label, x0+4, y0-5); };
  box(cam, "#3a86ff", "camera"); box(ws, "#2dc653", "workspace");
  const [ex] = P(ws[1],0); ctx.setLineDash([6,4]); ctx.strokeStyle="#ff5a5f"; ctx.beginPath(); ctx.moveTo(ex,by0-10); ctx.lineTo(ex,by1+10); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle="#ff8a8a"; ctx.fillText("u_max", ex+3, by1+12);
  // bins
  for(const k in L.bins){ const [x,y]=P(...L.bins[k]), r=18*sc;
    ctx.strokeStyle=typeColor(k); ctx.lineWidth=2; ctx.strokeRect(x-r,y-r,2*r,2*r); ctx.fillStyle=typeColor(k); ctx.fillText("bin "+k, x-r, y-r-5); }
  // parts
  const dtp=Math.min((now-cell.partsAt)/1000, 0.5);
  for(const o of cell.parts){
    if(o.u===undefined || o.v===undefined) continue;
    let u=o.u+cell.speed*dtp, v=o.v;
    if(o.state==="held" && cell.arm){ u=cell.arm.c[0]; v=cell.arm.c[1]; }
    const size=(L.sizes||{})[o.type]||[14,14], w=(size[0]||14)*sc, h=(size[1]||14)*sc;
    const [x,y]=P(u,v);
    ctx.save(); ctx.translate(x,y); ctx.rotate(-(o.heading_uv_deg||0)*Math.PI/180);
    ctx.fillStyle=typeColor(o.type); ctx.globalAlpha = o.zone==="past" ? 0.35 : 0.9; ctx.fillRect(-w/2,-h/2,w,h); ctx.globalAlpha=1;
    ctx.lineWidth = o.state==="free" ? 1 : 2.5;
    ctx.strokeStyle = o.state==="claimed" ? "#ffd166" : o.state==="held" ? "#ffffff" : "#0b0d10";
    ctx.strokeRect(-w/2,-h/2,w,h); ctx.restore();
  }
  // misses
  cell.marks = cell.marks.filter(m=>now-m.born<2500);
  for(const m of cell.marks){ const [x,y]=P(m.u + (m.kind==="miss_late"?0:0), m.v), a=1-(now-m.born)/2500, r=9;
    ctx.globalAlpha=a; ctx.strokeStyle= m.kind==="miss_grip" ? "#ff5a5f" : "#ffb703"; ctx.lineWidth=3;
    ctx.beginPath(); ctx.moveTo(x-r,y-r); ctx.lineTo(x+r,y+r); ctx.moveTo(x+r,y-r); ctx.lineTo(x-r,y+r); ctx.stroke(); ctx.globalAlpha=1; }
  // arm: base plate, upper arms, forearms, effector, cup
  const sh=L.shoulders.map(p=>P(...p));
  ctx.fillStyle="rgba(150,160,180,0.10)"; ctx.strokeStyle="#5b6472"; ctx.lineWidth=1.5;
  ctx.beginPath(); sh.forEach(([x,y],i)=> i?ctx.lineTo(x,y):ctx.moveTo(x,y)); ctx.closePath(); ctx.fill(); ctx.stroke();
  const A=cell.arm;
  if(A){
    const wr=A.wrists.map(p=>P(...p)), c=P(A.c[0],A.c[1]);
    A.elbows.forEach((e,i)=>{ const [x,y]=P(e[0],e[1]);
      ctx.strokeStyle="#6aa8ff"; ctx.lineWidth=7; ctx.lineCap="round"; ctx.beginPath(); ctx.moveTo(...sh[i]); ctx.lineTo(x,y); ctx.stroke();
      if(wr[i]){ ctx.strokeStyle="#c9d1dd"; ctx.lineWidth=2.5; ctx.beginPath(); ctx.moveTo(x,y); ctx.lineTo(...wr[i]); ctx.stroke(); }
      ctx.fillStyle="#9cc4ff"; ctx.beginPath(); ctx.arc(x,y,4,0,Math.PI*2); ctx.fill(); });
    ctx.lineCap="butt";
    ctx.fillStyle="#aab3c2"; ctx.beginPath(); wr.forEach(([x,y],i)=> i?ctx.lineTo(x,y):ctx.moveTo(x,y)); ctx.closePath(); ctx.fill();
    ctx.fillStyle = cell.pump ? "#ff5a5f" : "#f0f0f0"; ctx.strokeStyle="#15171c"; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.arc(c[0],c[1],Math.max(4,6*sc),0,Math.PI*2); ctx.fill(); ctx.stroke();
  }
  // cup height gauge
  const Z=L.z, gx=cssW-gaugeW+22, gy0=20, gy1=cssH-30, gz=(z)=>gy0+(Z.max-z)/(Z.max-Z.min)*(gy1-gy0);
  ctx.fillStyle="#1e2229"; ctx.fillRect(gx,gy0,14,gy1-gy0); ctx.fillStyle="#7c8696"; ctx.fillText("z", gx+3, gy0-6);
  for(const [name,z,color] of [["pick",Z.pickup,"#2dc653"],["clr",Z.clearance,"#8a93a3"]]){
    ctx.strokeStyle=color; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(gx-4,gz(z)); ctx.lineTo(gx+18,gz(z)); ctx.stroke(); ctx.fillStyle=color; ctx.fillText(name, gx-20, gz(z)-3); }
  if(A){ const zz=Math.max(Z.min, Math.min(Z.max, A.c[2])); ctx.fillStyle = cell.pump ? "#ff5a5f" : "#f0f0f0"; ctx.fillRect(gx-2, gz(zz)-3, 18, 6);
    ctx.fillStyle="#cfd6e2"; ctx.fillText(A.c[2].toFixed(0), gx-4, gy1+16); }
  // caption
  ctx.fillStyle="#cfd6e2";
  const last=flowHist.length?flowHist[flowHist.length-1]:null;
  ctx.fillText("belt "+cell.speed.toFixed(1)+" mm/s"+(last?("   t "+last.t.toFixed(0)+" s   picked "+last.picked+"   miss "+(last.miss_grip+last.miss_late)):"")
               +(cell.pump?"   pump ON":""), 10, cssH-10);
  const names=Object.keys(L.sizes||{});
  const legend=names.map(n=>`<span><i style="background:${typeColor(n)}"></i>${esc(n)}</span>`).join("")
    +'<span><i style="border:2px solid #ffd166"></i>claimed</span><span><i style="background:#ff5a5f"></i>pump on</span>';
  if($("cell_legend").dataset.k!==names.join()){ $("cell_legend").innerHTML=legend; $("cell_legend").dataset.k=names.join(); }
}

// ---------------------------------------------------------------- connection + watchdog
function setLive(v){
  live=v; const c=$("conn"); c.textContent = v ? "live" : "disconnected"; c.className = "pill "+(v?"ok":"bad"); applyEnable();
}
function connect(){
  const es = new EventSource("/events");
  es.onopen = () => { lastEvent=Date.now(); setLive(true); };
  es.onerror = () => setLive(false);
  es.onmessage = (e) => { try{ const m=JSON.parse(e.data); apply(m.type, m.data||{}); if(!live) setLive(true);}catch(_){} };
}
// Console events arrive at >= 1 Hz; silence means the page no longer reflects the cell.
setInterval(()=>{ if(sup!==null && live && Date.now()-lastEvent > 4000) setLive(false); }, 1000);
applyEnable();
connect();
</script>
</body>
</html>"""


class DashboardServer:
    """In-process web dashboard fed by scheduler events + an optional camera.

    Thread model: the HTTP server runs in a daemon thread (``ThreadingHTTPServer``
    spawns one thread per request, so SSE/MJPEG long-poll handlers do not block
    each other). ``emit`` is called from the scheduler / console threads; it stores a
    per-type snapshot (so a freshly connected browser sees current state) and
    fans the event out to every live SSE subscriber queue.
    """

    def __init__(self, port: int = 8000, host: str = "0.0.0.0", *,
                 mjpeg_fps: float = 15.0) -> None:
        self.port = int(port)
        self.host = host
        self._mjpeg_period = 1.0 / max(1.0, float(mjpeg_fps))
        self._lock = threading.Lock()
        self._snapshot: dict[str, dict[str, Any]] = {}     # type -> last event dict
        self._subscribers: set["queue.Queue[str]"] = set()
        self._camera: Any = None                            # object with jpeg_frame()
        self._api: "Callable[[str, str, dict[str, Any]], tuple[int, dict[str, Any]]] | None" = None
        self._httpd: "ThreadingHTTPServer | None" = None
        self._thread: "threading.Thread | None" = None

    # -- producer side (called by scheduler / main) -----------------------------

    def emit(self, event_type: str, payload: dict[str, Any]) -> None:
        """Record an event and push it to all connected SSE clients."""
        message = json.dumps({"type": event_type, "data": payload}, ensure_ascii=True)
        with self._lock:
            if event_type != "log":  # log lines are a stream, not state
                self._snapshot[event_type] = payload
            subscribers = list(self._subscribers)
        for q in subscribers:
            try:
                q.put_nowait(message)
            except queue.Full:
                pass  # slow client — drop rather than block the scheduler

    def attach_camera(self, source: Any) -> None:
        """Register a vision pipeline exposing ``jpeg_frame() -> bytes | None``.

        Also enables the web overlay so the annotated frame is produced even when
        the native cv2 window is disabled. ``None`` detaches the camera.
        """
        with self._lock:
            self._camera = source
        enable = getattr(source, "enable_web_overlay", None)
        if callable(enable):
            try:
                enable()
            except Exception:
                pass

    def set_api_handler(
        self, handler: "Callable[[str, str, dict[str, Any]], tuple[int, dict[str, Any]]] | None"
    ) -> None:
        """Route ``GET|POST /api/*`` to ``handler(method, path, json_body) -> (status, json)``."""
        self._api = handler

    # -- lifecycle --------------------------------------------------------------

    def start(self) -> None:
        server = self  # capture for the handler closure

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_a):  # silence default stderr access log
                pass

            def do_GET(self):  # noqa: N802 (stdlib naming)
                if self.path.startswith("/api/"):
                    server._handle_api(self, "GET")
                elif self.path.startswith("/events"):
                    server._handle_events(self)
                elif self.path.startswith("/stream.mjpg"):
                    server._handle_mjpeg(self)
                elif self.path in ("/", "/index.html"):
                    server._send_html(self, _DASHBOARD_HTML)
                else:
                    self.send_error(404, "Not Found")

            def do_POST(self):  # noqa: N802 (stdlib naming)
                if self.path.startswith("/api/"):
                    server._handle_api(self, "POST")
                else:
                    self.send_error(404, "Not Found")

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="DashboardServer", daemon=True)
        self._thread.start()
        print(f"[INTERFACE] Dashboard at http://localhost:{self.port}  "
              f"(bind {self.host}:{self.port})")

    def stop(self) -> None:
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    # -- request handlers (run on per-request threads) --------------------------

    @staticmethod
    def _send_html(handler: BaseHTTPRequestHandler, html: str) -> None:
        body = html.encode("utf-8")
        handler.send_response(200)
        handler.send_header("Content-Type", "text/html; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    @staticmethod
    def _send_json(handler: BaseHTTPRequestHandler, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        handler.send_response(status)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def _handle_api(self, handler: BaseHTTPRequestHandler, method: str) -> None:
        api = self._api
        if api is None:
            self._send_json(handler, 404, {"ok": False, "error": "no operator console in this run"})
            return
        body: dict[str, Any] = {}
        if method == "POST":
            length = int(handler.headers.get("Content-Length") or 0)
            if length > _MAX_BODY:
                self._send_json(handler, 413, {"ok": False, "error": "request too large"})
                return
            raw = handler.rfile.read(length) if length else b""
            try:
                body = json.loads(raw.decode("utf-8")) if raw else {}
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json(handler, 400, {"ok": False, "error": "body must be JSON"})
                return
            if not isinstance(body, dict):
                self._send_json(handler, 400, {"ok": False, "error": "body must be a JSON object"})
                return
        try:
            status, payload = api(method, handler.path, body)
        except Exception as exc:  # the handler reports its own errors; this is a last resort
            status, payload = 500, {"ok": False, "error": str(exc)}
        self._send_json(handler, status, payload)

    def _handle_events(self, handler: BaseHTTPRequestHandler) -> None:
        q: "queue.Queue[str]" = queue.Queue(maxsize=256)
        with self._lock:
            self._subscribers.add(q)
            snapshot = [json.dumps({"type": t, "data": d}, ensure_ascii=True)
                        for t, d in self._snapshot.items()]
        try:
            handler.send_response(200)
            handler.send_header("Content-Type", "text/event-stream")
            handler.send_header("Cache-Control", "no-cache")
            handler.send_header("Connection", "keep-alive")
            handler.end_headers()
            # Replay current state so a fresh browser is populated immediately.
            for msg in snapshot:
                handler.wfile.write(f"data: {msg}\n\n".encode("utf-8"))
            handler.wfile.flush()
            while True:
                try:
                    msg = q.get(timeout=10.0)
                    handler.wfile.write(f"data: {msg}\n\n".encode("utf-8"))
                except queue.Empty:
                    handler.wfile.write(b": keep-alive\n\n")  # SSE comment heartbeat
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # browser closed the tab
        finally:
            with self._lock:
                self._subscribers.discard(q)

    def _handle_mjpeg(self, handler: BaseHTTPRequestHandler) -> None:
        boundary = "frame"
        try:
            handler.send_response(200)
            handler.send_header("Age", "0")
            handler.send_header("Cache-Control", "no-cache, private")
            handler.send_header("Pragma", "no-cache")
            handler.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={boundary}",
            )
            handler.end_headers()
            while True:
                with self._lock:
                    cam = self._camera
                frame = None
                if cam is not None:
                    try:
                        frame = cam.jpeg_frame()
                    except Exception:
                        frame = None
                if not frame:
                    frame = _PLACEHOLDER_JPEG
                handler.wfile.write(
                    f"--{boundary}\r\n".encode("ascii")
                    + b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    + frame
                    + b"\r\n"
                )
                handler.wfile.flush()
                time.sleep(self._mjpeg_period)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # browser closed the stream


def _demo() -> None:
    """Standalone smoke test: serve synthetic events with no hardware."""
    import math

    server = DashboardServer(port=8000)
    server.start()
    print("[INTERFACE] Demo running — open http://localhost:8000  (Ctrl-C to stop)")
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            server.emit("status", {"scenario": "demo",
                                   "speed_mm_s": round(120.0 + 30.0 * math.sin(t), 1),
                                   "vx": 120.0, "vy": 0.0,
                                   "position_mm": round(120.0 * t, 1),
                                   "object_density": 2 + int(math.sin(t / 2.0) > 0),
                                   "round_trip_latency_s": round(0.01 + 0.003 * math.sin(t * 3), 4),
                                   "pick_cycle_s": round(2.0 + 0.2 * math.sin(t / 4.0), 2),
                                   "x": round(-100 + 80 * math.sin(t), 1),
                                   "y": round(60 * math.cos(t), 1),
                                   "z": round(-300 + 20 * math.sin(2 * t), 1),
                                   "e": 1 if math.sin(2 * t) > 0 else 0})
            u_demo = (t * 40.0) % 380.0
            zone_demo = ("ROI" if u_demo <= 120 else "transit" if u_demo < 188
                         else "workspace" if u_demo <= 363 else "past")
            server.emit("detect", {"t": round(t, 2), "objects": [
                {"id": "yolo-1", "type": "QFP", "u": round(u_demo, 1), "zone": zone_demo,
                 "x": round(450 + 30 * math.sin(t), 1), "y": round(20 * math.cos(t), 1)},
            ]})
            if int(t) % 3 == 0:
                server.emit("plan", {"plan_id": int(t), "object_id": "yolo-1",
                                     "predicted_pick_position_2d": [540.0, 0.0, -310.0]})
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[INTERFACE] Demo stopped.")
    finally:
        server.stop()


if __name__ == "__main__":
    _demo()
