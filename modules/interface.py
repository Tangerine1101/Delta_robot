#!/usr/bin/env python3
"""Zero-dependency web dashboard for live Delta-robot test runs.

This module replaces the old ``run_test.py`` (subprocess + matplotlib) with an
**in-process** web interface. The scheduler pushes structured events into a
``DashboardServer`` via a callback and (for camera scenarios) registers the
vision pipeline so the annotated frame can be streamed as MJPEG. The browser is
out-of-process, so there is no GUI-thread contention with the OpenCV/Qt window
that broke the old design — and the native cv2 window can be turned off entirely.

Endpoints
---------
* ``GET /``            — dashboard HTML (vanilla JS, no CDN).
* ``GET /events``      — Server-Sent Events stream of scheduler events.
* ``GET /stream.mjpg`` — ``multipart/x-mixed-replace`` MJPEG of the annotated frame.

Only the Python standard library is used. Run standalone for a smoke test::

    python3 -m modules.interface        # serves dummy events on :8000
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


_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Delta Robot — Live</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
         background:#15171c; color:#e6e6e6; }
  header { padding:10px 16px; background:#1e2229; border-bottom:1px solid #2c313a;
           display:flex; align-items:center; gap:14px; }
  header h1 { font-size:15px; margin:0; font-weight:600; letter-spacing:.3px; }
  #conn { font-size:12px; padding:2px 8px; border-radius:10px; background:#3a2020; color:#ff8a8a; }
  #conn.ok { background:#1f3320; color:#8aff9a; }
  nav { margin-left:auto; display:flex; gap:6px; }
  nav button { font:inherit; font-size:12px; padding:4px 12px; border-radius:6px; cursor:pointer;
               background:#222730; color:#9aa4b2; border:1px solid #2c313a; }
  nav button.active { background:#2d3a4d; color:#cfe3ff; border-color:#3d5170; }
  .tabpage { display:none; }
  .tabpage.active { display:block; }
  canvas { width:100%; height:260px; display:block; background:#111419; border-radius:6px; }
  .wrap { display:grid; grid-template-columns: minmax(420px, 1.4fr) 1fr; gap:14px; padding:14px; }
  @media (max-width: 900px){ .wrap{ grid-template-columns:1fr; } }
  .card { background:#1b1f26; border:1px solid #2c313a; border-radius:8px; padding:12px; }
  .card h2 { font-size:12px; text-transform:uppercase; letter-spacing:.6px; color:#8a93a3;
             margin:0 0 8px; }
  #cam { width:100%; border-radius:6px; background:#000; display:block; aspect-ratio:16/9;
         object-fit:contain; }
  .kv { display:grid; grid-template-columns:auto 1fr; gap:4px 12px; font-size:13px; }
  .kv b { color:#9aa4b2; font-weight:500; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th,td { text-align:left; padding:4px 6px; border-bottom:1px solid #262b33; }
  th { color:#8a93a3; font-weight:500; }
  #planlog { font-size:11px; max-height:220px; overflow:auto; white-space:pre-wrap;
             color:#b8c0cc; line-height:1.5; }
  .pill { display:inline-block; padding:1px 7px; border-radius:9px; background:#243; color:#9f9; }
  .grid2 { display:grid; grid-template-columns:1fr 1fr; gap:14px; }
  .section-h { font-size:13px; font-weight:600; color:#cfe3ff; letter-spacing:.3px;
               margin:18px 0 6px; padding-bottom:4px; border-bottom:1px solid #2c313a; }
  .section-h:first-child { margin-top:0; }
</style>
</head>
<body>
<header>
  <h1>DELTA ROBOT · LIVE TEST</h1>
  <span id="conn">connecting…</span>
  <span id="scn" class="pill" style="background:#223; color:#9bf;">—</span>
  <nav>
    <button id="tabbtn-live" class="active" onclick="showTab('live')">Live</button>
    <button id="tabbtn-charts" onclick="showTab('charts')">Charts (30s)</button>
  </nav>
</header>
<div id="tab-charts" class="tabpage">
  <div class="wrap" style="grid-template-columns:1fr;">
    <div class="section-h">Conveyor</div>
    <div class="card">
      <h2>Conveyor speed — last 30s (mm/s)</h2>
      <canvas id="chart_belt"></canvas>
    </div>
    <div class="card">
      <h2>Object density — last 30s (count in workspace)</h2>
      <canvas id="chart_density"></canvas>
    </div>

    <div class="section-h">End effector</div>
    <div class="card">
      <h2>End-effector position — last 30s (mm)</h2>
      <canvas id="chart_pose"></canvas>
      <div id="pose_note" style="font-size:11px; color:#8a93a3; margin-top:6px;"></div>
    </div>
    <div class="card">
      <h2>End-effector horizontal speed (XY) — last 30s (mm/s)</h2>
      <canvas id="chart_ee_xy_speed"></canvas>
    </div>
    <div class="card">
      <h2>End-effector vertical speed (Z) — last 30s (mm/s)</h2>
      <canvas id="chart_ee_z_speed"></canvas>
    </div>
  </div>
</div>
<div id="tab-live" class="tabpage active">
<div class="wrap">
  <div class="card">
    <h2>Camera (annotated overlay)</h2>
    <img id="cam" src="/stream.mjpg" alt="camera stream"/>
  </div>
  <div>
    <div class="grid2">
      <div class="card">
        <h2>Conveyor</h2>
        <div class="kv">
          <b>speed</b><span id="belt_speed">—</span>
          <b>vx, vy</b><span id="belt_v">—</span>
          <b>position</b><span id="belt_pos">—</span>
        </div>
      </div>
      <div class="card">
        <h2>Performance</h2>
        <div class="kv">
          <b>PLC round-trip</b><span id="perf_rtt">—</span>
          <b>pick cycle (avg)</b><span id="perf_cycle">—</span>
        </div>
      </div>
    </div>
    <div class="card" style="margin-top:14px;">
      <h2>Objects on belt (ROI → workspace)</h2>
      <table><thead><tr><th>id</th><th>type</th><th>zone</th><th>u (mm)</th><th>x</th><th>y</th></tr></thead>
      <tbody id="objs"><tr><td colspan="6" style="color:#666">no detections yet</td></tr></tbody></table>
    </div>
    <div class="card" style="margin-top:14px;">
      <h2>Plan log</h2>
      <div id="planlog">—</div>
    </div>
  </div>
</div>
</div>
<script>
const $ = (id) => document.getElementById(id);
const fmt = (v) => (v===undefined||v===null) ? "—" : (typeof v==="number" ? v.toFixed(2) : v);
const planlines = [];

// Rolling 30s history of status samples for the Charts tab.
const WINDOW_S = 30;
const hist = [];   // {t, speed, density, x, y, z, vxy, vz}
let hasPose = false;

function pushHistory(d){
  const now = Date.now()/1000;
  const prev = hist.length ? hist[hist.length-1] : null;
  // End-effector velocity: finite-difference of consecutive pos_EE samples
  // already in hist — no backend instrumentation needed for these two charts.
  let vxy = null, vz = null;
  if(prev && prev.x!==undefined && d.x!==undefined){
    const dt = now - prev.t;
    if(dt > 0){
      vxy = Math.hypot(d.x-prev.x, d.y-prev.y)/dt;
      vz = (d.z-prev.z)/dt;
    }
  }
  const s = {t: now, speed: (d.speed_mm_s!==undefined? d.speed_mm_s : null),
             density: (d.object_density!==undefined? d.object_density : null),
             x: d.x, y: d.y, z: d.z, vxy: vxy, vz: vz};
  if(d.x!==undefined) hasPose = true;
  hist.push(s);
  const cutoff = now - WINDOW_S - 1;
  while(hist.length && hist[0].t < cutoff) hist.shift();
}

function showTab(name){
  for(const t of ["live","charts"]){
    $("tab-"+t).classList.toggle("active", t===name);
    $("tabbtn-"+t).classList.toggle("active", t===name);
  }
}

function apply(type, d){
  if(type==="status"){
    if(d.speed_mm_s!==undefined) $("belt_speed").textContent = fmt(d.speed_mm_s)+" mm/s";
    $("belt_v").textContent = fmt(d.vx)+", "+fmt(d.vy);
    if(d.position_mm!==undefined) $("belt_pos").textContent = fmt(d.position_mm)+" mm";
    if(d.scenario) $("scn").textContent = d.scenario;
    if(d.round_trip_latency_s!==undefined) $("perf_rtt").textContent = (d.round_trip_latency_s*1000).toFixed(1)+" ms";
    if(d.pick_cycle_s!==undefined) $("perf_cycle").textContent = fmt(d.pick_cycle_s)+" s";
    pushHistory(d);
  } else if(type==="detect"){
    const tb=$("objs");
    // Every object on the belt, furthest along (nearest the workspace exit) first.
    const rows=(d.objects||[]).slice().sort((a,b)=>(b.u??-1)-(a.u??-1));
    const zoneColor={ROI:"#3a86ff", transit:"#ffb703", workspace:"#2dc653", past:"#888", upstream:"#888"};
    tb.innerHTML = rows.length ? rows.map(o=>
      `<tr><td>${o.id}</td><td>${o.type||""}</td>`
      +`<td style="color:${zoneColor[o.zone]||'#ccc'}">${o.zone||"—"}</td>`
      +`<td>${fmt(o.u)}</td><td>${fmt(o.x)}</td><td>${fmt(o.y)}</td></tr>`).join("")
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
// --- Minimal canvas line charts (no external libs) -------------------------
function drawChart(canvas, series, opts){
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 600, cssH = canvas.clientHeight || 260;
  if(canvas.width !== cssW*dpr || canvas.height !== cssH*dpr){
    canvas.width = cssW*dpr; canvas.height = cssH*dpr;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,cssW,cssH);
  const padL=46, padR=58, padT=10, padB=22;
  const w = cssW-padL-padR, h = cssH-padT-padB;
  const now = Date.now()/1000, t0 = now-WINDOW_S;

  // y-range across all series (with padding); skip if no numeric data.
  let lo=Infinity, hi=-Infinity, any=false;
  for(const s of series) for(const p of hist){
    const v = p[s.key];
    if(v===null||v===undefined||isNaN(v)) continue;
    if(p.t<t0) continue;
    any=true; if(v<lo)lo=v; if(v>hi)hi=v;
  }
  ctx.font="11px ui-monospace, monospace";
  if(!any){ ctx.fillStyle="#5b6472"; ctx.fillText(opts.empty||"waiting for data…", padL, padT+h/2); return; }
  if(lo===hi){ lo-=1; hi+=1; }
  const span=hi-lo; lo-=span*0.08; hi+=span*0.08;
  const X = (t)=> padL + (t-t0)/WINDOW_S * w;
  const Y = (v)=> padT + (1-(v-lo)/(hi-lo)) * h;

  // grid + axes
  ctx.strokeStyle="#262b33"; ctx.fillStyle="#7c8696"; ctx.lineWidth=1;
  for(let g=0; g<=4; g++){
    const yy = padT + h*g/4, val = hi-(hi-lo)*g/4;
    ctx.beginPath(); ctx.moveTo(padL,yy); ctx.lineTo(padL+w,yy); ctx.stroke();
    ctx.fillText(val.toFixed(0), 4, yy+3);
  }
  for(let s=0; s<=6; s++){
    const xx = padL + w*s/6;
    ctx.fillText("-"+(WINDOW_S-WINDOW_S*s/6).toFixed(0)+"s", xx-10, padT+h+16);
  }

  // series polylines + right-edge legend value
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
    if(lastV!==null){
      ctx.fillStyle=s.color;
      ctx.fillText(s.label+" "+lastV.toFixed(1), padL+w+4, Y(lastV)+3);
    }
  }
}

function redraw(){
  if($("tab-charts").classList.contains("active")){
    drawChart($("chart_belt"), [{key:"speed", color:"#39FF14", label:"v"}],
              {empty:"waiting for belt speed…"});
    drawChart($("chart_density"), [{key:"density", color:"#c792ea", label:"N"}],
              {empty:"waiting for density…"});
    drawChart($("chart_pose"),
              [{key:"x",color:"#00F0FF",label:"X"},{key:"y",color:"#FFB000",label:"Y"},
               {key:"z",color:"#FF007F",label:"Z"}],
              {empty:"no pos_EE (simulated run has no robot pose)"});
    drawChart($("chart_ee_xy_speed"), [{key:"vxy", color:"#00F0FF", label:"v_xy"}],
              {empty:"no pos_EE (simulated run has no robot pose)"});
    drawChart($("chart_ee_z_speed"), [{key:"vz", color:"#FF007F", label:"v_z"}],
              {empty:"no pos_EE (simulated run has no robot pose)"});
    $("pose_note").textContent = hasPose ? "" :
      "End-effector pose is only available on live-PLC scenarios (test_vision_only / test_accuracy / test_acceptance / production), not with --simulate-executor.";
  }
  requestAnimationFrame(redraw);
}
requestAnimationFrame(redraw);

function connect(){
  const es = new EventSource("/events");
  es.onopen = () => { $("conn").textContent="live"; $("conn").className="ok"; };
  es.onerror = () => { $("conn").textContent="reconnecting…"; $("conn").className=""; };
  es.onmessage = (e) => { try{ const m=JSON.parse(e.data); apply(m.type, m.data||{}); }catch(_){} };
}
connect();
</script>
</body>
</html>"""


class DashboardServer:
    """In-process web dashboard fed by scheduler events + an optional camera.

    Thread model: the HTTP server runs in a daemon thread (``ThreadingHTTPServer``
    spawns one thread per request, so SSE/MJPEG long-poll handlers do not block
    each other). ``emit`` is called from the scheduler's main loop; it stores a
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
        self._httpd: "ThreadingHTTPServer | None" = None
        self._thread: "threading.Thread | None" = None

    # -- producer side (called by scheduler / main) -----------------------------

    def emit(self, event_type: str, payload: dict[str, Any]) -> None:
        """Record an event and push it to all connected SSE clients."""
        message = json.dumps({"type": event_type, "data": payload}, ensure_ascii=True)
        with self._lock:
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
        the native cv2 window is disabled.
        """
        with self._lock:
            self._camera = source
        enable = getattr(source, "enable_web_overlay", None)
        if callable(enable):
            try:
                enable()
            except Exception:
                pass

    # -- lifecycle --------------------------------------------------------------

    def start(self) -> None:
        server = self  # capture for the handler closure

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_a):  # silence default stderr access log
                pass

            def do_GET(self):  # noqa: N802 (stdlib naming)
                if self.path.startswith("/events"):
                    server._handle_events(self)
                elif self.path.startswith("/stream.mjpg"):
                    server._handle_mjpeg(self)
                elif self.path in ("/", "/index.html"):
                    server._send_html(self, _DASHBOARD_HTML)
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
            # Sweep a fake object's belt position u across the ROI→workspace span
            # so the demo exercises the new zone/u columns.
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
