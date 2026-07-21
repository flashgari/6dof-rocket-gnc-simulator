#!/usr/bin/env python3
"""Build a standalone HTML animation from simulation CSV outputs."""

from __future__ import annotations

import csv
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT = PROJECT_ROOT / "outputs"


def load_series(path: Path, label: str, color: str, stride: int = 3) -> dict:
    with path.open() as f:
        rows = list(csv.DictReader(f))
    selected = rows[::stride]
    if selected[-1] is not rows[-1]:
        selected.append(rows[-1])

    samples = []
    for row in selected:
        samples.append(
            {
                "t": round(float(row["time_s"]), 4),
                "x": round(float(row["x_m"]), 4),
                "y": round(float(row["y_m"]), 4),
                "z": round(float(row["z_m"]), 4),
                "pitch": round(float(row["unwrapped_pitch_deg"]), 3),
                "bzz": round(float(row["body_z_z"]), 5),
                "bzx": round(float(row["body_z_x"]), 5),
                "lateral": round(float(row["lateral_m"]), 4),
                "tilt": round(float(row["tilt_deg"]), 3),
                "gimbal": round(float(row.get("gimbal_total_deg", "0") or 0.0), 4),
                "gimbal_x": round(float(row.get("gimbal_x_deg", "0") or 0.0), 4),
                "sat": int(row.get("saturated", "0") or 0),
            }
        )
    return {"label": label, "color": color, "samples": samples}


def build_html(data: list[dict]) -> str:
    payload = json.dumps(data, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rocket Flight GNC Animation</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f8fafc;
      --panel: #ffffff;
      --ink: #111827;
      --muted: #475569;
      --grid: #cbd5e1;
      --axis: #334155;
      --open: #dc2626;
      --ideal: #2563eb;
      --tvc: #059669;
      --accent: #7c3aed;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 24px;
      background: var(--bg);
      color: var(--ink);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    main {{ max-width: 1180px; margin: 0 auto; }}
    h1 {{ margin: 0 0 6px; font-size: 24px; font-weight: 700; }}
    p {{ margin: 0 0 16px; color: var(--muted); line-height: 1.45; }}
    .toolbar {{
      display: flex;
      align-items: center;
      gap: 12px;
      flex-wrap: wrap;
      margin: 14px 0 16px;
    }}
    .comparison-note {{
      margin: -4px 0 14px;
      padding: 10px 12px;
      border: 1px solid var(--grid);
      border-radius: 8px;
      background: var(--panel);
      color: var(--muted);
      font-size: 14px;
      line-height: 1.4;
    }}
    .legend {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px 16px;
      align-items: center;
      margin: 0 0 14px;
      padding: 10px 12px;
      border: 1px solid var(--grid);
      border-radius: 8px;
      background: var(--panel);
      color: var(--muted);
      font-size: 14px;
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 7px;
      white-space: nowrap;
    }}
    .swatch {{
      width: 22px;
      height: 4px;
      border-radius: 999px;
      display: inline-block;
    }}
    button, select {{
      border: 1px solid var(--axis);
      background: var(--panel);
      color: var(--ink);
      border-radius: 6px;
      padding: 8px 10px;
      font: inherit;
    }}
    input[type="range"] {{ flex: 1 1 320px; min-width: 220px; }}
    .time {{ min-width: 86px; color: var(--muted); font-variant-numeric: tabular-nums; }}
    canvas {{
      width: 100%;
      height: auto;
      display: block;
      background: var(--panel);
      border: 1px solid var(--grid);
      border-radius: 8px;
    }}
    .metrics {{
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      margin-top: 12px;
    }}
    .metric {{
      background: var(--panel);
      border: 1px solid var(--grid);
      border-radius: 8px;
      padding: 10px;
    }}
    .metric strong {{ display: block; font-size: 14px; margin-bottom: 6px; }}
    .metric span {{ display: block; color: var(--muted); font-size: 13px; font-variant-numeric: tabular-nums; }}
    .explanation {{
      margin-top: 12px;
      background: var(--panel);
      border: 1px solid var(--grid);
      border-radius: 8px;
      padding: 12px;
    }}
    .explanation h2 {{
      margin: 0 0 8px;
      font-size: 18px;
      line-height: 1.2;
    }}
    .explanation-grid {{
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
    }}
    .explanation h3 {{
      margin: 0 0 4px;
      font-size: 14px;
      line-height: 1.25;
    }}
    .explanation p {{
      margin: 0;
      font-size: 13px;
      line-height: 1.42;
    }}
    @media (max-width: 720px) {{
      body {{ padding: 14px; }}
      .metrics {{ grid-template-columns: 1fr; }}
      .explanation-grid {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
<main>
  <h1>6-DOF Rocket Flight GNC Animation</h1>
  <p>Synced playback of four separate simulation cases: disturbed open-loop failure, ideal body-torque control, PD thrust-vector control, and LQR thrust-vector control.</p>
  <div class="comparison-note"><strong>Why four rockets?</strong> Each lane is a different run of the same vehicle model under the same disturbance environment. The side-by-side layout shows how the trajectory changes as the control architecture becomes more realistic: no feedback, ideal torque, PD TVC, then LQR TVC.</div>
  <div class="legend" aria-label="Animation color legend">
    <span class="legend-item"><span class="swatch" style="background:#dc2626"></span>red = open loop</span>
    <span class="legend-item"><span class="swatch" style="background:#2563eb"></span>blue = ideal torque</span>
    <span class="legend-item"><span class="swatch" style="background:#059669"></span>green = PD TVC</span>
    <span class="legend-item"><span class="swatch" style="background:#0891b2"></span>teal = LQR TVC</span>
    <span class="legend-item"><span class="swatch" style="background:#7c3aed"></span>purple arrow = gimbal direction</span>
  </div>
  <div class="toolbar">
    <button id="play">Pause</button>
    <input id="time" type="range" min="0" max="200" value="0" aria-label="Animation time">
    <span id="timeLabel" class="time">t = 0.00 s</span>
    <label>Speed
      <select id="speed">
        <option value="0.5">0.5x</option>
        <option value="1" selected>1x</option>
        <option value="2">2x</option>
        <option value="4">4x</option>
      </select>
    </label>
  </div>
  <canvas id="flightCanvas" width="1400" height="720" aria-label="Animated rocket flight comparison"></canvas>
  <div class="metrics" id="metrics"></div>
  <section class="explanation" aria-labelledby="final-frame-heading">
    <h2 id="final-frame-heading">Why the final frame looks like this</h2>
    <div class="explanation-grid">
      <div>
        <h3>Open loop: apparent attitude is not recovery</h3>
        <p>The open-loop body has rotated through more than one revolution. A late positive body-axis dot product only means the thrust axis has come around again; earlier, body_z_z crossed through zero/negative values, so T cos(theta) stopped supporting ascent while T sin(theta) built crossrange velocity.</p>
      </div>
      <div>
        <h3>Ideal torque: decoupled moment authority</h3>
        <p>The ideal controller applies Kp e - Kd omega as direct body torque. It damps transverse angular-rate energy without rotating the thrust force, so attitude correction is decoupled from lateral acceleration and vertical thrust projection is preserved.</p>
      </div>
      <div>
        <h3>PD TVC: actuator-coupled stabilization</h3>
        <p>The PD TVC case realizes torque through r_engine x F_thrust. The same lateral thrust component that creates stabilizing pitch/yaw moment also enters m a_lateral, so the vehicle remains stable but carries more drift than the ideal-torque case.</p>
      </div>
      <div>
        <h3>LQR TVC: local Q/R trade</h3>
        <p>The LQR TVC case is designed from the upright double-integrator attitude model and penalizes attitude error, angular rate, and torque effort. It still pays the lateral-thrust cost of TVC, but the selected Q/R weighting reduces tilt and drift in this reference envelope.</p>
      </div>
    </div>
  </section>
</main>
<script>
const DATA = {payload};
const canvas = document.getElementById('flightCanvas');
const ctx = canvas.getContext('2d');
const slider = document.getElementById('time');
const playButton = document.getElementById('play');
const speedSelect = document.getElementById('speed');
const timeLabel = document.getElementById('timeLabel');
const metrics = document.getElementById('metrics');
const maxFrames = Math.max(...DATA.map(s => s.samples.length));
const tMax = Math.max(...DATA.map(s => s.samples[s.samples.length - 1].t));
const laneCount = DATA.length;
slider.max = String(maxFrames - 1);
let frame = 0;
let playing = true;
let last = performance.now();

function sample(series, i) {{
  const idx = Math.min(series.samples.length - 1, i);
  return series.samples[idx];
}}

function extrema() {{
  const xs = DATA.flatMap(s => s.samples.map(p => p.x));
  const zs = DATA.flatMap(s => s.samples.map(p => p.z));
  return {{ minX: Math.min(...xs, -2), maxX: Math.max(...xs, 2), minZ: Math.min(...zs, -2), maxZ: Math.max(...zs, 34) }};
}}
const bounds = extrema();

function project(p, lane) {{
  const marginX = 48;
  const laneW = (canvas.width - 2 * marginX) / laneCount;
  const left = marginX + lane * laneW;
  const pad = 30;
  const xSpan = Math.max(1, bounds.maxX - bounds.minX);
  const zSpan = Math.max(1, bounds.maxZ - bounds.minZ);
  const px = left + pad + (p.x - bounds.minX) / xSpan * (laneW - 2 * pad);
  const py = canvas.height - 118 - (p.z - bounds.minZ) / zSpan * 450;
  return [px, py];
}}

function drawArrow(x, y, angle, length, color) {{
  const ex = x + Math.sin(angle) * length;
  const ey = y - Math.cos(angle) * length;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ex, ey); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - Math.sin(angle + 0.55) * 10, ey + Math.cos(angle + 0.55) * 10);
  ctx.lineTo(ex - Math.sin(angle - 0.55) * 10, ey + Math.cos(angle - 0.55) * 10);
  ctx.closePath(); ctx.fill();
}}

function drawRocket(x, y, bodyAngle, color, gimbalDeg) {{
  const length = 54;
  const width = 13;
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(bodyAngle);
  ctx.fillStyle = color;
  ctx.globalAlpha = 0.9;
  ctx.fillRect(-width / 2, -length * 0.62, width, length);
  ctx.beginPath();
  ctx.moveTo(0, -length * 0.82);
  ctx.lineTo(-width / 2, -length * 0.62);
  ctx.lineTo(width / 2, -length * 0.62);
  ctx.closePath();
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.strokeStyle = '#111827';
  ctx.lineWidth = 1.2;
  ctx.strokeRect(-width / 2, -length * 0.62, width, length);
  ctx.restore();

  drawArrow(x, y, bodyAngle, 46, color);
  if (gimbalDeg) {{
    drawArrow(x, y + 22, bodyAngle + gimbalDeg * Math.PI / 180 + Math.PI, 34, '#7c3aed');
  }}
}}

function drawLane(series, lane) {{
  const s = sample(series, frame);
  const color = series.color;
  const laneW = (canvas.width - 96) / laneCount;
  const left = 48 + lane * laneW;
  const right = left + laneW;
  ctx.strokeStyle = '#cbd5e1';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(left + 22, canvas.height - 118); ctx.lineTo(right - 22, canvas.height - 118); ctx.stroke();
  ctx.fillStyle = '#111827';
  ctx.font = '600 18px system-ui';
  ctx.fillText(series.label, left + 26, 40);
  ctx.font = '500 12px system-ui';
  ctx.fillStyle = '#475569';
  ctx.fillText('separate simulation case', left + 26, 60);

  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i <= frame && i < series.samples.length; i++) {{
    const [x, y] = project(series.samples[i], lane);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }}
  ctx.stroke();

  const [x, y] = project(s, lane);
  const bodyAngle = Math.atan2(s.bzx, s.bzz);
  drawRocket(x, y, bodyAngle, color, series.label.includes('TVC') ? s.gimbal_x : 0);
}}

function drawMiniPlot(x, y, w, h, series, key, label) {{
  const values = DATA.flatMap(d => d.samples.map(p => p[key] ?? 0));
  const min = Math.min(...values), max = Math.max(...values);
  ctx.strokeStyle = '#cbd5e1'; ctx.lineWidth = 1;
  ctx.strokeRect(x, y, w, h);
  ctx.fillStyle = '#475569'; ctx.font = '500 12px system-ui'; ctx.fillText(label, x, y - 6);
  let lx = x + w - 280;
  ctx.font = '500 10px system-ui';
  for (const d of DATA) {{
    ctx.strokeStyle = d.color;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(lx, y + 12); ctx.lineTo(lx + 14, y + 12); ctx.stroke();
    ctx.fillStyle = '#475569';
    ctx.fillText(d.label, lx + 18, y + 15);
    lx += 68;
  }}
  for (const d of DATA) {{
    ctx.strokeStyle = d.color; ctx.lineWidth = 2; ctx.beginPath();
    for (let i = 0; i <= frame && i < d.samples.length; i++) {{
      const p = d.samples[i];
      const px = x + (p.t / tMax) * w;
      const py = y + h - ((p[key] ?? 0) - min) / Math.max(1e-9, max - min) * h;
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    }}
    ctx.stroke();
  }}
}}

function draw() {{
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  for (let lane = 0; lane < DATA.length; lane++) drawLane(DATA[lane], lane);
  drawMiniPlot(80, 575, 340, 86, DATA, 'bzz', 'body z dot up');
  drawMiniPlot(530, 575, 340, 86, DATA, 'lateral', 'lateral drift (m)');
  drawMiniPlot(980, 575, 340, 86, DATA, 'gimbal', 'TVC gimbal (deg)');
  const current = sample(DATA[0], frame);
  timeLabel.textContent = `t = ${{current.t.toFixed(2)}} s`;
  slider.value = String(frame);
  metrics.innerHTML = DATA.map(series => {{
    const p = sample(series, frame);
    const sat = series.label.includes('TVC') ? `, gimbal ${{p.gimbal.toFixed(2)}} deg${{p.sat ? ', saturated' : ''}}` : '';
    return `<div class="metric"><strong style="color:${{series.color}}">${{series.label}}</strong><span>alt ${{p.z.toFixed(2)}} m, pitch ${{p.pitch.toFixed(1)}} deg</span><span>body z dot up ${{p.bzz.toFixed(3)}}, lateral ${{p.lateral.toFixed(2)}} m${{sat}}</span></div>`;
  }}).join('');
}}

function tick(now) {{
  const speed = Number(speedSelect.value);
  if (playing && now - last > 50 / speed) {{
    frame = (frame + 1) % maxFrames;
    last = now;
    draw();
  }}
  requestAnimationFrame(tick);
}}

playButton.addEventListener('click', () => {{
  playing = !playing;
  playButton.textContent = playing ? 'Pause' : 'Play';
}});
slider.addEventListener('input', () => {{
  frame = Number(slider.value);
  draw();
}});
draw();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


def main() -> None:
    data = [
        load_series(OUT / "week2_disturbed_uncontrolled.csv", "Open loop", "#dc2626"),
        load_series(OUT / "week3a_controlled_ideal_torque.csv", "Ideal torque", "#2563eb"),
        load_series(OUT / "week3b_tvc_controlled.csv", "PD TVC", "#059669"),
        load_series(OUT / "week4a_lqr_tvc_controlled.csv", "LQR TVC", "#0891b2"),
    ]
    path = OUT / "rocket_flight_animation.html"
    path.write_text(build_html(data))
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()
