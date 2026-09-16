"""One-page product datasheet (HTML + PDF) generated from the model's own data.

Inputs : output/spec.json, output/bom.csv, output/verification.json, output/renders/*
Outputs: output/TENDRA-H1_datasheet.html and .pdf (printed with headless Edge/Chrome)

    python tools/build_spec_sheet.py
"""
import csv
import html
import json
import os
import shutil
import subprocess
from collections import OrderedDict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "output")

spec = json.load(open(os.path.join(OUT, "spec.json"), encoding="utf-8"))
verification = json.load(open(os.path.join(OUT, "verification.json"), encoding="utf-8"))
with open(os.path.join(OUT, "bom.csv"), encoding="utf-8") as fh:
    bom = [r for r in csv.DictReader(fh) if r.get("Qty") and r["Item"] != "TOTAL"]
esc = html.escape

# Range of motion, grouped by joint type (every finger shares the same limits).
fingers = [f["name"] for f in spec["fingers"]]
rom = OrderedDict()
for j in spec["joints"]:
    name = j["joint"]
    owner = next((f for f in fingers if name.startswith(f + " ")), None)
    key = name[len(owner) + 1:] if owner else name
    row = rom.setdefault(key, dict(range=(j["min_deg"], j["max_deg"]), count=0, actuator=j["actuator"]))
    row["count"] += 1
    if owner:
        row["actuator"] = j["actuator"].replace(owner, "Finger")

rom_rows = "".join(
    f"<tr><td>{esc(k[0].upper() + k[1:])}</td><td class=n>{v['range'][0]:+.0f}°</td><td class=n>{v['range'][1]:+.0f}°</td>"
    f"<td class=n>×{v['count']}</td><td class=drv>{esc(v['actuator'])}</td></tr>" for k, v in rom.items())

top_bom = sorted(bom, key=lambda r: -float(r["Total cost (USD)"]))[:6]
bom_rows = "".join(
    f"<tr><td>{esc(r['Item'])}</td><td class=n>{r['Qty']}</td><td class=n>{float(r['Total mass (g)']):.1f}</td>"
    f"<td class=n>${float(r['Total cost (USD)']):,.0f}</td></tr>" for r in top_bom)

mass = spec["mass_by_assembly"]
mass_max = max(mass.values())
mass_rows = "".join(
    f"<div class=bar><span>{esc(k)}</span><i style='width:{v / mass_max * 100:.1f}%'></i><b>{v:.0f} g</b></div>"
    for k, v in sorted(mass.items(), key=lambda kv: -kv[1]))

d = spec["dimensions"]
presets = " · ".join(spec["presets"].keys())
checks_ok = not verification["rest_interference"] and verification["components"] == 1 and \
    all(not v for v in verification["poses"].values())

page = f"""<!doctype html><html lang=en><head><meta charset=utf-8>
<title>{spec['product']} Datasheet</title>
<link rel=stylesheet href="https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@600;700&family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
@page {{ size: A4; margin: 0; }}
:root {{ --ink:#10161d; --muted:#5d6b77; --line:#d7dee4; --accent:#0aa7c9; --band:#070b10; }}
* {{ box-sizing: border-box; }}
body {{ margin:0; font:8.6pt/1.36 "IBM Plex Sans","Segoe UI",sans-serif; color:var(--ink); background:#fff; }}
.sheet {{ width:210mm; height:297mm; padding:0 12mm 8mm; display:grid; grid-template-rows:auto auto 1fr auto; gap:4mm; overflow:hidden; }}
header {{ margin:0 -12mm; background:var(--band); color:#e6eef4; display:grid; grid-template-columns:1fr 64mm; height:80mm; overflow:hidden; }}
header .txt {{ padding:9mm 6mm 5mm 12mm; display:flex; flex-direction:column; gap:3mm; }}
.eyebrow {{ font:600 7.5pt "Chakra Petch",sans-serif; letter-spacing:.24em; text-transform:uppercase; color:#19d9ff; }}
h1 {{ margin:0; font:700 34pt/0.95 "Chakra Petch",sans-serif; letter-spacing:.03em; }}
header p {{ margin:0; color:#9fb0bd; max-width:92mm; }}
header img {{ width:100%; height:80mm; object-fit:cover; object-position:50% 22%; }}
.kpis {{ display:grid; grid-template-columns:repeat(5,1fr); border-top:1px solid #1d2a36; margin-top:auto; }}
.kpis div {{ padding:2.5mm 0 0; }}
.kpis b {{ display:block; font:500 15pt "IBM Plex Mono",monospace; color:#fff; }}
.kpis span {{ font:600 6.5pt "Chakra Petch",sans-serif; letter-spacing:.16em; text-transform:uppercase; color:#8093a2; }}
.cols {{ display:grid; grid-template-columns:1fr 1fr; gap:6mm; }}
h2 {{ margin:0 0 1.8mm; font:700 8pt "Chakra Petch",sans-serif; letter-spacing:.2em; text-transform:uppercase; color:var(--accent); border-bottom:1.2pt solid var(--ink); padding-bottom:1mm; }}
table {{ width:100%; border-collapse:collapse; }}
td, th {{ padding:.9mm 0; border-bottom:.5pt solid var(--line); vertical-align:top; text-align:left; }}
th {{ font:600 6.5pt "Chakra Petch",sans-serif; letter-spacing:.14em; text-transform:uppercase; color:var(--muted); }}
td.k {{ color:var(--muted); width:38%; }}
td.drv {{ padding-left:3mm; font-size:8pt; }}
.n {{ font-family:"IBM Plex Mono",monospace; font-size:8.3pt; text-align:right; padding-left:2mm; white-space:nowrap; font-variant-numeric:tabular-nums; }}
section {{ display:flex; flex-direction:column; gap:4mm; }}
.bar {{ display:grid; grid-template-columns:18mm 1fr 13mm; align-items:center; gap:2mm; font-size:8pt; }}
.bar i {{ display:block; height:2.2mm; background:var(--ink); }}
.bar b {{ font:500 8pt "IBM Plex Mono",monospace; text-align:right; }}
.views {{ display:grid; grid-template-columns:repeat(3,1fr); gap:3mm; }}
.views img {{ width:100%; height:46mm; object-fit:cover; display:block; }}
.pass {{ display:inline-block; font:600 7pt "Chakra Petch",sans-serif; letter-spacing:.14em; padding:.6mm 2mm; color:#fff; background:{'#1c8a5a' if checks_ok else '#b3412e'}; }}
footer {{ display:flex; justify-content:space-between; font:7pt "IBM Plex Mono",monospace; color:var(--muted); border-top:.5pt solid var(--line); padding-top:2mm; }}
</style></head><body><div class=sheet>
<header>
  <div class=txt>
    <div class=eyebrow>Product datasheet · Rev A</div>
    <h1>{spec['product']}</h1>
    <p>Tendon-driven dexterous robotic hand with an exposed CF-nylon skeleton, opposable thumb and 2-DOF wrist. Every figure on this page is computed from the parametric model itself.</p>
    <div class=kpis>
      <div><b>{spec['joints_total']}</b><span>Joints</span></div>
      <div><b>{spec['actuators_total']}</b><span>Actuators</span></div>
      <div><b>{spec['mass_g']:.0f} g</b><span>Est. mass</span></div>
      <div><b>${spec['cost_usd']:,.0f}</b><span>BOM cost</span></div>
      <div><b>{spec['part_count']}</b><span>Parts</span></div>
    </div>
  </div>
  <img src="renders/01_hero_assembled_4x5.png" alt="">
</header>
<div class=cols>
  <section>
    <div><h2>General</h2><table>
      <tr><td class=k>Envelope (L × W × D)</td><td class=n>{d['length_mm']:.0f} × {d['width_mm']:.0f} × {d['thickness_mm']:.0f} mm</td></tr>
      <tr><td class=k>Wrist centre to middle tip</td><td class=n>{spec['fingers'][min(1, len(fingers)-1)]['meta_mm'] + spec['fingers'][min(1, len(fingers)-1)]['prox_mm'] + spec['fingers'][min(1, len(fingers)-1)]['mid_mm'] + spec['fingers'][min(1, len(fingers)-1)]['dist_mm'] + 30:.0f} mm</td></tr>
      <tr><td class=k>Fingers</td><td class=n>{len(fingers)} + opposable thumb</td></tr>
      <tr><td class=k>Unique line items</td><td class=n>{spec['unique_parts']}</td></tr>
      <tr><td class=k>Structure</td><td>PA12-CF links (MJF), Ti-6Al-4V yokes &amp; gimbal, 7075-T6 frame</td></tr>
      <tr><td class=k>Joint hardware</td><td>Ø3 mm 440C pins, flanged ball bearings, M2 12.9 screws</td></tr>
      <tr><td class=k>Sensing</td><td>Capacitive tactile pad per fingertip, magnetic encoder per actuator</td></tr>
    </table></div>
    <div><h2>Actuation</h2><table>
      <tr><td class=k>Method</td><td>{esc(spec['actuation'])}</td></tr>
      <tr><td class=k>Actuators</td><td>{spec['actuators_total']} × Ø9.5 mm coreless DC gearmotor, 136:1</td></tr>
      <tr><td class=k>Tendon</td><td>Dyneema Ø1.0 braid over grooved 440C pulleys</td></tr>
      <tr><td class=k>Grasp presets</td><td>{esc(presets)}</td></tr>
    </table></div>
    <div><h2>Mass by assembly</h2>{mass_rows}</div>
  </section>
  <section>
    <div><h2>Range of motion</h2><table>
      <tr><th>Joint</th><th class=n>Min</th><th class=n>Max</th><th class=n>Qty</th><th>Driven by</th></tr>
      {rom_rows}
    </table></div>
    <div><h2>Top cost drivers</h2><table>
      <tr><th>Item</th><th class=n>Qty</th><th class=n>g</th><th class=n>USD</th></tr>
      {bom_rows}
    </table></div>
  </section>
</div>
<div>
  <h2>Orthographic views &nbsp; <span class=pass>{'VERIFIED · 0 INTERFERENCES · NO FLOATING PARTS' if checks_ok else 'VERIFICATION ISSUES — SEE verification.json'}</span></h2>
  <div class=views>
    <img src="renders/06_ortho_front.png" alt="Front view"><img src="renders/06_ortho_side.png" alt="Side view"><img src="renders/06_ortho_top.png" alt="Top view">
  </div>
</div>
<footer><span>Designed &amp; built by Ayman Khayat · linkedin.com/in/ayman-khayat-350b4b335 · generated by hand_generator.py</span><span>{spec['product']} · Rev A</span></footer>
</div></body></html>"""

html_path = os.path.join(OUT, "TENDRA-H1_datasheet.html")
with open(html_path, "w", encoding="utf-8") as fh:
    fh.write(page)
print("wrote", html_path)

browser = next((p for p in (r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
                            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
                            shutil.which("chromium"), shutil.which("google-chrome")) if p and os.path.exists(p)), None)
if browser:
    pdf = os.path.join(OUT, "TENDRA-H1_datasheet.pdf")
    import tempfile
    profile = os.path.join(tempfile.gettempdir(), "tendra-pdf-profile")   # isolated profile: never attaches to a running browser
    subprocess.run([browser, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", f"--user-data-dir={profile}",
                    "--virtual-time-budget=4000", f"--print-to-pdf={pdf}", "file:///" + html_path.replace("\\", "/")],
                   check=False, capture_output=True, timeout=120)
    print("wrote", pdf if os.path.exists(pdf) else "(PDF export failed)")
