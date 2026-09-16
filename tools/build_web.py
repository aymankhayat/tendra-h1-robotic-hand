"""Assemble web/index.html: the landing page template plus the embedded GLB, clip metadata, spec summary and top BOM lines.

The GLB is embedded as base64 so the viewer is a single file that works offline,
on GitHub Pages, or inside a sandboxed page that blocks network fetches.

    python tools/build_web.py        (any Python 3; Blender's bundled one works)
"""
import base64
import csv
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "web")

with open(os.path.join(WEB, "template.html"), encoding="utf-8") as fh:
    html = fh.read()
with open(os.path.join(WEB, "tendra_h1.glb"), "rb") as fh:
    glb = base64.b64encode(fh.read()).decode("ascii")
with open(os.path.join(WEB, "tendra_h1_meta.json"), encoding="utf-8") as fh:
    meta = json.load(fh)
with open(os.path.join(ROOT, "output", "spec.json"), encoding="utf-8") as fh:
    spec = json.load(fh)
summary = {k: spec[k] for k in ("joints_total", "actuators_total", "mass_g", "cost_usd", "part_count")}
with open(os.path.join(ROOT, "output", "bom.csv"), encoding="utf-8") as fh:
    rows = [r for r in csv.DictReader(fh) if r.get("Qty") and r["Item"] != "TOTAL"]
rows.sort(key=lambda r: -float(r["Total cost (USD)"]))
summary["top_bom"] = [dict(item=r["Item"], qty=int(r["Qty"]), cost=float(r["Total cost (USD)"])) for r in rows[:6]]

html = (html.replace("__META_JSON__", json.dumps(meta))
            .replace("__SPEC_JSON__", json.dumps(summary))
            .replace("__GLB_BASE64__", glb))
out = os.path.join(WEB, "index.html")
with open(out, "w", encoding="utf-8") as fh:
    fh.write(html)
print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")
