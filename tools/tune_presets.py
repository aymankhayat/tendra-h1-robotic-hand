"""Tune the thumb values of each grasp preset against real part geometry (dev tool).

The finger pose stays fixed. Thumb opposition and curl are searched to
minimise (penetration of thumb parts into everything else) + (distance from
the thumb pad to that grasp's contact target).

    blender -b --factory-startup -P tools/tune_presets.py
"""
import bpy, os, sys, json
from mathutils import Vector
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import hand_generator as hg
import pipeline as pl

ctx = hg.build({})
S = hg._S
presets = json.loads(pl.rig()["rh_presets"])
TARGETS = {  # (object whose centre the thumb pad should approach, weight)
    "Pinch": ("RH_Tactile_Sensor_Pad_Index", 1.0),
    "OK Sign": ("RH_Tactile_Sensor_Pad_Index", 1.0),
    "Fist": ("RH_Middle_Phalanx_Middle", 0.6),
    "Point": ("RH_Middle_Phalanx_Middle", 0.6),
    "Power Grip": ("RH_Middle_Phalanx_Index", 0.5),
}

def centre(o):
    dg = bpy.context.evaluated_depsgraph_get(); ev = o.evaluated_get(dg)
    return sum((ev.matrix_world @ Vector(c) for c in ev.bound_box), Vector()) / 8

thumb_objs = [o for o in pl.part_objects() if o["rh_assembly"] == "Thumb" and "Socket" not in o["rh_name"]]
pad = bpy.data.objects["RH_Tactile_Sensor_Pad_Thumb"]
solid_kw = ("Phalanx", "Metacarpal", "Yoke", "Bearing", "Screw", "Pulley", "Light Ring", "Sensor Pad", "Cap", "Conduit")
results = {}
for name, (target_name, w) in TARGETS.items():
    pl.apply_preset(name)
    others = [o for o in pl.part_objects() if o["rh_assembly"] not in ("Thumb", "Forearm", "Wrist")
              and any(k in o["rh_name"] for k in solid_kw)]
    other_cache = pl.build_geometry_cache(others)
    target = bpy.data.objects[target_name]
    tgt = centre(target)
    best = None
    for o10 in range(0, 21):
        for c10 in range(0, 21):
            opp, curl = o10 / 20, c10 / 20
            pl.set_state(thumb_opposition=opp, thumb_curl=curl)
            tc = pl.build_geometry_cache(thumb_objs)
            pen = 0.0
            for a in tc.values():
                for b in other_cache.values():
                    if pl._aabb_hit(a, b, 0):
                        pen += max(pl.penetration(a, b, 0.2 * S), pl.penetration(b, a, 0.2 * S)) / S
            dist = (centre(pad) - tgt).length / S
            score = 20 * pen + w * dist
            if best is None or score < best[0]:
                best = (score, opp, curl, round(pen, 2), round(dist, 1))
    results[name] = best
    print("PRESET", name, best, flush=True)
json.dump(results, open(os.path.join(ROOT, "output", "preset_tuning.json"), "w"), indent=1)
