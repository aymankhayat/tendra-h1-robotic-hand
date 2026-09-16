"""Grid-search the thumb opposition mapping (dev tool).

Forward kinematics come straight from the generated armature's rest
matrices. The score rewards thumb-pad to index-pad contact with opposed pad
normals and penalises the two chains passing through each other.

    blender -b --factory-startup -P tools/tune_thumb.py
"""
import bpy, os, sys, itertools, math
from math import radians
from mathutils import Matrix, Vector, Euler
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import hand_generator as hg

ctx = hg.build({})
bones = ctx.rig.data.bones
rest = {b.name: b.matrix_local.copy() for b in bones}
S = hg._S

def fk(chain_names, rots):
    """World matrices of a bone chain rooted at palm (palm pose = rest)."""
    out, parent_world, parent_rest = [], rest["palm"], rest["palm"]
    for name, eul in zip(chain_names, rots):
        local = parent_rest.inverted() @ rest[name]
        world = parent_world @ local @ Euler(eul, "XYZ").to_matrix().to_4x4()
        out.append(world)
        parent_world, parent_rest = world, rest[name]
    return out

idx = ctx.chains[0]; th = ctx.thumb
iL = [l["length"] * S for l in idx.links]; iR = [l["R"] * S for l in idx.links]
tL = [l["length"] * S for l in th.links]; tR = [l["R"] * S for l in th.links]

def index_pose(c):
    return fk([l["bone"] for l in idx.links],
              [(0,0,0), (0,0,0), (radians(85*c),0,0), (radians(105*c),0,0), (radians(75*c),0,0)])

def thumb_pose(ax, ay, az, t):
    return fk([l["bone"] for l in th.links],
              [(radians(ax), radians(ay), radians(az)), (radians(55*t),0,0), (radians(75*t),0,0)])

def pad(m, L, R):
    y = (0.45 * L + L - R) / 2
    return m @ Vector((0, y, R + 0.00045)), (m.to_3x3() @ Vector((0, 0, 1))).normalized()

def samples(mats, lens, radii, skip_tip_frac=0.0):
    pts = []
    for m, L, R in zip(mats, lens, radii):
        for k in range(6):
            pts.append((m @ Vector((0, L * k / 5, 0)), R))
    return pts

best = []
palm_pts = [(Vector((x, 0, z)) * S, 0.0) for x in (-30, -10, 10, 30) for z in (15, 25, 34)]
for c in (0.45, 0.55, 0.65):
    im = index_pose(c)
    ipad, inorm = pad(im[4], iL[4], iR[4])
    isamp = samples(im[2:], iL[2:], iR[2:]) + samples(im[:1], iL[:1], iR[:1])
    for ax in range(-20, 81, 10):
        for ay in range(-90, 91, 10):
            for az in range(-80, 81, 10):
                for t10 in range(0, 9):
                    t = t10 / 10
                    tm = thumb_pose(ax, ay, az, t)
                    tpad, tnorm = pad(tm[2], tL[2], tR[2])
                    dist = (tpad - ipad).length
                    score = dist / S + 12 * (1 + tnorm.dot(inorm))
                    if score > 30:
                        continue
                    pen = 0.0
                    for p, r in samples(tm, tL, tR):
                        for q, r2 in isamp:
                            d = (p - q).length - (r + r2) * 0.95
                            if d < 0: pen += -d / S
                        for q, _ in palm_pts:
                            d = (p - q).length - r - 8 * S
                            if d < 0: pen += -d / S
                    score += 3 * pen
                    best.append((score, dict(c=c, ax=ax, ay=ay, az=az, t=t, pad_dist_mm=round(dist / S, 1),
                                             normal_dot=round(tnorm.dot(inorm), 2), pen=round(pen, 1))))
best.sort(key=lambda b: b[0])
for sc, b in best[:12]:
    print("TUNE", round(sc, 2), b)


# ---- continuous refinement (pattern search) from the best grid seeds ----------
def evaluate(v):
    c, ax, ay, az, t = v
    if not (0.3 <= c <= 0.8 and 0 <= t <= 1): return 1e9, 0, 0, 0
    im = index_pose(c)
    ipad, inorm = pad(im[4], iL[4], iR[4])
    tm = thumb_pose(ax, ay, az, t)
    tpad, tnorm = pad(tm[2], tL[2], tR[2])
    dist = (tpad - ipad).length
    pen = 0.0
    isamp = samples(im[2:], iL[2:], iR[2:]) + samples(im[:1], iL[:1], iR[:1])
    for p, r in samples(tm, tL, tR):
        for q, r2 in isamp:
            d = (p - q).length - (r + r2) * 0.95
            if d < 0: pen += -d / S
    # prefer small joint excursions (natural-looking opposition)
    effort = (abs(ax) + abs(ay) + abs(az)) / 60.0
    return 3 * dist / S + 6 * (1 + tnorm.dot(inorm)) + 3 * pen + effort, dist / S, tnorm.dot(inorm), pen

refined = []
seen = set()
for _, b in best[:40]:
    key = (b["ax"], b["ay"], b["az"])
    if key in seen: continue
    seen.add(key)
    v = [b["c"], b["ax"], b["ay"], b["az"], b["t"]]
    steps = [0.05, 5.0, 5.0, 5.0, 0.1]
    cur = evaluate(v)[0]
    for it in range(60):
        improved = False
        for i in range(5):
            for sgn in (1, -1):
                w = list(v); w[i] += sgn * steps[i]
                sc = evaluate(w)[0]
                if sc < cur:
                    v, cur, improved = w, sc, True
        if not improved:
            steps = [s * 0.5 for s in steps]
            if steps[1] < 0.2: break
    refined.append((cur, v, evaluate(v)))
refined.sort(key=lambda r: r[0])
for cur, v, ev in refined[:6]:
    print("REFINE", round(cur, 2), [round(x, 2) for x in v], "dist_mm", round(ev[1], 2), "ndot", round(ev[2], 2), "pen", round(ev[3], 2))


# ---- CMC placement sweep -------------------------------------------------------
base_rest = {k: v.copy() for k, v in rest.items()}
results = []
for dx in (-6, -3, 0):
    for dy in (-8, -4, 0):
        for dz in (0, 5, 10, 15):
            delta = Vector((dx, dy, dz)) * S
            for name in ("thumb_cmc", "thumb_mcp", "thumb_ip"):
                rest[name] = base_rest[name].copy()
                rest[name].translation = base_rest[name].translation + delta
            bestv = None
            for seed in ([0.5, 30, -20, -15, 0.2], [0.5, 60, 50, 70, 0.1], [0.55, 20, 0, 0, 0.3], [0.6, 40, 20, 30, 0.4]):
                v, steps = list(seed), [0.05, 8.0, 8.0, 8.0, 0.15]
                cur = evaluate(v)[0]
                for it in range(80):
                    improved = False
                    for i in range(5):
                        for sgn in (1, -1):
                            w = list(v); w[i] += sgn * steps[i]
                            sc = evaluate(w)[0]
                            if sc < cur: v, cur, improved = w, sc, True
                    if not improved:
                        steps = [s * 0.5 for s in steps]
                        if steps[1] < 0.3: break
                if bestv is None or cur < bestv[0]:
                    bestv = (cur, v)
            ev = evaluate(bestv[1])
            results.append((bestv[0], (dx, dy, dz), bestv[1], ev))
results.sort(key=lambda r: r[0])
for r in results[:8]:
    print("CMC", round(r[0], 2), r[1], [round(x, 2) for x in r[2]], "dist", round(r[3][1], 2), "ndot", round(r[3][2], 2), "pen", round(r[3][3], 2))
