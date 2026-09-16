"""
TENDRA-H1 production pipeline (run headless inside Blender)

    blender -b --factory-startup -P pipeline.py -- --steps check,preview
    blender -b --factory-startup -P pipeline.py -- --steps all

Steps
  check    interference + floating-part verification (rest pose and every grasp preset)
  preview  fast Workbench renders from several angles (geometry sanity pass)
  stills   Cycles renders: hero / exploded / labeled technical, in 4:5 and 16:9
  poses    grasp-taxonomy renders (one per preset)
  ortho    front / side / top blueprint orthographics with dimension lines
  variant  second parameter set, rendered (proves the script is a generator)
  save     .blend + BOM / mass / cost / spec reports
  glb      web export with explode + pose clips baked as NLA animations
  anim     explode -> reassemble -> grip showcase animation (EEVEE)
"""

import bpy
import json
import math
import os
import sys
import time
from math import radians
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hand_generator as hg  # noqa: E402

OUT = os.path.join(HERE, "output")
REN = os.path.join(OUT, "renders")
os.makedirs(REN, exist_ok=True)


def argv():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    opts = {"steps": "check,preview", "samples": 96, "pct": 100, "frames": None}
    i = 0
    while i < len(a):
        if a[i].startswith("--"):
            opts[a[i][2:]] = a[i + 1]
            i += 2
        else:
            i += 1
    return opts


def log(*msg):
    print("[pipeline]", *msg, flush=True)


# -----------------------------------------------------------------------------
# scene helpers
# -----------------------------------------------------------------------------

def rig():
    return bpy.data.objects["RH_Rig"]


def set_state(**props):
    r = rig()
    for k, v in props.items():
        r[k] = v
    r.update_tag()           # ID-property edits don't tag the depsgraph on their own
    sc = bpy.context.scene
    sc.frame_set(sc.frame_current)
    bpy.context.view_layer.update()
    # Second pass: bone drivers evaluate before their child objects' matrices settle.
    for ob in bpy.data.objects:
        if ob.parent is r:
            ob.update_tag()
    bpy.context.view_layer.update()


def reset_pose(ctx):
    vals = dict(explode=0.0, show_labels=False, technical_view=False, isolate=False, spread=0.0,
                palm_cup=0.0, thumb_curl=0.0, thumb_opposition=0.0, wrist_pitch=0.0, wrist_yaw=0.0, label_detail=0)
    for ch in ctx.chains:
        vals[f"curl_{ch.key}"] = 0.0
    set_state(**vals)


def apply_preset(name, **extra):
    presets = json.loads(rig()["rh_presets"])
    vals = dict(presets[name])
    vals.update(extra)
    set_state(**vals)


def part_objects():
    return [o for o in bpy.data.objects if o.get("rh_part")]


def world_points(objs):
    dg = bpy.context.evaluated_depsgraph_get()
    pts = []
    for o in objs:
        ev = o.evaluated_get(dg)
        m = ev.matrix_world
        pts += [m @ Vector(c) for c in ev.bound_box]
    return pts


def frame_camera(cam, direction, objs, aspect, margin=1.08, lens=None, up=Vector((0, 0, 1)), extra_pts=()):
    """Aim `cam` along `direction` and back it off until every bbox corner fits."""
    if lens:
        cam.data.lens = lens
    pts = world_points(objs) + list(extra_pts)
    fwd = Vector(direction).normalized()
    right = fwd.cross(up).normalized()
    upv = right.cross(fwd).normalized()
    c0 = sum(pts, Vector()) / len(pts)
    xs = [(p - c0).dot(right) for p in pts]
    ys = [(p - c0).dot(upv) for p in pts]
    center = c0 + right * (max(xs) + min(xs)) / 2 + upv * (max(ys) + min(ys)) / 2
    rot = Matrix((right, upv, -fwd)).transposed().to_4x4()
    cam.data.sensor_fit = "AUTO"
    if cam.data.type == "ORTHO":
        w = (max(xs) - min(xs)) * margin
        h = (max(ys) - min(ys)) * margin
        cam.data.ortho_scale = max(w, h * aspect) if aspect >= 1 else max(w / aspect, h)
        cam.data.ortho_scale = max(w, h * aspect) if aspect >= 1 else max(h, w / aspect)
        dist = 3.0
    else:
        sensor = cam.data.sensor_width
        half_fov = math.atan(sensor / 2 / cam.data.lens)
        tan_w = math.tan(half_fov) if aspect >= 1 else math.tan(half_fov) * aspect
        tan_h = tan_w / aspect
        dist = 0.0
        for p in pts:
            d = p - center
            x, y, z = d.dot(right), d.dot(upv), -d.dot(fwd)
            dist = max(dist, abs(x) * margin / tan_w + z, abs(y) * margin / tan_h + z)
    m = rot.copy()
    m.translation = center - fwd * dist
    cam.matrix_world = m
    return cam


def point_labels_at(cam):
    for o in bpy.data.objects:
        for c in o.constraints:
            if c.type in ("TRACK_TO", "DAMPED_TRACK") and (o.name.startswith("RH_Label") or o.name == "RH_SpecCallout"):
                c.target = cam


def setup_cycles(samples):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = int(samples)
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.03
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 8
    sc.cycles.transparent_max_bounces = 24
    sc.cycles.glossy_bounces = 4
    sc.cycles.caustics_reflective = False
    sc.cycles.caustics_refractive = False
    sc.render.film_transparent = False
    colour_management()
    add_glare()


def colour_management():
    vs = bpy.context.scene.view_settings
    try:
        vs.view_transform = "AgX"
        for look in ("AgX - Medium High Contrast", "Medium High Contrast", "AgX - Punchy", "Punchy"):
            try:
                vs.look = look
                break
            except TypeError:
                continue
    except TypeError:
        pass
    vs.exposure = 0.35


def add_glare(strength=0.30):
    """Bloom on the emissive accents (Blender 5 compositor node-group API)."""
    sc = bpy.context.scene
    try:
        tree = bpy.data.node_groups.get("RH_Compositor") or bpy.data.node_groups.new("RH_Compositor", "CompositorNodeTree")
        tree.nodes.clear()
        if not any(s.name == "Image" for s in tree.interface.items_tree):
            tree.interface.new_socket("Image", in_out="OUTPUT", socket_type="NodeSocketColor")
        rl = tree.nodes.new("CompositorNodeRLayers")
        gl = tree.nodes.new("CompositorNodeGlare")
        gl.inputs["Type"].default_value = "Bloom"
        gl.inputs["Threshold"].default_value = 1.6
        gl.inputs["Strength"].default_value = strength
        gl.inputs["Size"].default_value = 0.35
        out = tree.nodes.new("NodeGroupOutput")
        tree.links.new(rl.outputs["Image"], gl.inputs["Image"])
        tree.links.new(gl.outputs["Image"], out.inputs[0])
        sc.compositing_node_group = tree
    except Exception as exc:
        log("glare skipped:", exc)


def remove_glare():
    try:
        bpy.context.scene.compositing_node_group = None
    except Exception:
        pass


def setup_workbench():
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "MATERIAL"
    sh.show_cavity = True
    sh.cavity_type = "BOTH"
    sh.show_object_outline = False
    sh.show_specular_highlight = True
    sc.display.render_aa = "8"
    sc.view_settings.view_transform = "Standard"
    remove_glare()


def render(path, cam, res, pct=100):
    sc = bpy.context.scene
    sc.camera = cam
    point_labels_at(cam)
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = int(pct)
    sc.render.image_settings.media_type = "IMAGE"
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = path
    t = time.time()
    bpy.ops.render.render(write_still=True)
    log(f"rendered {os.path.basename(path)} {res[0]}x{res[1]}@{pct}% in {time.time() - t:.1f}s")


def get_cam(name, ortho=False):
    cam = bpy.data.objects.get(name)
    if cam is None:
        data = bpy.data.cameras.new(name)
        cam = bpy.data.objects.new(name, data)
        bpy.data.collections["RH Studio"].objects.link(cam)
    cam.data.type = "ORTHO" if ortho else "PERSP"
    cam.data.clip_start, cam.data.clip_end = 0.002, 50
    return cam


def hand_parts(include_hidden=False):
    return [o for o in part_objects()]


def set_stand_and_floor(stand=True, floor=True):
    for name, vis in (("RH_Floor", floor),):
        ob = bpy.data.objects.get(name)
        if ob:
            ob.hide_render = not vis


# -----------------------------------------------------------------------------
# verification
# -----------------------------------------------------------------------------

def _world_mesh(o, dg):
    ev = o.evaluated_get(dg)
    me = ev.to_mesh()
    m = ev.matrix_world
    verts = [m @ v.co for v in me.vertices]
    polys = [tuple(p.vertices) for p in me.polygons]
    ev.to_mesh_clear()
    return verts, polys


def _islands(verts, polys):
    """Split a mesh into its separate closed solids (primitives never share vertices)."""
    parent = list(range(len(verts)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for p in polys:
        r0 = find(p[0])
        for v in p[1:]:
            r = find(v)
            if r != r0:
                parent[r] = r0
    groups = {}
    for p in polys:
        groups.setdefault(find(p[0]), []).append(p)
    out = []
    for group in groups.values():
        used = sorted({v for p in group for v in p})
        remap = {v: i for i, v in enumerate(used)}
        iv = [verts[v] for v in used]
        mn = Vector(map(min, *[tuple(v) for v in iv]))
        mx = Vector(map(max, *[tuple(v) for v in iv]))
        out.append((BVHTree.FromPolygons(iv, [tuple(remap[v] for v in p) for p in group]), mn, mx))
    return out


def build_geometry_cache(objs):
    dg = bpy.context.evaluated_depsgraph_get()
    cache = {}
    for o in objs:
        verts, polys = _world_mesh(o, dg)
        if not verts:
            continue
        mn = Vector(map(min, *[tuple(v) for v in verts])) if len(verts) > 1 else verts[0]
        mx = Vector(map(max, *[tuple(v) for v in verts])) if len(verts) > 1 else verts[0]
        cache[o.name] = dict(obj=o, verts=verts, bvh=BVHTree.FromPolygons(verts, polys), mn=mn, mx=mx,
                             islands=_islands(verts, polys))
    return cache


def _aabb_hit(a, b, tol):
    return all(a["mn"][i] - tol <= b["mx"][i] and b["mn"][i] - tol <= a["mx"][i] for i in range(3))


RAY_DIRS = (Vector((1, 0.013, 0.007)).normalized(), Vector((-0.011, 1, 0.017)).normalized(),
            Vector((0.009, -0.015, 1)).normalized())


def _inside(bvh, v, far):
    """Ray-parity inside test (majority of three rays) — independent of normal orientation."""
    votes = 0
    for d in RAY_DIRS:
        hits, origin = 0, v.copy()
        for _ in range(16):
            loc, _, _, dist = bvh.ray_cast(origin, d, far)
            if loc is None:
                break
            hits += 1
            origin = loc + d * 1e-7
        votes += hits % 2
    return votes >= 2


def penetration(a, b, tol):
    """Deepest vertex of `a` lying inside mesh `b` (0 when none)."""
    deepest = 0.0
    far = (b["mx"] - b["mn"]).length * 2 + 1e-3
    for v in a["verts"]:
        if not all(b["mn"][i] + tol <= v[i] <= b["mx"][i] - tol for i in range(3)):
            continue
        loc, normal, _, dist = b["bvh"].find_nearest(v)
        if loc is None or dist <= tol:
            continue
        for bvh, mn, mx in b["islands"]:
            if all(mn[i] <= v[i] <= mx[i] for i in range(3)) and _inside(bvh, v, far):
                deepest = max(deepest, dist)
                break
    return deepest


# Intended interferences: pins and trunnions pass through bores that are not
# modelled as holes, bearings and pulleys ride on pins, and link lugs pivot
# inside bosses and sockets.
ALLOWED = [
    ("Joint Pin", "*"), ("Abduction Pivot Pin", "*"), ("Wrist Gimbal Cross", "Wrist Yoke"),
    ("Wrist Gimbal Cross", "Wrist Pitch Bearing"), ("Wrist Gimbal Cross", "Wrist Yaw Bearing"),
    ("Metacarpal —", "Carpal Frame"), ("Metacarpal — Thumb", "CMC Saddle Socket"),
    ("Thumb Mount Bracket", "CMC Saddle Socket"), ("Thumb Mount Bracket", "Carpal Frame"),
    ("Tendon Lead", "Tendon Spool"), ("Wiring Trunk", "Wiring Harness Manifold"),
    ("Wiring Conduit", "Wiring Harness Manifold"), ("Conduit Clip", "Wiring Conduit"),
    ("Flexor Tendon", "Tendon Anchor"), ("Tendon Pulley", "Joint Pin"),
]


def allowed(a, b):
    for x, y in ALLOWED:
        if (x in a and (y == "*" or y in b)) or (x in b and (y == "*" or y in a)):
            return True
    return False


def check_interference(cache, tol_mm=0.25, cross_chain_only=False, scale=0.001):
    tol = tol_mm * scale
    names = list(cache)
    issues = []
    for i, na in enumerate(names):
        a = cache[na]
        for nb in names[i + 1:]:
            b = cache[nb]
            if not _aabb_hit(a, b, 0):
                continue
            pa, pb = a["obj"]["rh_name"], b["obj"]["rh_name"]
            if cross_chain_only and a["obj"]["rh_assembly"] == b["obj"]["rh_assembly"]:
                continue
            if allowed(pa, pb):
                continue
            depth = max(penetration(a, b, tol), penetration(b, a, tol))
            if depth > tol:
                issues.append((pa, pb, round(depth / scale, 2)))
    return sorted(issues, key=lambda x: -x[2])


def check_floating(cache, tol_mm=0.2, scale=0.001):
    """Contact graph: parts touching within tol are connected. Everything should be one component."""
    tol = tol_mm * scale
    names = list(cache)
    adj = {n: set() for n in names}
    for i, na in enumerate(names):
        a = cache[na]
        for nb in names[i + 1:]:
            b = cache[nb]
            if not _aabb_hit(a, b, tol):
                continue
            touch = bool(a["bvh"].overlap(b["bvh"]))    # intersecting (e.g. pin through bore)
            for src, dst in (() if touch else ((a, b), (b, a))):
                for v in src["verts"]:
                    if not all(dst["mn"][k] - tol <= v[k] <= dst["mx"][k] + tol for k in range(3)):
                        continue
                    loc, _, _, dist = dst["bvh"].find_nearest(v)
                    if loc is not None and dist <= tol:
                        touch = True
                        break
                if touch:
                    break
            if touch:
                adj[na].add(nb)
                adj[nb].add(na)
    seen, comps = set(), []
    for n in names:
        if n in seen:
            continue
        stack, comp = [n], []
        seen.add(n)
        while stack:
            x = stack.pop()
            comp.append(x)
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    return comps


def step_check(ctx):
    scale = hg._S
    reset_pose(ctx)
    t = time.time()
    cache = build_geometry_cache(part_objects())
    floating = check_floating(cache, scale=scale)
    rest = check_interference(cache, scale=scale)
    report = {"rest_interference": rest,
              "components": len(floating),
              "floating_groups": [[cache[n]["obj"]["rh_name"] for n in comp] for comp in floating[1:]],
              "poses": {}}
    log(f"rest: {len(rest)} interferences, {len(floating)} connected components ({time.time() - t:.1f}s)")
    for r in rest[:25]:
        log("   interference", r)
    for comp in report["floating_groups"][:25]:
        log("   floating", comp)
    presets = json.loads(rig()["rh_presets"])
    for name in presets:
        apply_preset(name)
        cache = build_geometry_cache(part_objects())
        issues = check_interference(cache, tol_mm=0.4, scale=scale)
        report["poses"][name] = issues
        log(f"pose {name}: {len(issues)} interferences")
        for r in issues[:12]:
            log("   ", r)
    reset_pose(ctx)
    with open(os.path.join(OUT, "verification.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    return report


# -----------------------------------------------------------------------------
# preview (fast geometry pass)
# -----------------------------------------------------------------------------

def step_preview(ctx, opts):
    setup_workbench()
    d = os.path.join(OUT, "preview")
    os.makedirs(d, exist_ok=True)
    cam = get_cam("RH_CAM_Preview")
    views = {
        "palmar_34": Vector((0.45, 0.85, -0.25)),
        "dorsal_34": Vector((0.35, -0.9, -0.2)),
        "radial": Vector((-1, 0.05, -0.05)),
        "tip_down": Vector((0.1, 0.25, -1.0)),
    }
    reset_pose(ctx)
    objs = part_objects()
    bpy.data.objects["RH_Floor"].hide_render = True
    for name, direction in views.items():
        frame_camera(cam, direction, objs, 1.0, lens=60)
        render(os.path.join(d, f"rest_{name}.png"), cam, (900, 900))
    for preset in (opts.get("preview_poses") or "Fist,Pinch,Point,Power Grip,OK Sign").split(","):
        apply_preset(preset)
        frame_camera(cam, Vector((0.55, 0.8, -0.2)), objs, 1.0, lens=60)
        render(os.path.join(d, f"pose_{preset.replace(' ', '_')}.png"), cam, (700, 700))
    reset_pose(ctx)
    set_state(explode=1.0)
    frame_camera(cam, Vector((0.35, -0.9, -0.2)), objs, 1.0, lens=50)
    render(os.path.join(d, "exploded_dorsal.png"), cam, (1000, 1000))
    reset_pose(ctx)
    bpy.data.objects["RH_Floor"].hide_render = False


# -----------------------------------------------------------------------------
# final stills (Cycles)
# -----------------------------------------------------------------------------

RELAXED = dict(thumb_opposition=0.30, thumb_curl=0.25, spread=0.30, palm_cup=0.15, wrist_pitch=-6.0, wrist_yaw=4.0)
RELAXED_CURL = [0.12, 0.18, 0.24, 0.30, 0.30, 0.30]

HERO_DIR = Vector((0.62, 0.74, -0.18))        # palmar-radial three-quarter, slightly above
DORSAL_DIR = Vector((0.55, -0.80, -0.22))     # dorsal three-quarter
EXPLODE_DIR = Vector((0.30, 0.90, -0.30))     # mostly palmar, from above: chains separate cleanly
LABEL_DIR = None                              # taken from the generator's labeled camera
FORMATS = {"4x5": (1080, 1350), "16x9": (1920, 1080)}


def relaxed(ctx, **extra):
    vals = dict(RELAXED)
    for i, ch in enumerate(ctx.chains):
        vals[f"curl_{ch.key}"] = RELAXED_CURL[min(i, len(RELAXED_CURL) - 1)]
    vals.update(extra)
    set_state(**vals)


def scene_objs_for_framing():
    return part_objects()


def shot(ctx, name, direction, fmt, lens, pct, margin=1.10, extra_pts=()):
    res = FORMATS[fmt]
    cam = get_cam("RH_CAM_Render")
    frame_camera(cam, direction, scene_objs_for_framing(), res[0] / res[1], margin=margin, lens=lens, extra_pts=extra_pts)
    render(os.path.join(REN, f"{name}_{fmt}.png"), cam, res, pct)


def label_points():
    """Label anchors plus their estimated text extents, so framing keeps callouts in shot."""
    cam = bpy.data.objects["RH_CAM_Labeled"]
    right = cam.matrix_world.col[0].xyz.normalized()
    pts = []
    for o in bpy.data.objects:
        if o.name.startswith("RH_Label_") and not o.hide_render:
            side = 1 if o.data.align_x == "LEFT" else -1
            width = len(o.data.body) * o.data.size * 0.62 + abs(o.data.offset_x)
            p = o.matrix_world.translation
            pts += [p, p + right * side * width]
    return pts


def step_stills(ctx, opts):
    setup_cycles(opts["samples"])
    pct = int(opts["pct"])
    only = opts.get("only", "hero,dorsal,exploded,labeled").split(",")
    fmts = opts.get("formats", "4x5,16x9").split(",")
    for fmt in fmts:
        if "hero" in only:
            reset_pose(ctx); relaxed(ctx)
            shot(ctx, "01_hero_assembled", HERO_DIR, fmt, 85, pct)
        if "dorsal" in only:
            reset_pose(ctx); relaxed(ctx)
            shot(ctx, "02_dorsal_assembled", DORSAL_DIR, fmt, 85, pct)
        if "exploded" in only:
            reset_pose(ctx); set_state(explode=1.0, spread=0.0)
            shot(ctx, "03_exploded", EXPLODE_DIR, fmt, 70, pct, margin=1.04)
        if "labeled" in only:
            reset_pose(ctx); set_state(explode=1.0, technical_view=True, show_labels=True, label_detail=0)
            direction = bpy.data.objects["RH_CAM_Labeled"].matrix_world.col[2].xyz * -1
            shot(ctx, "04_labeled_technical", direction, fmt, 55, pct, margin=1.04, extra_pts=label_points())
    reset_pose(ctx)


# -----------------------------------------------------------------------------
# grasp taxonomy
# -----------------------------------------------------------------------------

def caption(cam, text, size=0.018, name="RH_Caption"):
    """Camera-locked caption near the bottom of the frame."""
    ob = bpy.data.objects.get(name)
    if ob is None:
        cu = bpy.data.curves.new(name, "FONT")
        font = hg._font("Inter.woff2")
        if font:
            cu.font = font
        cu.materials.append(bpy.data.materials["RH_label"])
        ob = bpy.data.objects.new(name, cu)
        bpy.data.collections["RH Studio"].objects.link(ob)
    ob.data.body = text
    ob.data.align_x = "CENTER"
    ob.data.size = size
    ob.parent = cam
    ob.matrix_parent_inverse = Matrix.Identity(4)
    d = 0.5
    half_h = cam.data.sensor_width / 2 / cam.data.lens * d
    ob.location = (0, -half_h * 0.86, -d)
    ob.rotation_euler = (0, 0, 0)
    ob.hide_render = False
    return ob


def step_poses(ctx, opts):
    setup_cycles(opts["samples"])
    pct = int(opts["pct"])
    cam = get_cam("RH_CAM_Render")
    files = []
    for name in json.loads(rig()["rh_presets"]):
        reset_pose(ctx)
        apply_preset(name, wrist_pitch=-5.0)
        hand = [o for o in part_objects() if o["rh_assembly"] != "Forearm"]
        frame_camera(cam, Vector((0.62, 0.74, -0.15)), hand, 1.0, margin=1.28, lens=85)
        caption(cam, name.upper(), size=0.02)
        path = os.path.join(REN, f"05_grasp_{name.replace(' ', '_').lower()}.png")
        render(path, cam, (1080, 1080), pct)
        files.append(path)
    bpy.data.objects["RH_Caption"].hide_render = True
    reset_pose(ctx)
    return files


# -----------------------------------------------------------------------------
# orthographic blueprint views with dimension lines
# -----------------------------------------------------------------------------

def _label_mat():
    return bpy.data.materials["RH_label"]


def _poly_curve(name, pts, coll, width):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = width
    cu.bevel_resolution = 0
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for i, p in enumerate(pts):
        sp.points[i].co = (*p, 1.0)
    cu.materials.append(_label_mat())
    ob = bpy.data.objects.new(name, cu)
    coll.objects.link(ob)
    return ob


def _text(coll, body, size, font_name="DejaVuSansMono.woff2", align="CENTER"):
    cu = bpy.data.curves.new("RH_DimText", "FONT")
    cu.body = body
    cu.size = size
    cu.align_x = align
    cu.align_y = "BOTTOM"
    font = hg._font(font_name)
    if font:
        cu.font = font
    cu.materials.append(_label_mat())
    ob = bpy.data.objects.new("RH_DimText", cu)
    coll.objects.link(ob)
    return ob


def make_dimension(coll, p1, p2, offset, fwd, text, S, right, up):
    """Engineering dimension: extension lines, dimension line, arrowheads and value text.

    p1, p2 : measured points in the drawing plane (world)
    offset : vector from the measured feature to the dimension line
    fwd    : camera viewing direction (text faces against it)
    """
    a, b = p1 + offset, p2 + offset
    n = offset.normalized()
    w = 0.22 * S
    _poly_curve("RH_DimExt", [p1 + n * 2.0 * S, a + n * 3.0 * S], coll, w)
    _poly_curve("RH_DimExt", [p2 + n * 2.0 * S, b + n * 3.0 * S], coll, w)
    _poly_curve("RH_DimLine", [a, b], coll, w)
    d = (b - a).normalized()
    side = d.cross(fwd).normalized()
    for tip, direction in ((a, d), (b, -d)):
        base = tip + direction * 4.5 * S
        me = bpy.data.meshes.new("RH_DimArrow")
        me.from_pydata([tuple(tip), tuple(base + side * 1.3 * S), tuple(base - side * 1.3 * S)], [], [(0, 1, 2)])
        me.materials.append(_label_mat())
        coll.objects.link(bpy.data.objects.new("RH_DimArrow", me))
    size = 6.0 * S
    t = _text(coll, text, size)
    # Always readable: baseline runs left-to-right or bottom-to-top on the page,
    # and the text sits on the outward side of the dimension line.
    x_axis = d if (d.dot(right) > 1e-4 or (abs(d.dot(right)) <= 1e-4 and d.dot(up) > 0)) else -d
    y_axis = (-fwd).cross(x_axis).normalized()
    m = Matrix((x_axis, y_axis, -fwd)).transposed().to_4x4()
    m.translation = (a + b) / 2 + y_axis * (1.4 * S if y_axis.dot(n) >= 0 else -(size + 1.4 * S))
    t.matrix_world = m


def _bbox(objs):
    pts = world_points(objs)
    return Vector(map(min, *[tuple(p) for p in pts])), Vector(map(max, *[tuple(p) for p in pts]))


def step_ortho(ctx, opts):
    setup_cycles(max(16, int(opts["samples"]) // 2))
    remove_glare()
    pct = int(opts["pct"])
    S = hg._S
    reset_pose(ctx)
    set_state(technical_view=True)
    bpy.data.objects["RH_Floor"].hide_render = True
    stand = bpy.data.objects["RH_DisplayStand"]
    for fc in stand.animation_data.drivers:
        fc.mute = True
    stand.hide_render = True
    parts = part_objects()
    mn, mx = _bbox(parts)
    grid = bpy.data.materials.get("RH_Grid") or hg._grid_material("RH_Grid")

    def group(pred):
        return _bbox([o for o in parts if pred(o)])
    tip = group(lambda o: o["rh_name"] == f"Distal Phalanx — {ctx.chains[min(1, len(ctx.chains) - 1)].name}")
    first = group(lambda o: o["rh_assembly"] == ctx.chains[0].name)
    last = group(lambda o: o["rh_assembly"] == ctx.chains[-1].name)
    fore = group(lambda o: o["rh_assembly"] == "Forearm")
    palm = group(lambda o: o["rh_name"] == "Carpal Frame")
    V = Vector
    views = {
        # name: (camera forward, camera up, [(p1, p2, offset_mm, label)])
        "front": (V((0, -1, 0)), V((0, 0, 1)), [
            (V((mx.x, 0, mn.z)), V((mx.x, 0, mx.z)), V((16, 0, 0)), "OVERALL LENGTH {:.0f}"),
            (V((last[0].x, 0, 0)), V((last[0].x, 0, tip[1].z)), V((-(last[0].x - mn.x) / S - 16, 0, 0)), "WRIST CENTRE TO TIP {:.0f}"),
            (V((last[0].x, 0, mx.z)), V((first[1].x, 0, mx.z)), V((0, 0, 12)), "FINGER SPAN {:.0f}"),
            (V((fore[0].x, 0, mn.z)), V((fore[1].x, 0, mn.z)), V((0, 0, -12)), "FOREARM {:.0f}"),
            (V((mn.x, 0, palm[0].z)), V((mx.x, 0, palm[0].z)), V((0, 0, -(palm[0].z - mn.z) / S - 30)), "OVERALL WIDTH {:.0f}"),
        ]),
        "side": (V((-1, 0, 0)), V((0, 0, 1)), [
            (V((0, mn.y, mn.z)), V((0, mx.y, mn.z)), V((0, 0, -12)), "DEPTH {:.0f}"),
            (V((0, mn.y, mn.z)), V((0, mn.y, mx.z)), V((0, -16, 0)), "OVERALL LENGTH {:.0f}"),
            (V((0, fore[0].y, fore[1].z)), V((0, fore[1].y, fore[1].z)), V((0, 0, 8)), "{:.0f}"),
        ]),
        "top": (V((0, 0, -1)), V((0, 1, 0)), [
            (V((mn.x, mn.y, 0)), V((mx.x, mn.y, 0)), V((0, -12, 0)), "OVERALL WIDTH {:.0f}"),
            (V((mx.x, mn.y, 0)), V((mx.x, mx.y, 0)), V((14, 0, 0)), "DEPTH {:.0f}"),
        ]),
    }
    files = []
    for view, (fwd, up, dims) in views.items():
        coll = bpy.data.collections.new(f"RH Dims {view}")
        bpy.data.collections["RH Studio"].children.link(coll)
        right = fwd.cross(up).normalized()
        corners = [Vector((x, y, z)) for x in (mn.x, mx.x) for y in (mn.y, mx.y) for z in (mn.z, mx.z)]
        front_depth = max(c.dot(-fwd) for c in corners) + 0.01     # drawing plane just in front of the part
        extra = []
        for p1, p2, off, label in dims:
            lift = lambda p: p + (-fwd) * (front_depth - p.dot(-fwd))
            q1, q2 = lift(p1), lift(p2)
            make_dimension(coll, q1, q2, off * S, fwd, label.format((p2 - p1).length / S) + " mm", S, right, up)
            extra += [q1 + off * S + off.normalized() * 9 * S, q2 + off * S + off.normalized() * 9 * S]
        cam = get_cam(f"RH_CAM_Ortho_{view}", ortho=True)
        frame_camera(cam, fwd, parts, 1.0, margin=1.10, up=up, extra_pts=extra)
        half = cam.data.ortho_scale / 2
        cm = cam.matrix_world
        basis = Matrix((right, up, -fwd)).transposed().to_4x4()
        title = _text(coll, f"{hg.PRODUCT_NAME}  ·  {view.upper()} VIEW  ·  DIMENSIONS IN mm  ·  POSE: OPEN",
                      cam.data.ortho_scale * 0.017, align="LEFT")
        tm = basis.copy()
        tm.translation = cm.translation + fwd * 1.0 - right * half * 0.94 - up * half * 0.95
        title.matrix_world = tm
        me = bpy.data.meshes.new("RH_Sheet")
        h = half * 1.05
        me.from_pydata([(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0)], [], [(0, 1, 2, 3)])
        me.materials.append(grid)
        sheet = bpy.data.objects.new("RH_Sheet", me)
        coll.objects.link(sheet)
        sm = basis.copy()
        sm.translation = cm.translation + fwd * 3.6          # well behind the subject (camera sits 3 m out)
        sheet.matrix_world = sm
        for other in bpy.data.collections:
            if other.name.startswith("RH Dims"):
                for o in other.objects:
                    o.hide_render = other is not coll
        path = os.path.join(REN, f"06_ortho_{view}.png")
        render(path, cam, (1600, 1600), pct)
        files.append(path)
    for other in [c for c in bpy.data.collections if c.name.startswith("RH Dims")]:
        for o in other.objects:
            o.hide_render = True
    for fc in stand.animation_data.drivers:
        fc.mute = False
    bpy.data.objects["RH_Floor"].hide_render = False
    add_glare()
    reset_pose(ctx)
    return files


# -----------------------------------------------------------------------------
# parametric variant, reports, .blend
# -----------------------------------------------------------------------------

VARIANT = {"hand_scale": 1.12, "finger_length_ratio": 1.35, "palm_width_mm": 72.0, "num_fingers": 3}


def step_variant(opts):
    """Second parameter set through the same generator, verified and rendered."""
    default_report = os.path.join(OUT, "verification.json")
    backup = default_report + ".default"
    if os.path.exists(default_report):
        os.replace(default_report, backup)            # keep the default hand's report
    ctx = hg.build(VARIANT)
    step_check(ctx)
    os.replace(default_report, os.path.join(OUT, "verification_variant.json"))
    if os.path.exists(backup):
        os.replace(backup, default_report)
    setup_cycles(opts["samples"])
    relaxed(ctx)
    shot(ctx, "07_variant_3finger_hero", HERO_DIR, "4x5", 85, int(opts["pct"]))
    reset_pose(ctx)
    set_state(explode=1.0)
    shot(ctx, "07_variant_3finger_exploded", EXPLODE_DIR, "4x5", 70, int(opts["pct"]), margin=1.04)
    return hg.build({})


def step_save(ctx):
    reset_pose(ctx)
    bpy.context.scene.camera = bpy.data.objects["RH_CAM_Hero"]
    point_labels_at(bpy.data.objects["RH_CAM_Labeled"])
    bpy.context.scene.render.engine = "CYCLES"
    spec, _ = hg.write_reports(ctx, OUT)
    path = os.path.join(OUT, "tendra_h1.blend")
    bpy.ops.wm.save_as_mainfile(filepath=path, compress=True)
    log("saved", path, f"mass {spec['mass_g']} g, cost ${spec['cost_usd']}")
    return spec


# -----------------------------------------------------------------------------
# glTF / GLB web export
# -----------------------------------------------------------------------------

CLIP_FRAMES = 60


def step_glb(ctx):
    """Bake every interactive control into NLA clips that a web viewer can scrub.

    Custom-property drivers don't survive glTF export. Each control therefore
    becomes its own animation clip over frames 0..60 (0 = slider minimum,
    60 = maximum) that touches only its own bones or nodes. In three.js each
    clip becomes an AnimationAction whose time is set straight from a slider,
    so all the controls combine freely, just like the Blender panel.
    """
    reset_pose(ctx)
    r = rig()
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = 0, CLIP_FRAMES
    parts = part_objects()
    for o in parts:                                   # 1) strip drivers, keep rest state
        if o.animation_data:
            for fc in list(o.animation_data.drivers):
                o.animation_data.drivers.remove(fc)
        o.delta_location = (0, 0, 0)
    for fc in list(r.animation_data.drivers):
        r.animation_data.drivers.remove(fc)
    for pb in r.pose.bones:
        pb.rotation_euler = (0, 0, 0)
        for c in list(pb.constraints):
            pb.constraints.remove(c)

    def push(idb, track_name):
        ad = idb.animation_data
        track = ad.nla_tracks.new()
        track.name = track_name
        track.strips.new(track_name, 0, ad.action)
        ad.action = None

    for o in parts:                                   # 2) explode clip: node translations
        o.animation_data_create()
        base = o.location.copy()
        o.keyframe_insert("location", frame=0)
        o.location = base + Vector(o["rh_explode"])
        o.keyframe_insert("location", frame=CLIP_FRAMES)
        o.location = base
        push(o, "Explode")

    rad = math.radians                                # 3) one pose clip per control
    controls = {}
    for ch in ctx.chains:
        controls[f"Curl_{ch.name.replace(' ', '')}"] = [(f"{ch.key}_prox", 0, rad(hg.CURL_COUPLING["mcp"])),
                                                         (f"{ch.key}_mid", 0, rad(hg.CURL_COUPLING["pip"])),
                                                         (f"{ch.key}_dist", 0, rad(hg.CURL_COUPLING["dip"]))]
    controls["Spread"] = [(f"{ch.key}_abd", 2, rad(ch.row["spread"])) for ch in ctx.chains]
    controls["PalmCup"] = [(f"{ch.key}_meta", 0, rad(ch.row["cup"])) for ch in ctx.chains if ch.row["cup"] > 0]
    controls["ThumbCurl"] = [("thumb_mcp", 0, rad(hg.THUMB_CURL_COUPLING["mcp"])),
                             ("thumb_ip", 0, rad(hg.THUMB_CURL_COUPLING["ip"]))]
    controls["ThumbOpposition"] = [("thumb_cmc", i, rad(hg.THUMB_OPPOSITION[a])) for i, a in enumerate("xyz")]
    p_lo, p_hi = hg.ROM["wrist_pitch"]
    controls["WristPitch"] = [("wrist_pitch", 0, (rad(p_lo), rad(p_hi)))]
    y_lo, y_hi = -hg.ROM["wrist_yaw"][1], -hg.ROM["wrist_yaw"][0]     # UI degrees (+ = radial)
    controls["WristYaw"] = [("wrist_yaw", 2, (-rad(y_lo), -rad(y_hi)))]
    for clip, channels in controls.items():
        for f in range(0, CLIP_FRAMES + 1, 5):
            t = f / CLIP_FRAMES
            per_bone = {}
            for bone, axis, amount in channels:
                e = per_bone.setdefault(bone, [0.0, 0.0, 0.0])
                e[axis] = amount[0] + (amount[1] - amount[0]) * t if isinstance(amount, tuple) else amount * t
            for bone, e in per_bone.items():
                pb = r.pose.bones[bone]
                pb.rotation_euler = e
                pb.keyframe_insert("rotation_euler", frame=f)
        push(r, clip)
        for pb in r.pose.bones:
            pb.rotation_euler = (0, 0, 0)

    for mat in bpy.data.materials:                    # 4) plain PBR materials for glTF
        if not mat.name.startswith("RH_") or not mat.node_tree:
            continue
        nodes = mat.node_tree.nodes
        bsdf = nodes.get("RH_Principled")
        out = next((n for n in nodes if n.type == "OUTPUT_MATERIAL"), None)
        if bsdf and out:
            mat.node_tree.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
            for link in list(bsdf.inputs["Roughness"].links):
                mat.node_tree.links.remove(link)
    for o in parts:                                   # lighter bevels for the browser
        for m in o.modifiers:
            if m.type == "BEVEL":
                m.segments = 1

    for o in bpy.data.objects:
        o.select_set(False)
    for o in parts + [r]:
        o.select_set(True)
    bpy.context.view_layer.objects.active = r
    web = os.path.join(HERE, "web")
    os.makedirs(web, exist_ok=True)
    path = os.path.join(web, "tendra_h1.glb")
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", use_selection=True, export_apply=True,
                              export_extras=True, export_animations=True, export_animation_mode="NLA_TRACKS",
                              export_force_sampling=True, export_yup=True, export_cameras=False, export_lights=False)
    presets = json.loads(r["rh_presets"])
    meta = {"product": hg.PRODUCT_NAME, "clip_frames": CLIP_FRAMES, "fps": scene.render.fps,
            "controls": ["Explode"] + list(controls.keys()),
            "wrist_pitch_range": [p_lo, p_hi], "wrist_yaw_range": [y_lo, y_hi],
            "fingers": [ch.name for ch in ctx.chains], "presets": {}}
    for name, vals in presets.items():
        clip = {f"Curl_{ch.name.replace(' ', '')}": vals[f"curl_{ch.key}"] for ch in ctx.chains}
        clip.update(ThumbCurl=vals["thumb_curl"], ThumbOpposition=vals["thumb_opposition"],
                    Spread=vals["spread"], PalmCup=vals["palm_cup"])
        meta["presets"][name] = clip
    with open(os.path.join(web, "tendra_h1_meta.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=1)
    dg = bpy.context.evaluated_depsgraph_get()
    tris = 0
    for o in parts:
        ev = o.evaluated_get(dg)
        tris += sum(len(p.vertices) - 2 for p in ev.to_mesh().polygons)
        ev.to_mesh_clear()
    log(f"glb written {os.path.getsize(path) / 1e6:.2f} MB, {tris} triangles, {len(controls) + 1} clips")


# -----------------------------------------------------------------------------
# showcase animation: explode -> hold -> reassemble -> grip
# -----------------------------------------------------------------------------

def _key_props(frame, **vals):
    r = rig()
    for k, v in vals.items():
        r[k] = v
        r.keyframe_insert(f'["{k}"]', frame=frame)


def step_anim(ctx, opts):
    sc = bpy.context.scene
    fmt = opts.get("anim_format", "16x9")
    res = FORMATS[fmt]
    engine = opts.get("anim_engine", "BLENDER_EEVEE")
    if engine == "CYCLES":
        setup_cycles(int(opts.get("anim_samples", 16)))
    else:
        sc.render.engine = engine
        for attr, val in (("taa_render_samples", int(opts.get("anim_samples", 32))), ("use_raytracing", False),
                          ("use_shadows", True)):
            try:
                setattr(sc.eevee, attr, val)
            except Exception as exc:
                log("eevee setting skipped:", attr, exc)
        colour_management()
        add_glare()
    reset_pose(ctx)
    total = 240                                   # 10 s at 24 fps
    sc.frame_start, sc.frame_end = 1, total
    sc.render.fps = 24
    chains = ctx.chains
    base = dict(RELAXED, explode=0.0, **{f"curl_{ch.key}": RELAXED_CURL[min(i, 5)] for i, ch in enumerate(chains)})
    flat = dict(base, spread=0.2, thumb_opposition=0.0, thumb_curl=0.0, palm_cup=0.0, wrist_pitch=0.0, wrist_yaw=0.0,
                **{f"curl_{ch.key}": 0.0 for ch in chains})
    presets = json.loads(rig()["rh_presets"])
    power = dict(base, **presets["Power Grip"], wrist_pitch=8.0, wrist_yaw=-4.0)
    pinch = dict(base, **presets["Pinch"], wrist_pitch=-4.0, wrist_yaw=6.0)
    beats = [(1, base), (28, flat), (84, dict(flat, explode=1.0)), (124, dict(flat, explode=1.0)),
             (172, flat), (196, power), (218, pinch), (240, base)]
    for frame, vals in beats:
        _key_props(frame, **vals)
    # Camera framed on the exploded state, then orbiting. (explode is keyframed now, so
    # jump to a fully exploded frame instead of setting the property.)
    sc.frame_set(100)
    bpy.context.view_layer.update()
    bpy.data.objects["RH_Floor"].hide_render = True          # clean floating hero; EEVEE floor gets noisy
    stand = bpy.data.objects["RH_DisplayStand"]
    for fc in stand.animation_data.drivers:
        fc.mute = True
    stand.hide_render = True
    pivot = bpy.data.objects.get("RH_Orbit")
    if pivot is None:
        pivot = bpy.data.objects.new("RH_Orbit", None)
        bpy.data.collections["RH Studio"].objects.link(pivot)
    cam = get_cam("RH_CAM_Anim")
    cam.parent = None
    frame_camera(cam, EXPLODE_DIR, part_objects(), res[0] / res[1], margin=1.16, lens=50)
    world_m = cam.matrix_world.copy()
    lo, hi = _bbox(part_objects())
    centre = (lo + hi) / 2
    fwd = -world_m.col[2].xyz.normalized()
    cam_pos = world_m.translation.copy()
    centre = cam_pos + fwd * (centre - cam_pos).dot(fwd)      # aim point on the view ray
    sc.frame_set(1)
    pivot.location = centre                       # orbit about the exploded assembly's centre
    pivot.rotation_euler = (0, 0, 0)
    cam.parent = pivot
    cam.matrix_parent_inverse = Matrix.Identity(4)
    cam.location = cam_pos - centre
    cam.rotation_euler = world_m.to_euler()
    # Dolly: push in on the assembled hand, pull back while it is exploded.
    sc.frame_set(1)
    lo1, hi1 = _bbox(part_objects())
    assembled_centre = (lo1 + hi1) / 2
    far_local = cam.location.copy()
    near_local = far_local * 0.64
    for frame, near in ((1, True), (28, True), (84, False), (124, False), (172, True), (240, True)):
        cam.location = near_local if near else far_local
        cam.keyframe_insert("location", frame=frame)
        pivot.location = assembled_centre if near else centre
        pivot.keyframe_insert("location", frame=frame)
    pivot.rotation_euler = (0, 0, math.radians(-35))
    pivot.keyframe_insert("rotation_euler", frame=1)
    pivot.rotation_euler = (0, 0, math.radians(35))
    pivot.keyframe_insert("rotation_euler", frame=total)
    sc.camera = cam
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = int(opts["pct"])
    frames = opts.get("frames")
    if frames:                                        # review stills, e.g. --frames 1,120,250
        for f in [int(x) for x in frames.split(",")]:
            sc.frame_set(f)
            render(os.path.join(OUT, "preview", f"anim_{fmt}_{f:03d}.png"), cam, res, int(opts["pct"]))
        return
    anim_dir = os.path.join(OUT, "animation")
    os.makedirs(anim_dir, exist_ok=True)
    sc.render.image_settings.media_type = "VIDEO"
    sc.render.image_settings.file_format = "FFMPEG"
    sc.render.ffmpeg.format = "MPEG4"
    sc.render.ffmpeg.codec = "H264"
    sc.render.ffmpeg.constant_rate_factor = "HIGH"
    sc.render.ffmpeg.ffmpeg_preset = "GOOD"
    sc.render.filepath = os.path.join(anim_dir, f"tendra_h1_showcase_{fmt}_")
    t = time.time()
    bpy.ops.render.render(animation=True)
    log(f"animation {fmt} rendered in {(time.time() - t) / 60:.1f} min")


# -----------------------------------------------------------------------------
# main
# -----------------------------------------------------------------------------

def main():
    opts = argv()
    steps = opts["steps"].split(",")
    if steps == ["all"]:
        steps = ["check", "stills", "poses", "ortho", "variant", "save", "glb"]
    params = json.loads(opts["params"]) if "params" in opts else {}
    ctx = hg.build(params)
    if "check" in steps:
        step_check(ctx)
    if "preview" in steps:
        step_preview(ctx, opts)
    if "stills" in steps:
        step_stills(ctx, opts)
    if "poses" in steps:
        step_poses(ctx, opts)
    if "ortho" in steps:
        step_ortho(ctx, opts)
    if "variant" in steps:
        ctx = step_variant(opts)
    if "save" in steps:
        step_save(ctx)
    if "anim" in steps:
        step_anim(ctx, opts)
    if "glb" in steps:          # destructive (bakes the drivers away), so always last
        step_glb(ctx)


if __name__ == "__main__":
    main()
