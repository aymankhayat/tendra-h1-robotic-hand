"""
===============================================================================
 TENDRA-H1  ·  Parametric tendon-driven robotic hand  ·  Blender generator
===============================================================================

This one script builds the whole hand inside Blender, starting from an empty
scene: geometry, materials, armature, drivers, exploded view, labels, studio,
UI panel and bill of materials. Nothing is modelled by hand. Change a
parameter, run it again, and you get a different (but still mechanically
consistent) hand.

HOW TO RUN
  * Blender GUI  : Scripting tab -> open this file -> Run Script.
                   Open the N-panel in the 3D viewport -> "TENDRA" tab.
  * Command line : blender -b -P hand_generator.py -- [options]
                     --scale 1.0            overall size multiplier
                     --finger-length 1.0    phalanx length multiplier
                     --palm-width 84        palm width at the knuckles (mm)
                     --fingers 4            number of fingers (thumb extra)
                     --save out.blend       write a .blend when done
                     --reports DIR          write BOM / mass / cost / spec JSON

DESIGN SUMMARY
  * Exposed-skeleton ("exoskeleton") construction: every phalanx is a U-channel
    link printed in carbon-fibre nylon. It carries a forked clevis at its distal
    end, and the next link's lug pivots inside that fork on a hardened pin with
    flanged bearings.
  * Tendon actuation: a Dyneema tendon runs down the palmar channel of each
    finger, wraps a grooved pulley at every joint and ends on a spool in the
    forearm actuator pack. The joints of one finger are coupled
    (under-actuated), which is why a single "curl" value drives all of them.
  * The thumb has real opposition geometry. It sits on a separate CMC
    ball-and-socket on the radial side of the carpal frame, and its rest axis
    and pad direction are rotated out of the palm plane, so its flexion axis is
    not parallel to the fingers'.
  * The wrist is a 2-DOF universal joint (pitch + yaw) built from a titanium
    cross with four trunnions.

COORDINATE CONVENTIONS (right hand, SI units = metres)
  +Z  toward the fingertips      -Y  palmar side (palm faces -Y)
  +X  radial side (thumb side)   origin = wrist gimbal centre
  Every link is modelled in its own joint frame:
    local +Y = along the link, local +X = hinge (flexion) axis,
    local +Z = palmar direction, so a positive X rotation curls the finger.
  That frame is also the Blender bone matrix, so pose rotations map 1:1 onto
  mechanical joint angles.

SCRIPT MAP
  0. Parameters, anatomy table, palette
  1. Low-level geometry (2-D profiles -> prisms, cylinders, spheres)
  2. Materials (studio shading + blueprint + isolate dimming, all switchable)
  3. Skeleton layout (joint frames computed from the anatomy table)
  4. Armature: bones, human range-of-motion limits, pose drivers
  5. Part builders: links, joints, tendons, palm, wrist, forearm
  6. Exploded-view drivers
  7. Labels with leader lines
  8. Studio: world, lights, floor, cameras
  9. Bill of materials, mass and cost estimate, spec data
 10. UI panel module (grip presets, click-to-isolate, regenerate)
 11. build() / command-line entry point
"""

import bpy
import bmesh
import json
import math
import os
import sys
from math import radians, pi, sin, cos, sqrt
from mathutils import Matrix, Vector

# =============================================================================
# 0. PARAMETERS, ANATOMY, PALETTE
# =============================================================================

PRODUCT_NAME = "TENDRA-H1"

DEFAULT_PARAMS = {
    "hand_scale": 1.0,            # 1.0 = adult male hand (~190 mm wrist->tip)
    "finger_length_ratio": 1.0,   # multiplies phalanx lengths only
    "palm_width_mm": 84.0,        # knuckle width
    "num_fingers": 4,             # fingers besides the thumb (1..6)
    "explode_distance_mm": 13.0,  # base spacing of the exploded view
    "bevel": True,                # edge bevels (nicer highlights, more polys)
}

# Adult hand proportions (mm), radial -> ulnar. Bone lengths come from
# anthropometric averages (joint centre to joint centre). Widths are
# link widths: slightly slimmer than a real finger, as a robot would be.
#   meta  : CMC -> MCP      prox/mid/dist : phalanx lengths
#   cup   : max CMC flexion for palm cupping (deg)
#   spread: MCP abduction at full spread (deg, + = toward ulnar side)
FINGER_TABLE = [
    dict(name="Index",  meta=64.0, prox=42.0, mid=25.0, dist=21.0, width=16.0, cup=0.0,  spread=-11.0),
    dict(name="Middle", meta=64.0, prox=46.0, mid=28.0, dist=22.0, width=16.5, cup=0.0,  spread=-2.0),
    dict(name="Ring",   meta=59.0, prox=43.0, mid=27.0, dist=21.0, width=15.5, cup=8.0,  spread=8.0),
    dict(name="Little", meta=52.0, prox=34.0, mid=20.0, dist=19.0, width=13.5, cup=15.0, spread=17.0),
]
THUMB = dict(name="Thumb", meta=46.0, prox=33.0, dist=27.0, width=18.0)

# Human range of motion (deg) used for the bone Limit Rotation constraints.
# Hyperextension is capped at 5-15 deg so no pose looks anatomically broken.
ROM = {
    "finger_cmc": (0.0, 15.0),     # palm cupping (ring / little only)
    "finger_abd": (-20.0, 20.0),   # MCP ab/adduction
    "finger_mcp": (-10.0, 90.0),
    "finger_pip": (-5.0, 110.0),
    "finger_dip": (-10.0, 80.0),
    "thumb_cmc_x": (-10.0, 65.0),    # CMC axes are coupled into one opposition sweep,
    "thumb_cmc_y": (-20.0, 55.0),    # so the Euler limits are wider than any single
    "thumb_cmc_z": (-30.0, 75.0),    # anatomical plane

    "thumb_mcp": (-10.0, 60.0),
    "thumb_ip": (-15.0, 80.0),
    "wrist_pitch": (-60.0, 70.0),  # extension / flexion
    "wrist_yaw": (-20.0, 30.0),    # radial / ulnar deviation (bone Z space)
}

# How one 0..1 "curl" value is shared between the coupled joints (deg at 1.0).
CURL_COUPLING = {"mcp": 85.0, "pip": 105.0, "dip": 75.0}
THUMB_CURL_COUPLING = {"mcp": 55.0, "ip": 75.0}
# Thumb opposition: CMC rotation (deg) about local X / Y / Z at opposition = 1.
# Tuned with tune_presets.py so the thumb pad meets the index pad in a pinch.
THUMB_OPPOSITION = {"x": 58.0, "y": 50.0, "z": 70.0}

# Grasp taxonomy presets (values for the rig's custom properties).
# curl lists are radial -> ulnar and get resampled for other finger counts.
GRASP_PRESETS = {
    # Thumb values found by tools/tune_presets.py: zero part interpenetration
    # with the thumb pad as close as possible to each grasp's contact target.
    "Open":        dict(curl=[0.00, 0.00, 0.00, 0.00], thumb_curl=0.00, thumb_opposition=0.00, spread=0.35, palm_cup=0.00),
    "Fist":        dict(curl=[1.00, 1.00, 1.00, 1.00], thumb_curl=0.60, thumb_opposition=0.00, spread=0.00, palm_cup=0.70),
    "Pinch":       dict(curl=[0.55, 0.35, 0.30, 0.25], thumb_curl=0.35, thumb_opposition=0.85, spread=0.10, palm_cup=0.20),
    "Point":       dict(curl=[0.00, 1.00, 1.00, 1.00], thumb_curl=0.85, thumb_opposition=0.10, spread=0.00, palm_cup=0.50),
    "Power Grip":  dict(curl=[0.62, 0.65, 0.68, 0.72], thumb_curl=0.20, thumb_opposition=0.20, spread=0.00, palm_cup=0.60),
    "OK Sign":     dict(curl=[0.55, 0.05, 0.08, 0.12], thumb_curl=0.30, thumb_opposition=0.90, spread=0.60, palm_cup=0.10),
}

# One place to restyle everything. Hex values are sRGB and get converted below.
PALETTE = {
    "accent":        "#19D9FF",   # the ONLY accent colour (emissive cyan)
    "chassis":       "#16181C",   # matte carbon-fibre nylon
    "titanium":      "#5E636B",
    "anodized":      "#2C3037",
    "steel":         "#C4C8CE",
    "black_oxide":   "#1A1B1E",
    "cable":         "#D8DBD5",
    "conduit":       "#25282E",
    "pcb":           "#0C1716",
    "motor":         "#3B4048",
    "backdrop":      "#050608",
    "bp_navy":       "#0A1A2F",   # blueprint background
    "bp_fill":       "#10345C",   # blueprint part fill
    "bp_line":       "#7FE6FF",   # blueprint line colour
    "label":         "#CFF6FF",
}


def srgb(hex_code, alpha=1.0):
    """'#RRGGBB' (sRGB) -> linear RGBA tuple for Blender colour sockets."""
    h = hex_code.lstrip("#")
    out = []
    for i in (0, 2, 4):
        c = int(h[i:i + 2], 16) / 255.0
        out.append(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
    return (*out, alpha)


# Physical and visual definition of every material. The density feeds the
# mass estimate, and `label` is what the BOM and inspection callouts show.
MATERIAL_SPECS = {
    "chassis":     dict(label="PA12-CF (MJF-printed carbon-fibre nylon)", density=1.12,
                        color="chassis", metallic=0.0, roughness=0.62, coat=0.04),
    "titanium":    dict(label="Ti-6Al-4V, CNC machined", density=4.43,
                        color="titanium", metallic=1.0, roughness=0.30, aniso=0.35),
    "anodized":    dict(label="7075-T6 aluminium, hard-anodised", density=2.81,
                        color="anodized", metallic=1.0, roughness=0.38, aniso=0.25),
    "steel":       dict(label="440C stainless, hardened & ground", density=7.75,
                        color="steel", metallic=1.0, roughness=0.24, aniso=0.60),
    "black_oxide": dict(label="12.9 alloy steel, black oxide", density=7.85,
                        color="black_oxide", metallic=1.0, roughness=0.36),
    "cable":       dict(label="UHMWPE (Dyneema) braided tendon", density=0.97,
                        color="cable", metallic=0.0, roughness=0.55),
    "conduit":     dict(label="TPU 95A wiring conduit", density=1.21,
                        color="conduit", metallic=0.0, roughness=0.42, coat=0.3),
    "accent":      dict(label="PMMA light guide + SMD LED", density=1.18,
                        color="black_oxide", metallic=0.0, roughness=0.2, emission=3.2),
    "sensor":      dict(label="Capacitive tactile array, silicone skin", density=1.30,
                        color="black_oxide", metallic=0.0, roughness=0.35, emission=1.6),
    "pcb":         dict(label="FR-4 4-layer PCB, assembled", density=1.85,
                        color="pcb", metallic=0.0, roughness=0.45, coat=0.5),
    "motor":       dict(label="Coreless DC gearmotor + magnetic encoder", density=None,
                        color="motor", metallic=1.0, roughness=0.34, aniso=0.5),
}

# Scale factor (metres per millimetre) of the build in progress. It is set
# once in build() so that every dimension below can be written in mm.
_S = 0.001


def mm(value):
    return value * _S


# =============================================================================
# 1. LOW-LEVEL GEOMETRY
#    Parts are prisms: a 2-D profile extruded along one axis, or a profile
#    with a matching hole. Rounded 2-D profiles give machined-looking parts
#    and keep polycounts low enough for the web export.
# =============================================================================

# Axis mappings for prism(): (u index, v index, extrusion index) in xyz.
AX_X = (1, 2, 0)   # profile in YZ, extruded along X  (side plates, pins)
AX_Y = (2, 0, 1)   # profile in ZX, extruded along Y
AX_Z = (0, 1, 2)   # profile in XY, extruded along Z  (tabs, spools)
AX_XZ_Y = (0, 2, 1)  # profile in XZ, extruded along Y (palm plates)


def arc(cu, cv, r, a0, a1, n):
    return [(cu + r * cos(a0 + (a1 - a0) * i / n), cv + r * sin(a0 + (a1 - a0) * i / n))
            for i in range(n + 1)]


def circle(cu, cv, r, n=24):
    return [(cu + r * cos(2 * pi * i / n), cv + r * sin(2 * pi * i / n)) for i in range(n)]


def stadium(u0, u1, r, n=10, round0=True, round1=True):
    """Slot outline between centres (u0, 0) and (u1, 0), either end round or square."""
    pts = arc(u1, 0, r, -pi / 2, pi / 2, n) if round1 else [(u1, -r), (u1, r)]
    pts += arc(u0, 0, r, pi / 2, 3 * pi / 2, n) if round0 else [(u0, r), (u0, -r)]
    return pts


def rounded_rect(cu, cv, hu, hv, r, n=4):
    r = min(r, hu * 0.99, hv * 0.99)
    pts = []
    for (sx, sy, a) in ((1, -1, -pi / 2), (1, 1, 0.0), (-1, 1, pi / 2), (-1, -1, pi)):
        pts += arc(cu + sx * (hu - r), cv + sy * (hv - r), r, a, a + pi / 2, n)
    return pts


def clip(poly, axis, limit, keep="le"):
    """Clip a 2-D polygon against u (axis 0) or v (axis 1) <= / >= limit."""
    inside = (lambda p: p[axis] <= limit + 1e-12) if keep == "le" else (lambda p: p[axis] >= limit - 1e-12)
    out = []
    for i, cur in enumerate(poly):
        prev = poly[i - 1]
        if inside(cur):
            if not inside(prev):
                out.append(_intersect(prev, cur, axis, limit))
            out.append(cur)
        elif inside(prev):
            out.append(_intersect(prev, cur, axis, limit))
    return out


def _intersect(a, b, axis, limit):
    t = (limit - a[axis]) / (b[axis] - a[axis])
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def resample(poly, n):
    """Resample a closed polygon to n points by arc length (used to pair rings)."""
    segs = [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]
    lens = [sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) for a, b in segs]
    total = sum(lens)
    out, acc, k = [], 0.0, 0
    for i in range(n):
        target = total * i / n
        while acc + lens[k] < target and k < len(segs) - 1:
            acc += lens[k]
            k += 1
        a, b = segs[k]
        t = (target - acc) / lens[k] if lens[k] > 0 else 0.0
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def _place(u, v, w, axes):
    p = [0.0, 0.0, 0.0]
    p[axes[0]], p[axes[1]], p[axes[2]] = u, v, w
    return p


def prism(poly, axes, w0, w1):
    """Extrude a closed 2-D polygon between w0 and w1. Returns (verts, faces)."""
    n = len(poly)
    verts = [_place(u, v, w0, axes) for u, v in poly] + [_place(u, v, w1, axes) for u, v in poly]
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return verts, faces


def prism_hole(outer, inner, axes, w0, w1):
    """Extrude a profile with one through-hole. `outer` and `inner` need the same point count."""
    n = len(outer)
    assert len(inner) == n
    V = []
    for w in (w0, w1):
        V += [_place(u, v, w, axes) for u, v in outer]
        V += [_place(u, v, w, axes) for u, v in inner]
    o0, i0, o1, i1 = 0, n, 2 * n, 3 * n
    F = []
    for k in range(n):
        j = (k + 1) % n
        F.append([o0 + k, i0 + k, i0 + j, o0 + j])      # cap at w0
        F.append([o1 + k, o1 + j, i1 + j, i1 + k])      # cap at w1
        F.append([o0 + k, o0 + j, o1 + j, o1 + k])      # outer wall
        F.append([i0 + k, i1 + k, i1 + j, i0 + j])      # inner wall
    return V, F


def cylinder(r, w0, w1, axis="x", at=(0.0, 0.0, 0.0), n=24, r_in=None):
    """Cylinder (or tube when r_in is set) along a principal axis through `at`."""
    axes = {"x": AX_X, "y": AX_Y, "z": AX_Z}[axis]
    cu, cv = at[axes[0]], at[axes[1]]
    if r_in:
        return prism_hole(circle(cu, cv, r, n), circle(cu, cv, r_in, n), axes, w0, w1)
    return prism(circle(cu, cv, r, n), axes, w0, w1)


def box(x0, x1, y0, y1, z0, z1, r=0.0):
    """Axis-aligned box, optionally rounded around its Z edges."""
    if r > 0:
        return prism(rounded_rect((x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2, r), AX_Z, z0, z1)
    return prism([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], AX_Z, z0, z1)


def uv_sphere(r, rings=12, segs=24):
    verts = [(0, 0, -r)]
    for i in range(1, rings):
        phi = -pi / 2 + pi * i / rings
        verts += [(r * cos(phi) * cos(2 * pi * j / segs), r * cos(phi) * sin(2 * pi * j / segs), r * sin(phi))
                  for j in range(segs)]
    verts.append((0, 0, r))
    top = len(verts) - 1
    faces = [[0, 1 + (j + 1) % segs, 1 + j] for j in range(segs)]
    for i in range(rings - 2):
        a, b = 1 + i * segs, 1 + (i + 1) * segs
        faces += [[a + j, a + (j + 1) % segs, b + (j + 1) % segs, b + j] for j in range(segs)]
    last = 1 + (rings - 2) * segs
    faces += [[last + j, last + (j + 1) % segs, top] for j in range(segs)]
    return verts, faces


def spherical_cup(r_in, r_out, polar_max, rings=8, segs=24):
    """Closed shell of a sphere cap around local +Z, from the pole to polar_max."""
    V, F = [], []

    def ring(r, phi):
        return [(r * sin(phi) * cos(2 * pi * j / segs), r * sin(phi) * sin(2 * pi * j / segs), r * cos(phi))
                for j in range(segs)]
    # Outer and inner surfaces as ring stacks with a tiny polar hole (keeps quads).
    phis = [radians(2.0) + (polar_max - radians(2.0)) * i / rings for i in range(rings + 1)]
    for r in (r_out, r_in):
        for phi in phis:
            V += ring(r, phi)
    stack = rings + 1

    def idx(surface, i, j):
        return surface * stack * segs + i * segs + (j % segs)
    for s in (0, 1):
        for i in range(rings):
            for j in range(segs):
                F.append([idx(s, i, j), idx(s, i, j + 1), idx(s, i + 1, j + 1), idx(s, i + 1, j)])
    for i in (0, rings):   # join outer and inner at both edges
        for j in range(segs):
            F.append([idx(0, i, j), idx(1, i, j), idx(1, i, j + 1), idx(0, i, j + 1)])
    return V, F


class MeshBuilder:
    """Collects several primitives (optionally transformed) into one mesh."""

    def __init__(self):
        self.verts, self.faces = [], []

    def add(self, geo, matrix=None):
        verts, faces = geo
        base = len(self.verts)
        if matrix is not None:
            verts = [matrix @ Vector(p) for p in verts]
        self.verts.extend(tuple(p) for p in verts)
        self.faces.extend([i + base for i in f] for f in faces)
        return self

    def tube(self, p0, p1, r, n=12):
        """Cylinder between two arbitrary points (tendons, conduits, brackets)."""
        p0, p1 = Vector(p0), Vector(p1)
        d = p1 - p0
        rot = d.normalized().to_track_quat("Z", "Y").to_matrix().to_4x4()
        return self.add(prism(circle(0, 0, r, n), AX_Z, 0.0, d.length), Matrix.Translation(p0) @ rot)


def make_frame(origin, y_axis, z_hint):
    """Orthonormal joint frame: +Y along the link, +Z toward z_hint, X = Y x Z."""
    y = Vector(y_axis).normalized()
    zh = Vector(z_hint)
    z = (zh - y * zh.dot(y)).normalized()
    x = y.cross(z)
    m = Matrix((x, y, z)).transposed().to_4x4()
    m.translation = Vector(origin)
    return m


# =============================================================================
# 2. MATERIALS
#    Every material is  Principled -> [Isolate dim mix] -> [Technical mix] -> out
#    so that "Technical View" and click-to-isolate work in every shader
#    without swapping materials.
# =============================================================================

def _socket(sockets, identifier):
    for s in sockets:
        if s.identifier == identifier:
            return s
    raise KeyError(identifier)


def _new_node_material(name):
    mat = bpy.data.materials.new(name)
    if mat.node_tree is None:           # Blender < 5 needs use_nodes explicitly
        mat.use_nodes = True
    mat.node_tree.nodes.clear()
    return mat


def _blueprint_group():
    """Shared node group: navy translucent fill + cyan wireframe edges."""
    ng = bpy.data.node_groups.get("RH_Blueprint")
    if ng:
        return ng
    ng = bpy.data.node_groups.new("RH_Blueprint", "ShaderNodeTree")
    ng.interface.new_socket("Shader", in_out="OUTPUT", socket_type="NodeSocketShader")
    N, L = ng.nodes, ng.links
    out = N.new("NodeGroupOutput")
    wire = N.new("ShaderNodeWireframe")
    wire.use_pixel_size = True
    wire.inputs["Size"].default_value = 1.1
    facing = N.new("ShaderNodeLayerWeight")
    facing.inputs["Blend"].default_value = 0.25
    edge = N.new("ShaderNodeMath")
    edge.operation = "POWER"
    edge.inputs[1].default_value = 2.5
    L.new(facing.outputs["Facing"], edge.inputs[0])
    line = N.new("ShaderNodeMath")
    line.operation = "MAXIMUM"
    L.new(wire.outputs["Fac"], line.inputs[0])
    L.new(edge.outputs[0], line.inputs[1])
    mix = N.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    _socket(mix.inputs, "A_Color").default_value = srgb(PALETTE["bp_fill"])
    _socket(mix.inputs, "B_Color").default_value = srgb(PALETTE["bp_line"])
    L.new(line.outputs[0], _socket(mix.inputs, "Factor_Float"))
    emit = N.new("ShaderNodeEmission")
    emit.inputs["Strength"].default_value = 1.4
    L.new(_socket(mix.outputs, "Result_Color"), emit.inputs["Color"])
    alpha = N.new("ShaderNodeMath")
    alpha.operation = "MAXIMUM"
    alpha.inputs[1].default_value = 0.35      # opacity of the blueprint fill
    L.new(line.outputs[0], alpha.inputs[0])
    transp = N.new("ShaderNodeBsdfTransparent")
    final = N.new("ShaderNodeMixShader")
    L.new(alpha.outputs[0], final.inputs[0])
    L.new(transp.outputs[0], final.inputs[1])
    L.new(emit.outputs[0], final.inputs[2])
    L.new(final.outputs[0], out.inputs[0])
    return ng


def add_switch_chain(mat, surface_socket, rig):
    """Insert the isolate-dim and technical-view mixes between a shader and the output."""
    N, L = mat.node_tree.nodes, mat.node_tree.links
    out = N.new("ShaderNodeOutputMaterial")
    # Isolate: object custom property "dim" (0/1) fades the part to a ghost.
    attr = N.new("ShaderNodeAttribute")
    attr.attribute_type = "OBJECT"
    attr.attribute_name = "dim"
    ghost_t = N.new("ShaderNodeBsdfTransparent")
    ghost_e = N.new("ShaderNodeEmission")
    ghost_e.inputs["Color"].default_value = srgb("#0F2233")
    ghost = N.new("ShaderNodeMixShader")
    ghost.inputs[0].default_value = 0.12
    L.new(ghost_t.outputs[0], ghost.inputs[1])
    L.new(ghost_e.outputs[0], ghost.inputs[2])
    dim = N.new("ShaderNodeMixShader")
    dim.name = "RH_DimMix"
    L.new(attr.outputs["Fac"], dim.inputs[0])
    L.new(surface_socket, dim.inputs[1])
    L.new(ghost.outputs[0], dim.inputs[2])
    # Technical view: driven by the rig's "technical_view" property.
    bp = N.new("ShaderNodeGroup")
    bp.node_tree = _blueprint_group()
    tech = N.new("ShaderNodeMixShader")
    tech.name = "RH_TechMix"
    L.new(dim.outputs[0], tech.inputs[1])
    L.new(bp.outputs[0], tech.inputs[2])
    L.new(tech.outputs[0], out.inputs["Surface"])
    add_driver(tech.inputs[0], "default_value", -1, "t", {"t": (rig, '["technical_view"]')},
               owner=mat.node_tree)
    return out


def build_materials(ctx):
    mats = {}
    for key, spec in MATERIAL_SPECS.items():
        mat = _new_node_material(f"RH_{key}")
        N, L = mat.node_tree.nodes, mat.node_tree.links
        bsdf = N.new("ShaderNodeBsdfPrincipled")
        bsdf.name = "RH_Principled"
        bsdf.inputs["Base Color"].default_value = srgb(PALETTE[spec["color"]])
        bsdf.inputs["Metallic"].default_value = spec["metallic"]
        bsdf.inputs["Roughness"].default_value = spec["roughness"]
        if spec.get("coat"):
            bsdf.inputs["Coat Weight"].default_value = spec["coat"]
            bsdf.inputs["Coat Roughness"].default_value = 0.3
        if spec.get("aniso"):
            bsdf.inputs["Anisotropic"].default_value = spec["aniso"]
        if spec.get("emission"):
            bsdf.inputs["Emission Color"].default_value = srgb(PALETTE["accent"])
            bsdf.inputs["Emission Strength"].default_value = spec["emission"]
        if key == "chassis":
            # Subtle roughness breakup so large dark surfaces don't look like plastic toys.
            noise = N.new("ShaderNodeTexNoise")
            noise.inputs["Scale"].default_value = 900.0
            ramp = N.new("ShaderNodeMapRange")
            ramp.inputs["To Min"].default_value = 0.48
            ramp.inputs["To Max"].default_value = 0.68
            L.new(noise.outputs["Fac"], ramp.inputs["Value"])
            L.new(ramp.outputs["Result"], bsdf.inputs["Roughness"])
        add_switch_chain(mat, bsdf.outputs["BSDF"], ctx.rig)
        # Viewport (Solid mode) colours follow the same palette.
        mat.diffuse_color = srgb(PALETTE["accent"]) if spec.get("emission") else srgb(PALETTE[spec["color"]])
        mat.metallic = spec["metallic"]
        mat.roughness = spec["roughness"]
        mats[key] = mat
    # Label / leader material: flat emissive, readable in both views.
    lab = _new_node_material("RH_label")
    e = lab.node_tree.nodes.new("ShaderNodeEmission")
    e.inputs["Color"].default_value = srgb(PALETTE["label"])
    e.inputs["Strength"].default_value = 2.5
    o = lab.node_tree.nodes.new("ShaderNodeOutputMaterial")
    lab.node_tree.links.new(e.outputs[0], o.inputs["Surface"])
    mats["label"] = lab
    lead = _new_node_material("RH_leader")
    e = lead.node_tree.nodes.new("ShaderNodeEmission")
    e.inputs["Color"].default_value = srgb(PALETTE["accent"])
    e.inputs["Strength"].default_value = 3.0
    o = lead.node_tree.nodes.new("ShaderNodeOutputMaterial")
    lead.node_tree.links.new(e.outputs[0], o.inputs["Surface"])
    mats["leader"] = lead
    ctx.mats = mats


# =============================================================================
# 3. SKELETON LAYOUT
#    Pure maths: joint frames for every link, taken from the anatomy table
#    and the parameters. Geometry, bones and exploded vectors all read this.
# =============================================================================

CARPAL_Z0, CARPAL_Z1 = 14.0, 34.0      # carpal frame block (mm above wrist centre)
CMC_Z = 30.0                           # finger metacarpal pivots
PIN_R, BEAR_T, PLATE_T, GAP = 1.5, 1.4, 2.0, 0.5
BRIDGE_T, CABLE_R, CONDUIT_R = 1.6, 0.5, 1.1


class Chain:
    """A kinematic chain (one finger or the thumb) and its per-link data."""

    def __init__(self, key, name, index):
        self.key, self.name, self.index = key, name, index
        self.links = []        # dicts: name, bone, frame, length, R, depth
        self.dir = Vector((0, 0, 1))
        self.lat = Vector((0, 0, 0))


def _lerp_table(t):
    """Interpolate FINGER_TABLE for any finger count (t=0 radial ... 1 ulnar)."""
    pos = t * (len(FINGER_TABLE) - 1)
    i = min(int(pos), len(FINGER_TABLE) - 2)
    f = pos - i
    a, b = FINGER_TABLE[i], FINGER_TABLE[i + 1]
    row = {k: a[k] + (b[k] - a[k]) * f for k in a if k != "name"}
    return row


def _resample_list(values, n):
    if n == len(values):
        return list(values)
    out = []
    for i in range(n):
        pos = (i / (n - 1) if n > 1 else 0.33) * (len(values) - 1)
        j = min(int(pos), len(values) - 2)
        f = pos - j
        out.append(values[j] + (values[j + 1] - values[j]) * f)
    return out


def compute_layout(ctx):
    p = ctx.params
    n = int(p["num_fingers"])
    pw = p["palm_width_mm"]
    flen = p["finger_length_ratio"]
    spacing = pw / n
    ctx.carpal_hw = pw * 0.36
    ctx.chains = []
    palmar = Vector((0, -1, 0))
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.33
        row = _lerp_table(t)
        name = FINGER_TABLE[i]["name"] if n == 4 else f"Finger {i + 1}"
        ch = Chain(name.lower().replace(" ", ""), name, i)
        width = min(row["width"], spacing - 2.5)
        R1 = 0.40 * width
        ch.dims = dict(width=width, w_out=0.95 * width, R=[R1, R1, 0.92 * R1, 0.84 * R1])
        ch.dims["w_in"] = ch.dims["w_out"] - 2 * (GAP + PLATE_T)
        ch.dims["slot"] = max(2.4, 3.0 * width / 16.0)
        ch.dims["cable_z"] = ch.dims["R"][3] - 1.5          # constant along the finger
        ch.row = row
        x_mcp = pw / 2 - spacing * (i + 0.5)
        x_cmc = x_mcp * 0.62
        dx = x_mcp - x_cmc
        dz = sqrt(max(row["meta"] ** 2 - dx ** 2, 1.0))
        cmc = Vector((x_cmc, 0, CMC_Z))
        mcp = Vector((x_mcp, 0, CMC_Z + dz))
        ch.dir = (mcp - cmc).normalized()
        ch.lat = Vector((x_mcp / (pw / 2) * 1.35 * (n - 1) / 3.0 if n > 1 else 0, 0, 0))
        pivot = mcp - ch.dir * 1.9 * R1
        lens = [row["prox"] * flen, row["mid"] * flen, row["dist"] * flen]
        # (link name, bone, origin, length, radius, depth)
        specs = [("Metacarpal", f"{ch.key}_meta", cmc, (pivot - cmc).length, R1, 1.0),
                 ("MCP Abduction Yoke", f"{ch.key}_abd", pivot, 1.9 * R1, R1, 1.5),
                 ("Proximal Phalanx", f"{ch.key}_prox", mcp, lens[0], R1, 2.0),
                 ("Middle Phalanx", f"{ch.key}_mid", mcp + ch.dir * lens[0], lens[1], ch.dims["R"][2], 3.0),
                 ("Distal Phalanx", f"{ch.key}_dist", mcp + ch.dir * (lens[0] + lens[1]), lens[2], ch.dims["R"][3], 4.0)]
        for lname, bone, origin, length, R, depth in specs:
            ch.links.append(dict(name=lname, bone=bone, frame=make_frame(origin * _S, ch.dir, palmar),
                                 length=length, R=R, depth=depth))
        ctx.chains.append(ch)

    # Thumb: its own CMC on the radial side of the carpal frame. The rest axis
    # points up/out and palmar, and the pad faces the index finger, which puts
    # its flexion axis at roughly 60 deg to the fingers' (true opposition).
    th = Chain("thumb", "Thumb", n)
    width = THUMB["width"]
    R1 = 0.39 * width
    th.dims = dict(width=width, w_out=0.95 * width, R=[R1, 0.92 * R1, 0.84 * R1])
    th.dims["w_in"] = th.dims["w_out"] - 2 * (GAP + PLATE_T)
    th.dims["slot"] = 3.2
    th.dims["cable_z"] = th.dims["R"][2] - 1.5
    th.dims["ball_r"] = 0.95 * R1
    th.cmc = Vector((ctx.carpal_hw + 9.0, -12.0, 22.0))
    th.dir = Vector((0.50, -0.30, 0.81)).normalized()
    th.pad = Vector((-0.95, -0.30, 0.0))
    th.lat = Vector((1.0, -0.2, -0.3)).normalized() * 0.9
    lens = [THUMB["meta"], THUMB["prox"] * flen, THUMB["dist"] * flen]
    origin = th.cmc.copy()
    for (lname, bone, length, R, depth) in (("Metacarpal", "thumb_cmc", lens[0], th.dims["R"][0], 1.0),
                                            ("Proximal Phalanx", "thumb_mcp", lens[1], th.dims["R"][1], 2.0),
                                            ("Distal Phalanx", "thumb_ip", lens[2], th.dims["R"][2], 3.0)):
        th.links.append(dict(name=lname, bone=bone, frame=make_frame(origin * _S, th.dir, th.pad),
                             length=length, R=R, depth=depth))
        origin = origin + th.dir * length
    ctx.thumb = th


# =============================================================================
# 4. ARMATURE, RANGE-OF-MOTION LIMITS AND POSE DRIVERS
# =============================================================================

def add_driver(owner_struct, path, index, expression, variables, owner=None):
    """Attach a scripted driver. variables = {name: (id_block, data_path)}.

    The expressions are kept inside Blender's "simple expression" subset, so
    they are evaluated natively and keep working even with Python
    auto-execution turned off.
    """
    fc = owner_struct.driver_add(path, index) if index >= 0 else owner_struct.driver_add(path)
    drv = fc.driver
    drv.type = "SCRIPTED"
    drv.expression = expression
    for name, (idb, data_path) in variables.items():
        var = drv.variables.new()
        var.name = name
        var.type = "SINGLE_PROP"
        var.targets[0].id_type = "OBJECT" if isinstance(idb, bpy.types.Object) else "SCENE"
        var.targets[0].id = idb
        var.targets[0].data_path = data_path
    return fc


def add_prop(idb, name, default, lo=None, hi=None, desc=""):
    idb[name] = default
    ui = idb.id_properties_ui(name)
    kwargs = {"description": desc}
    if lo is not None and not isinstance(default, bool):
        kwargs.update(min=lo, max=hi, soft_min=lo, soft_max=hi)
    ui.update(**kwargs)


def build_armature(ctx):
    arm = bpy.data.armatures.new("RH_Rig")
    rig = bpy.data.objects.new("RH_Rig", arm)
    ctx.coll["rig"].objects.link(rig)
    rig.show_in_front = True
    arm.display_type = "STICK"
    ctx.rig = rig

    # Bone list: (name, parent, rest frame (mm-scaled already), length)
    up = Vector((0, 0, 1))
    palmar = Vector((0, -1, 0))
    bones = [("forearm", None, make_frame((0, 0, mm(-108)), up, palmar), mm(90)),
             ("wrist_pitch", "forearm", make_frame((0, 0, 0), up, palmar), mm(10)),
             ("wrist_yaw", "wrist_pitch", make_frame((0, 0, 0), up, palmar), mm(10)),
             ("palm", "wrist_yaw", make_frame((0, 0, 0), up, palmar), mm(CARPAL_Z1))]
    for ch in ctx.chains:
        parent = "palm"
        for link in ch.links:
            bones.append((link["bone"], parent, link["frame"], mm(link["length"])))
            parent = link["bone"]
    parent = "palm"
    for link in ctx.thumb.links:
        bones.append((link["bone"], parent, link["frame"], mm(link["length"])))
        parent = link["bone"]

    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode="EDIT")
    for name, parent, frame, length in bones:
        eb = arm.edit_bones.new(name)
        eb.head = (0, 0, 0)
        eb.tail = (0, length, 0)
        eb.matrix = frame
        if parent:
            eb.parent = arm.edit_bones[parent]
            eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    ctx.bone_rest = {b.name: b.matrix_local.copy() for b in arm.bones}

    # ---- control properties (the rig object is the single control surface) ----
    add_prop(rig, "explode", 0.0, 0.0, 1.0, "Exploded-view factor")
    add_prop(rig, "show_labels", False, desc="Show part callouts with leader lines")
    add_prop(rig, "label_detail", 0, 0, 1, "0 = key parts, 1 = every major part")
    add_prop(rig, "technical_view", False, desc="Blueprint / wireframe presentation shading")
    add_prop(rig, "isolate", False, desc="Click-to-isolate inspection mode")
    for ch in ctx.chains:
        add_prop(rig, f"curl_{ch.key}", 0.0, 0.0, 1.0, f"{ch.name} flexion (coupled MCP/PIP/DIP)")
    add_prop(rig, "thumb_curl", 0.0, 0.0, 1.0, "Thumb MCP + IP flexion")
    add_prop(rig, "thumb_opposition", 0.0, 0.0, 1.0, "Thumb CMC opposition")
    add_prop(rig, "spread", 0.0, 0.0, 1.0, "Finger abduction (spread)")
    add_prop(rig, "palm_cup", 0.0, 0.0, 1.0, "Ring/little metacarpal cupping")
    add_prop(rig, "wrist_pitch", 0.0, ROM["wrist_pitch"][0], ROM["wrist_pitch"][1], "Wrist flexion (+) / extension (-), deg")
    add_prop(rig, "wrist_yaw", 0.0, -ROM["wrist_yaw"][1], -ROM["wrist_yaw"][0], "Wrist radial (+) / ulnar (-) deviation, deg")

    # Presets resampled to this finger count and stored on the rig (UI + web export).
    presets = {}
    for pname, pv in GRASP_PRESETS.items():
        vals = {f"curl_{ch.key}": round(c, 3) for ch, c in zip(ctx.chains, _resample_list(pv["curl"], len(ctx.chains)))}
        vals.update({k: pv[k] for k in ("thumb_curl", "thumb_opposition", "spread", "palm_cup")})
        presets[pname] = vals
    rig["rh_presets"] = json.dumps(presets)
    rig["rh_product"] = PRODUCT_NAME
    for k, v in ctx.params.items():
        rig[f"gen_{k}"] = v

    # ---- pose drivers + Limit Rotation constraints ----
    def pose(bone_name, rom_x=None, rom_y=None, rom_z=None):
        pb = rig.pose.bones[bone_name]
        pb.rotation_mode = "XYZ"
        pb.lock_location = (True, True, True)
        pb.lock_rotation = (rom_x is None, rom_y is None, rom_z is None)
        pb.lock_scale = (True, True, True)
        con = pb.constraints.new("LIMIT_ROTATION")
        con.owner_space = "LOCAL"
        for axis, rom in (("x", rom_x), ("y", rom_y), ("z", rom_z)):
            setattr(con, f"use_limit_{axis}", True)
            lo, hi = rom if rom else (0.0, 0.0)
            setattr(con, f"min_{axis}", radians(lo))
            setattr(con, f"max_{axis}", radians(hi))
        return pb

    def drive_rot(bone_name, axis_index, expr, prop):
        add_driver(rig, f'pose.bones["{bone_name}"].rotation_euler', axis_index, expr,
                   {"v": (rig, f'["{prop}"]')})

    pose("forearm")
    pose("palm")
    pose("wrist_pitch", rom_x=ROM["wrist_pitch"])
    pose("wrist_yaw", rom_z=ROM["wrist_yaw"])
    drive_rot("wrist_pitch", 0, "v*0.0174533", "wrist_pitch")
    drive_rot("wrist_yaw", 2, "-v*0.0174533", "wrist_yaw")

    for ch in ctx.chains:
        k = ch.key
        pose(f"{k}_meta", rom_x=ROM["finger_cmc"])
        pose(f"{k}_abd", rom_z=ROM["finger_abd"])
        pose(f"{k}_prox", rom_x=ROM["finger_mcp"])
        pose(f"{k}_mid", rom_x=ROM["finger_pip"])
        pose(f"{k}_dist", rom_x=ROM["finger_dip"])
        drive_rot(f"{k}_meta", 0, f"v*{radians(ch.row['cup']):.6f}", "palm_cup")
        drive_rot(f"{k}_abd", 2, f"v*{radians(ch.row['spread']):.6f}", "spread")
        drive_rot(f"{k}_prox", 0, f"v*{radians(CURL_COUPLING['mcp']):.6f}", f"curl_{k}")
        drive_rot(f"{k}_mid", 0, f"v*{radians(CURL_COUPLING['pip']):.6f}", f"curl_{k}")
        drive_rot(f"{k}_dist", 0, f"v*{radians(CURL_COUPLING['dip']):.6f}", f"curl_{k}")

    pose("thumb_cmc", rom_x=ROM["thumb_cmc_x"], rom_y=ROM["thumb_cmc_y"], rom_z=ROM["thumb_cmc_z"])
    pose("thumb_mcp", rom_x=ROM["thumb_mcp"])
    pose("thumb_ip", rom_x=ROM["thumb_ip"])
    for axis_index, axis in enumerate("xyz"):
        drive_rot("thumb_cmc", axis_index, f"v*{radians(THUMB_OPPOSITION[axis]):.6f}", "thumb_opposition")
    drive_rot("thumb_mcp", 0, f"v*{radians(THUMB_CURL_COUPLING['mcp']):.6f}", "thumb_curl")
    drive_rot("thumb_ip", 0, f"v*{radians(THUMB_CURL_COUPLING['ip']):.6f}", "thumb_curl")


# =============================================================================
# 5. PART BUILDERS
# =============================================================================

class Part:
    """Registry record for one physical part (one Blender object)."""

    def __init__(self, obj, name, bom, material, assembly, explode, purchased=False,
                 unit_cost=None, catalog_mass_g=None, label_level=None, dof=None):
        self.obj, self.name, self.bom, self.material = obj, name, bom, material
        self.assembly, self.explode, self.purchased = assembly, explode, purchased
        self.unit_cost, self.catalog_mass_g = unit_cost, catalog_mass_g
        self.label_level, self.dof = label_level, dof


def add_part(ctx, name, mb, frame, material, bone, assembly, explode, bom,
             bevel=0.35, smooth_angle=35.0, **meta):
    """Create a mesh object in `frame`, parent it rigidly to `bone`, register it."""
    safe = "RH_" + name.replace(" — ", "_").replace(" ", "_").replace("/", "-")
    me = bpy.data.meshes.new(safe)
    me.from_pydata(mb.verts, [], mb.faces)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    for poly in me.polygons:
        poly.use_smooth = True
    me.set_sharp_from_angle(angle=radians(smooth_angle))
    me.materials.append(ctx.mats[material])

    obj = bpy.data.objects.new(safe, me)
    ctx.assembly_collection(assembly).objects.link(obj)
    if ctx.params["bevel"] and bevel:
        mod = obj.modifiers.new("Bevel", "BEVEL")
        mod.width = mm(bevel)
        mod.segments = 2
        mod.limit_method = "ANGLE"
        mod.angle_limit = radians(smooth_angle)
        mod.harden_normals = True

    # Rigid bone parenting. Blender parents to the bone *tail*, so a parent
    # inverse of the tail's rest matrix makes matrix_basis == world rest
    # matrix. That also puts delta_location (used by the explode drivers) in
    # rest world space, and it rotates with the bone when the hand is posed.
    rest = ctx.bone_rest[bone]
    tail = ctx.rig.matrix_world @ rest @ Matrix.Translation((0, ctx.rig.data.bones[bone].length, 0))
    obj.parent = ctx.rig
    obj.parent_type = "BONE"
    obj.parent_bone = bone
    obj.matrix_parent_inverse = tail.inverted()
    obj.matrix_basis = frame

    obj["rh_part"] = True
    obj["rh_name"] = name
    obj["rh_bom"] = bom
    obj["rh_material"] = MATERIAL_SPECS[material]["label"]
    obj["rh_assembly"] = assembly
    obj["rh_explode"] = [round(c, 6) for c in explode]
    obj["dim"] = 0.0
    if meta.get("dof"):
        obj["rh_dof"] = meta["dof"]
    part = Part(obj, name, bom, material, assembly, Vector(explode), **meta)
    ctx.parts.append(part)
    return part


# ---- finger / thumb links ----------------------------------------------------

def link_body(mb, y0, y1, R, w_in, slot, web_top, round_end=False, web_start=None):
    """U-channel link: two side cheeks plus a dorsal web, open on the palmar side.

    The open palmar channel is the tendon path. The web is clipped below the
    cable (web_top) and, near the proximal pin, pulled back (web_start) to
    leave room for the tendon pulley.
    """
    prof = stadium(y0, y1, R, n=12, round0=True, round1=round_end)
    half, s = w_in / 2, slot / 2
    mb.add(prism(prof, AX_X, s, half))
    mb.add(prism(prof, AX_X, -half, -s))
    web = clip(prof, 1, web_top, "le")
    if web_start is not None:
        web = clip(web, 0, web_start, "ge")
    if len(web) >= 3:
        mb.add(prism(web, AX_X, -s, s))


def fork_plates(mb, yf0, L, R, w_in):
    """Clevis side plates at the distal end of a link, with a lightening slot."""
    x_in = w_in / 2 + mm(GAP)
    x_out = x_in + mm(PLATE_T)
    outer = stadium(yf0, L, R, n=12, round0=False, round1=True)
    s0, s1 = yf0 + 0.55 * R, L - 1.05 * R
    for a, b in ((x_in, x_out), (-x_out, -x_in)):
        if s1 - s0 > 0.2 * R:
            inner = stadium(s0, s1, 0.32 * R, n=12)
            mb.add(prism_hole(resample(outer, 40), resample(inner, 40), AX_X, a, b))
        else:
            mb.add(prism(outer, AX_X, a, b))
    return x_out


def build_link(ctx, ch, li, is_thumb=False):
    """Structural link plus its channel hardware (pulley, strap, conduit, pads)."""
    link = ch.links[li]
    F, L, R = link["frame"], mm(link["length"]), mm(link["R"])
    d = ch.dims
    w_in, slot = mm(d["w_in"]), mm(d["slot"])
    cable_z = mm(d["cable_z"])
    web_top = cable_z - mm(CABLE_R + 0.5)
    E = mm(ctx.params["explode_distance_mm"])
    pal = F.col[2].xyz
    depth = link["depth"] * (1.15 if is_thumb else 1.0)
    off = ch.dir * E * (1.25 * depth - 0.5) + ch.lat * E * depth
    bone = link["bone"]
    kind = link["name"]
    is_meta = kind == "Metacarpal"
    is_tip = kind == "Distal Phalanx"
    child = ch.links[li + 1] if li + 1 < len(ch.links) else None
    if child and child["name"] == "MCP Abduction Yoke":
        child = ch.links[li + 2]
    Rc = mm(child["R"]) if child else R
    clear = mm(1.5)
    asm = "Thumb" if is_thumb else ch.name
    title = f"{kind} — {ch.name}"
    mb = MeshBuilder()
    y_start = 0.0

    if is_thumb and is_meta:
        # CMC ball (the moving half of the saddle joint) and a slim neck.
        rb = mm(d["ball_r"])
        mb.add(uv_sphere(rb, 10, 20))
        y_start = rb + 0.9 * R
        mb.add(cylinder(0.45 * rb, 0.0, y_start, "y", n=16))

    if is_tip:
        link_body(mb, y_start, L - R, R, w_in, slot, web_top, round_end=True, web_start=R - mm(0.5))
    elif is_meta and not is_thumb:
        # Finger metacarpal: square end at the abduction pivot plus a round boss.
        link_body(mb, 0.0, L, R, w_in, slot, web_top)
        mb.add(cylinder(w_in / 2, -R, web_top, "z", at=(0, L, 0), n=24))
    else:
        body_end = L - Rc - clear
        link_body(mb, y_start, body_end, R, w_in, slot, web_top,
                  web_start=None if is_meta else R - mm(0.5))
        yf0 = max(y_start + R + mm(1.0), L - 2.4 * R)
        x_out = fork_plates(mb, yf0, L, R, w_in)
        # Dorsal bridge ties the two fork plates to the link body.
        mb.add(box(-x_out, x_out, yf0, body_end, -R - mm(BRIDGE_T), -R))

    add_part(ctx, title, mb, F, "chassis", bone, asm, off,
             bom=f"{kind} link ({ch.name})", label_level=0 if ch.index == 0 or is_tip else 1)

    # ---- tendon, pulley, retainer strap ---------------------------------------
    cable_end = L if not is_tip else 0.3 * L - mm(2.5)
    cable_start = 0.0
    if is_meta and not is_thumb:
        # The tendon enters the carpal frame through a bore; model it from where it exits.
        cable_start = (mm(CARPAL_Z1) - F.translation.z) / ch.dir.z + mm(0.5)
    elif is_thumb and is_meta:
        cable_start = y_start
    cb = MeshBuilder().tube((0, cable_start, cable_z), (0, cable_end, cable_z), mm(CABLE_R), 10)
    add_part(ctx, f"Flexor Tendon — {ch.name} {kind}", cb, F, "cable", bone, asm,
             off + pal * E * 1.6, bom="Tendon, Dyneema Ø1.0 (per link)", bevel=0, purchased=True, unit_cost=0.20)

    if not is_meta:
        pul = MeshBuilder().add(cylinder(cable_z - mm(CABLE_R), -(slot / 2 - mm(0.35)), slot / 2 - mm(0.35),
                                         "x", n=24, r_in=mm(PIN_R)))
        joint = {"Proximal Phalanx": "MCP" if not is_thumb else "MCP", "Middle Phalanx": "PIP",
                 "Distal Phalanx": "DIP" if not is_thumb else "IP"}[kind]
        add_part(ctx, f"Tendon Pulley — {ch.name} {joint}", pul, F, "steel", bone, asm,
                 off + pal * E * 0.9, bom="Grooved tendon pulley, 440C", purchased=True, unit_cost=3.50,
                 label_level=0 if (ch.key == "ring" and joint == "PIP") else None)

    span_end = L - Rc - clear if not is_tip else 0.45 * L
    if span_end - (y_start + R) > mm(6):
        ym = (y_start + R + span_end) / 2
        st = MeshBuilder().add(box(-(slot / 2 + mm(0.9)), slot / 2 + mm(0.9), ym - mm(1.4), ym + mm(1.4), R, R + mm(0.7)))
        add_part(ctx, f"Tendon Retainer — {ch.name} {kind}", st, F, "steel", bone, asm,
                 off + pal * E * 1.05, bom="Tendon retainer strap, 301 SS", bevel=0.15, purchased=True, unit_cost=0.40)

    # ---- dorsal wiring conduit + clip (phalanges only) ------------------------------
    if not is_meta:
        zc = -(R + mm(BRIDGE_T) + mm(CONDUIT_R))
        ya = y_start + R + mm(1.0)
        yb = (L - Rc - clear) if not is_tip else 0.42 * L + mm(2.0)
        if yb - ya > mm(4):
            cm = MeshBuilder().tube((0, ya, zc), (0, yb, zc), mm(CONDUIT_R), 12)
            add_part(ctx, f"Wiring Conduit — {ch.name} {kind}", cm, F, "conduit", bone, asm,
                     off - pal * E * 1.5, bom="Sensor wiring conduit, TPU", bevel=0, purchased=True, unit_cost=0.30,
                     label_level=0 if (ch.key == "little" and kind == "Proximal Phalanx") else None)
            yc = ya + mm(2.0)
            ztop = zc - mm(CONDUIT_R)
            clipm = MeshBuilder()
            for sgn in (1, -1):
                a, b = sorted((sgn * (mm(CONDUIT_R) + mm(0.1)), sgn * (mm(CONDUIT_R) + mm(0.9))))
                clipm.add(box(a, b, yc - mm(1.0), yc + mm(1.0), ztop, -R))
            clipm.add(box(-(mm(CONDUIT_R) + mm(0.9)), mm(CONDUIT_R) + mm(0.9), yc - mm(1.0), yc + mm(1.0),
                          ztop - mm(0.6), ztop))
            add_part(ctx, f"Conduit Clip — {ch.name} {kind}", clipm, F, "black_oxide", bone, asm,
                     off - pal * E * 1.0, bom="Conduit clip, PA12", bevel=0.1, purchased=True, unit_cost=0.15)

    if is_tip:
        hw = w_in / 2 - mm(0.3)
        py0 = min(0.45 * L, L - R - mm(3.0))
        pad = MeshBuilder().add(prism(rounded_rect(0, (py0 + L - R) / 2, hw, (L - R - py0) / 2, mm(1.2)),
                                      AX_Z, R, R + mm(0.9)))
        add_part(ctx, f"Tactile Sensor Pad — {ch.name}", pad, F, "sensor", bone, asm,
                 off + pal * E * 1.3, bom="Capacitive tactile sensor pad", bevel=0.2, purchased=True,
                 unit_cost=14.0, label_level=0 if ch.key in ("middle",) else 1)
        nail = MeshBuilder().add(prism(rounded_rect(0, (0.42 * L + L - 0.6 * R) / 2, w_in / 2 + mm(0.4),
                                                    (L - 0.6 * R - 0.42 * L) / 2, mm(1.5)),
                                       AX_Z, -R - mm(BRIDGE_T), -R))
        add_part(ctx, f"Fingertip Cap — {ch.name}", nail, F, "titanium", bone, asm,
                 off - pal * E * 0.9 + ch.dir * E * 0.4, bom="Fingertip cap, Ti-6Al-4V", bevel=0.25,
                 purchased=True, unit_cost=9.0)
        anc = MeshBuilder().add(cylinder(mm(0.75), 0.3 * L - mm(2.5), 0.3 * L, "y", at=(0, 0, cable_z), n=12))
        add_part(ctx, f"Tendon Anchor — {ch.name}", anc, F, "steel", bone, asm,
                 off + pal * E * 1.6, bom="Tendon crimp anchor", bevel=0, purchased=True, unit_cost=0.20)


def joint_hardware(ctx, ch, joint, frame, parent_bone, R, w_out, depth, asm):
    """Pin, flanged bearings, bolt caps and light rings for one revolute joint.

    Everything is modelled in the child's joint frame (pin on local X). It is
    parented to the parent link, because the pin is pressed into the parent's
    clevis. The exploded vectors slide the hardware out along the pin axis,
    the way a service manual shows it.
    """
    E = mm(ctx.params["explode_distance_mm"])
    Eh = E * 0.8
    X = frame.col[0].xyz
    base = ch.dir * E * (1.25 * depth - 0.5) + ch.lat * E * depth
    half = w_out / 2
    rb = 0.62 * R
    bt = mm(BEAR_T)
    tag = f"{ch.name} {joint}"

    pin = MeshBuilder().add(cylinder(mm(PIN_R), -(half + bt), half + bt, "x", n=16))
    add_part(ctx, f"Joint Pin — {tag}", pin, frame, "steel", parent_bone, asm, base + X * Eh * 0.9,
             bom=f"Shoulder pin Ø3 × {round((2 * (half + bt)) / _S)} mm, 440C", bevel=0.1,
             purchased=True, unit_cost=0.60, dof=f"{joint} revolute joint")
    for side, label in ((1, "Radial"), (-1, "Ulnar")):
        a, b = sorted((side * half, side * (half + bt)))
        brg = MeshBuilder().add(cylinder(rb, a, b, "x", n=24, r_in=mm(PIN_R)))
        add_part(ctx, f"Bearing — {tag} {label}", brg, frame, "steel", parent_bone, asm, base + X * side * Eh * 0.5,
                 bom=f"Flanged ball bearing 3×{round(2 * rb / _S)}×{BEAR_T}", bevel=0.1, purchased=True, unit_cost=1.80,
                 label_level=0 if (ch.index == 0 and joint == "PIP" and side == 1) else None)
        a, b = sorted((side * (half + bt), side * (half + bt + mm(1.0))))
        bolt = MeshBuilder().add(cylinder(0.36 * R, a, b, "x", n=6))
        add_part(ctx, f"Retaining Screw — {tag} {label}", bolt, frame, "black_oxide", parent_bone, asm,
                 base + X * side * Eh * 1.1, bom="M2 × 4 socket button screw, 12.9", bevel=0.12, purchased=True,
                 unit_cost=0.12)
        a, b = sorted((side * half, side * (half + mm(0.45))))
        ring = MeshBuilder().add(cylinder(rb + mm(0.85), a, b, "x", n=32, r_in=rb + mm(0.25)))
        add_part(ctx, f"Status Light Ring — {tag} {label}", ring, frame, "accent", parent_bone, asm,
                 base + X * side * Eh * 0.28, bom="Joint status light ring (light guide + LED)", bevel=0,
                 purchased=True, unit_cost=0.90)


def build_mcp_yoke(ctx, ch):
    """2-DOF MCP: abduction yoke pivoting on the metacarpal boss, carrying the flexion pin."""
    meta, yoke, prox = ch.links[0], ch.links[1], ch.links[2]
    F = yoke["frame"]
    R = mm(yoke["R"])
    d = ch.dims
    w_in, w_out = mm(d["w_in"]), mm(d["w_out"])
    t = mm(PLATE_T)
    E = mm(ctx.params["explode_distance_mm"])
    pal = F.col[2].xyz
    off = ch.dir * E * (1.25 * yoke["depth"] - 0.5) + ch.lat * E * yoke["depth"]
    L = mm(yoke["length"])
    cable_z = mm(d["cable_z"])

    def tab(y_front):
        return arc(0, 0, w_out / 2, pi, 2 * pi, 12) + [(w_out / 2, y_front), (-w_out / 2, y_front)]
    mb = MeshBuilder()
    mb.add(prism(tab(L), AX_Z, -R - t, -R))                     # dorsal tab (reaches the pin)
    mb.add(prism(tab(0.9 * R - mm(1.0)), AX_Z, R, R + t))       # palmar tab (clears flexion)
    x_in = w_in / 2 + mm(GAP)
    for a, b in ((x_in, x_in + t), (-x_in - t, -x_in)):
        mb.add(prism(stadium(0.8 * R, L, R, 12, round0=False), AX_X, a, b))
    add_part(ctx, f"MCP Abduction Yoke — {ch.name}", mb, F, "titanium", yoke["bone"], ch.name, off,
             bom=f"MCP abduction yoke, Ti-6Al-4V ({ch.name})", purchased=True, unit_cost=36.0,
             label_level=0 if ch.key == "middle" else 1, dof="MCP abduction ±20°")

    # Abduction pivot: dorsal pin into the metacarpal boss plus a palmar stub.
    pin = MeshBuilder().add(cylinder(mm(PIN_R), -R - t - mm(BEAR_T), -mm(1.0), "z", n=16))
    pin.add(cylinder(mm(PIN_R), R, R + t + mm(BEAR_T), "z", n=16))
    add_part(ctx, f"Abduction Pivot Pin — {ch.name}", pin, F, "steel", meta["bone"], ch.name,
             off - pal * E * 0.2 + ch.dir * 0, bom="Shoulder pin Ø3, abduction pivot", bevel=0.1,
             purchased=True, unit_cost=0.60)
    for sgn, (z0, z1) in ((-1, (-R - t - mm(BEAR_T), -R - t)), (1, (R + t, R + t + mm(BEAR_T)))):
        b = MeshBuilder().add(cylinder(0.55 * w_out / 2, z0, z1, "z", n=24, r_in=mm(PIN_R)))
        add_part(ctx, f"Abduction Bearing — {ch.name} {'Dorsal' if sgn < 0 else 'Palmar'}", b, F, "steel",
                 yoke["bone"], ch.name, off + pal * sgn * E * 1.0, bom="Flanged thrust bearing 3×8×1.4",
                 bevel=0.1, purchased=True, unit_cost=1.80)
        zz0, zz1 = sorted((z1 if sgn > 0 else z0, (z1 + mm(1.0)) if sgn > 0 else (z0 - mm(1.0))))
        s = MeshBuilder().add(cylinder(mm(2.2), zz0, zz1, "z", n=6))
        add_part(ctx, f"Pivot Screw — {ch.name} {'Dorsal' if sgn < 0 else 'Palmar'}", s, F, "black_oxide",
                 yoke["bone"], ch.name, off + pal * sgn * E * 1.9, bom="M2 × 4 socket button screw, 12.9",
                 bevel=0.12, purchased=True, unit_cost=0.12)
    cb = MeshBuilder().tube((0, 0, cable_z), (0, L, cable_z), mm(CABLE_R), 10)
    add_part(ctx, f"Flexor Tendon — {ch.name} MCP Yoke", cb, F, "cable", yoke["bone"], ch.name,
             off + pal * E * 1.6, bom="Tendon, Dyneema Ø1.0 (per link)", bevel=0, purchased=True, unit_cost=0.20)

    # MCP flexion joint hardware lives in the yoke.
    joint_hardware(ctx, ch, "MCP", prox["frame"], yoke["bone"], R, w_out, 1.75, ch.name)


def build_finger(ctx, ch):
    for li in (0, 2, 3, 4):
        build_link(ctx, ch, li)
    build_mcp_yoke(ctx, ch)
    joint_hardware(ctx, ch, "PIP", ch.links[3]["frame"], ch.links[2]["bone"], mm(ch.links[2]["R"]),
                   mm(ch.dims["w_out"]), 2.5, ch.name)
    joint_hardware(ctx, ch, "DIP", ch.links[4]["frame"], ch.links[3]["bone"], mm(ch.links[3]["R"]),
                   mm(ch.dims["w_out"]), 3.5, ch.name)


def build_thumb(ctx):
    th = ctx.thumb
    for li in range(3):
        build_link(ctx, th, li, is_thumb=True)
    joint_hardware(ctx, th, "MCP", th.links[1]["frame"], th.links[0]["bone"], mm(th.links[0]["R"]),
                   mm(th.dims["w_out"]), 1.5 * 1.15, "Thumb")
    joint_hardware(ctx, th, "IP", th.links[2]["frame"], th.links[1]["bone"], mm(th.links[1]["R"]),
                   mm(th.dims["w_out"]), 2.5 * 1.15, "Thumb")

    # Palm-side socket of the CMC saddle joint plus its mounting bracket.
    E = mm(ctx.params["explode_distance_mm"])
    rb = mm(th.dims["ball_r"])
    c = Vector([mm(v) for v in th.cmc])
    frame = Matrix.Translation(c) @ (-th.dir).to_track_quat("Z", "Y").to_matrix().to_4x4()
    cup = MeshBuilder().add(spherical_cup(rb + mm(0.1), rb + mm(2.4), radians(62)))
    add_part(ctx, "CMC Saddle Socket — Thumb", cup, frame, "titanium", "palm", "Thumb",
             th.dir * E * 0.2 + Vector((E * 0.9, 0, 0)), bom="CMC socket, Ti-6Al-4V", purchased=True, unit_cost=42.0,
             label_level=0, dof="Thumb CMC 3-axis (opposition)")
    hw = mm(ctx.carpal_hw)
    y_lim = mm(ctx.dorsal_y) - mm(2.0)
    a = Vector((hw - mm(4.0), max(c.y, -y_lim), c.z - mm(6.0)))
    b = c - th.dir * (rb + mm(1.6))
    b2 = c - th.dir * (rb + mm(9.0))
    br = MeshBuilder().tube(b2, b, mm(4.2), 16)      # straight into the back of the socket
    br.tube(a, b2, mm(4.2), 16)
    br.add(uv_sphere(mm(4.2), 8, 16), Matrix.Translation(b2))
    add_part(ctx, "Thumb Mount Bracket", br, Matrix.Identity(4), "anodized", "palm", "Palm",
             Vector((E * 0.5, 0, 0)), bom="Thumb mount bracket, 7075", purchased=True, unit_cost=22.0)


# ---- palm ---------------------------------------------------------------------

def build_palm(ctx):
    E = mm(ctx.params["explode_distance_mm"])
    hw = mm(ctx.carpal_hw)
    dy = mm(ctx.dorsal_y)                      # dorsal surface of the metacarpals
    plate_t = mm(2.5)
    top = dy + plate_t
    z0, z1 = mm(CARPAL_Z0), mm(CARPAL_Z1)
    I = Matrix.Identity(4)
    dorsal = Vector((0, 1, 0))

    carpal = MeshBuilder().add(prism(rounded_rect(0, (z0 + z1) / 2, hw, (z1 - z0) / 2, mm(5)), AX_XZ_Y, -top, top))
    add_part(ctx, "Carpal Frame", carpal, I, "chassis", "palm", "Palm", Vector((0, 0, -E * 0.6)),
             bom="Carpal frame, PA12-CF", bevel=0.6, label_level=0)

    # Dorsal frame plate: a skeletal plate notched between the metacarpals.
    pts = [(-hw, z1), (hw, z1)]
    tops = []
    for ch in ctx.chains:
        link = ch.links[0]
        cmc = link["frame"].translation
        s_end = mm(link["length"]) - (mm(ch.dims["w_out"]) / 2 + mm(2.0))
        p_end = cmc + ch.dir * s_end
        perp = Vector((ch.dir.z, 0, -ch.dir.x))
        half = mm(ch.dims["w_out"]) / 2 + mm(1.5)
        a, b = p_end + perp * half, p_end - perp * half
        tops.append(sorted([(a.x, a.z), (b.x, b.z)], key=lambda q: -q[0]))
    for i, pair in enumerate(tops):
        pts += pair
        if i < len(tops) - 1:
            nxt = tops[i + 1]
            vx = (pair[1][0] + nxt[0][0]) / 2
            vz = min(pair[1][1], nxt[0][1]) - mm(16.0)
            pts.append((vx, max(vz, z1 + mm(12))))
    plate = MeshBuilder().add(prism(pts, AX_XZ_Y, dy, top))
    add_part(ctx, "Dorsal Frame Plate", plate, I, "anodized", "palm", "Palm", dorsal * E * 1.8,
             bom="Dorsal frame plate, 7075-T6 CNC", purchased=True, unit_cost=28.0, label_level=0)

    manifold_z = z1 + mm(5.5)
    xs = []
    for ch in ctx.chains:
        link = ch.links[0]
        cmc = link["frame"].translation
        R1 = mm(ch.dims["R"][0])
        perp = Vector((ch.dir.z, 0, -ch.dir.x))
        s_a = (manifold_z + mm(6.0) - cmc.z) / ch.dir.z
        s_b = mm(link["length"]) - (mm(ch.dims["w_out"]) / 2 + mm(5.5))
        stations = (s_a, s_b) if s_b - s_a > mm(8) else ((s_a + s_b) / 2,)
        for k, station in enumerate(stations):
            p = cmc + ch.dir * station
            if R1 < dy - mm(0.05):
                sp = MeshBuilder().add(cylinder(mm(2.4), R1, dy, "y", at=(p.x, 0, p.z), n=16))
                add_part(ctx, f"Plate Spacer — {ch.name} {k + 1}", sp, I, "anodized", "palm", "Palm",
                         dorsal * E * 1.2, bom="Plate spacer, 7075", bevel=0, purchased=True, unit_cost=0.30)
            bolt = MeshBuilder().add(cylinder(mm(2.2), top, top + mm(1.2), "y", at=(p.x, 0, p.z), n=6))
            add_part(ctx, f"Plate Screw — {ch.name} {k + 1}", bolt, I, "black_oxide", "palm", "Palm",
                     dorsal * E * 2.8, bom="M2 × 8 socket button screw, 12.9", bevel=0.12, purchased=True, unit_cost=0.12)
        # Conduit along the metacarpal, offset from the screws.
        s0 = (manifold_z - cmc.z) / ch.dir.z
        s1 = mm(link["length"]) - (mm(ch.dims["w_out"]) / 2 + mm(3.0))
        a = cmc + ch.dir * s0 + perp * mm(3.6)
        b = cmc + ch.dir * s1 + perp * mm(3.6)
        a.y = b.y = top + mm(CONDUIT_R)
        xs.append(a.x)
        cond = MeshBuilder().tube(a, b, mm(CONDUIT_R), 12)
        add_part(ctx, f"Wiring Conduit — {ch.name} Metacarpal", cond, I, "conduit", "palm", "Palm",
                 dorsal * E * 3.4, bom="Sensor wiring conduit, TPU", bevel=0, purchased=True, unit_cost=0.30)
    mx0, mx1 = min(xs) - mm(4), max(xs) + mm(4)
    man = MeshBuilder().add(box(mx0, mx1, top, top + mm(2.6), manifold_z - mm(3.0), manifold_z + mm(3.0), r=0))
    add_part(ctx, "Wiring Harness Manifold", man, I, "chassis", "palm", "Palm", dorsal * E * 3.4,
             bom="Harness manifold, PA12-CF", bevel=0.3, label_level=1)
    led = MeshBuilder().add(box(-mm(9), mm(9), top + mm(2.6), top + mm(3.0), manifold_z - mm(0.8), manifold_z + mm(0.8)))
    add_part(ctx, "Status Light Bar", led, I, "accent", "palm", "Palm", dorsal * E * 3.9,
             bom="Status light bar (light guide + LED)", bevel=0, purchased=True, unit_cost=2.0)
    trunk = MeshBuilder().tube((0, top + mm(CONDUIT_R * 1.4), manifold_z - mm(3.0)), (0, top + mm(CONDUIT_R * 1.4), z0 + mm(4)),
                               mm(CONDUIT_R * 1.4), 12)
    add_part(ctx, "Wiring Trunk — Palm", trunk, I, "conduit", "palm", "Palm", dorsal * E * 3.4,
             bom="Sensor wiring conduit, TPU", bevel=0, purchased=True, unit_cost=0.30)


# ---- wrist ---------------------------------------------------------------------

def build_wrist(ctx):
    E = mm(ctx.params["explode_distance_mm"])
    I = Matrix.Identity(4)
    c = mm(5.5)                     # half size of the cross
    arm_in, arm_t, arm_r = c + mm(GAP), mm(2.5), mm(4.5)
    arm_out = arm_in + arm_t
    tr = mm(2.2)                    # trunnion radius
    bt = mm(BEAR_T)

    cross = MeshBuilder().add(box(-c, c, -c, c, -c, c, r=mm(1.5)))
    for axis in ("x", "y"):
        cross.add(cylinder(tr, -(arm_out + bt), arm_out + bt, axis, n=20))
    add_part(ctx, "Wrist Gimbal Cross", cross, I, "titanium", "wrist_pitch", "Wrist", Vector((0, 0, -E * 1.6)),
             bom="Wrist universal-joint cross, Ti-6Al-4V", purchased=True, unit_cost=58.0, label_level=0,
             dof="Wrist pitch −60/+70°, yaw −30/+20°")

    # Palm-side arms (±Y) hang from the carpal frame; forearm arms (±X) rise from the flange.
    palm_arms = MeshBuilder()
    for a, b in ((arm_in, arm_out), (-arm_out, -arm_in)):
        palm_arms.add(prism(stadium(0.0, mm(CARPAL_Z0), arm_r, 12, round1=False), (2, 0, 1), a, b))
    add_part(ctx, "Wrist Yoke — Palm Side", palm_arms, I, "anodized", "palm", "Wrist", Vector((0, 0, -E * 0.9)),
             bom="Wrist yoke (palm side), 7075-T6", purchased=True, unit_cost=34.0, label_level=1)
    fore_arms = MeshBuilder()
    for a, b in ((arm_in, arm_out), (-arm_out, -arm_in)):
        fore_arms.add(prism(stadium(-mm(16.0), 0.0, arm_r, 12, round0=False), (2, 1, 0), a, b))
    add_part(ctx, "Wrist Yoke — Forearm Side", fore_arms, I, "anodized", "forearm", "Wrist",
             Vector((0, 0, -E * 2.3)), bom="Wrist yoke (forearm side), 7075-T6", purchased=True, unit_cost=34.0,
             label_level=1)
    for axis, bone, oz, name in (("x", "forearm", -E * 2.3, "Pitch"), ("y", "palm", -E * 0.9, "Yaw")):
        vec = Vector((1, 0, 0)) if axis == "x" else Vector((0, 1, 0))
        for side in (1, -1):
            a, b = sorted((side * arm_out, side * (arm_out + bt)))
            brg = MeshBuilder().add(cylinder(mm(3.9), a, b, axis, n=24, r_in=tr))
            add_part(ctx, f"Wrist {name} Bearing {'+' if side > 0 else '−'}", brg, I, "steel", bone, "Wrist",
                     Vector((0, 0, oz)) + vec * side * E * 1.0, bom="Flanged ball bearing 4.4×8×1.4", bevel=0.1,
                     purchased=True, unit_cost=2.20)
            a, b = sorted((side * (arm_out + bt), side * (arm_out + bt + mm(1.0))))
            cap = MeshBuilder().add(cylinder(mm(2.6), a, b, axis, n=6))
            add_part(ctx, f"Wrist {name} Screw {'+' if side > 0 else '−'}", cap, I, "black_oxide", bone, "Wrist",
                     Vector((0, 0, oz)) + vec * side * E * 1.9, bom="M2.5 × 5 socket button screw, 12.9", bevel=0.12,
                     purchased=True, unit_cost=0.14)


# ---- forearm actuator pack ----------------------------------------------------

def actuator_names(ctx):
    names = [f"{ch.name} Flexor" for ch in ctx.chains]
    return names + ["Thumb Flexor", "Thumb Opposition", "Finger Spread", "Palm Cup", "Wrist Pitch", "Wrist Yaw"]


def build_forearm(ctx):
    E = mm(ctx.params["explode_distance_mm"])
    I = Matrix.Identity(4)
    down = Vector((0, 0, -1))
    fl = MeshBuilder().add(box(-mm(29), mm(29), -mm(17), mm(17), -mm(20), -mm(16), r=mm(5)))
    add_part(ctx, "Wrist Flange", fl, I, "anodized", "forearm", "Forearm", down * E * 2.8,
             bom="Wrist flange, 7075-T6 CNC", purchased=True, unit_cost=26.0, label_level=1)

    rail_outer = rounded_rect(-mm(62), 0, mm(42), mm(15), mm(3), 6)
    rail_hole = stadium(-mm(88), -mm(40), mm(6.5), 12)
    rails = MeshBuilder()
    for a, b in ((mm(26.5), mm(29)), (-mm(29), -mm(26.5))):
        rails.add(prism_hole(resample(rail_outer, 48), resample(rail_hole, 48), (2, 1, 0), a, b))
    add_part(ctx, "Forearm Side Rails", rails, I, "anodized", "forearm", "Forearm", down * E * 3.4,
             bom="Forearm side rail pair, 7075-T6", purchased=True, unit_cost=38.0, label_level=1)
    deck = MeshBuilder().add(box(-mm(26.5), mm(26.5), -mm(15), mm(15), -mm(34), -mm(31)))
    add_part(ctx, "Spool Deck", deck, I, "chassis", "forearm", "Forearm", down * E * 3.8,
             bom="Actuator spool deck, PA12-CF", bevel=0.3)

    names = actuator_names(ctx)
    cols = math.ceil(len(names) / 2)
    pitch = mm(42.0) / max(cols - 1, 1)
    motor_r = min(mm(4.6), pitch / 2 - mm(0.4))
    for i, label in enumerate(names):
        col, row = i // 2, i % 2
        x = -mm(21.0) + col * pitch if cols > 1 else 0.0
        y = mm(7.4) if row == 0 else -mm(7.4)
        spread = Vector((x, y, 0)) * 0.9 / mm(10)
        at = (x, y, 0)
        motor = MeshBuilder().add(cylinder(motor_r, -mm(72), -mm(40), "z", at=at, n=24))
        motor.add(cylinder(motor_r + mm(0.3), -mm(40), -mm(34), "z", at=at, n=24))
        motor.add(cylinder(motor_r - mm(0.6), -mm(76), -mm(72), "z", at=at, n=24))
        add_part(ctx, f"Actuator {i + 1:02d} — {label}", motor, I, "motor", "forearm", "Forearm",
                 down * E * 4.8 + spread * E * 0.35,
                 bom="Coreless DC gearmotor Ø9.5, 136:1, magnetic encoder", purchased=True, unit_cost=42.0,
                 catalog_mass_g=13.5, label_level=0 if i == 0 else 1, dof=f"Drives: {label}")
        spool = MeshBuilder().add(cylinder(mm(2.6), -mm(31), -mm(26.5), "z", at=at, n=20))
        spool.add(cylinder(mm(3.8), -mm(31), -mm(30.4), "z", at=at, n=20))
        spool.add(cylinder(mm(3.8), -mm(27.1), -mm(26.5), "z", at=at, n=20))
        add_part(ctx, f"Tendon Spool — Actuator {i + 1:02d}", spool, I, "steel", "forearm", "Forearm",
                 down * E * 3.3 + spread * E * 0.2, bom="Tendon spool, 7075 anodised", bevel=0.1, purchased=True,
                 unit_cost=4.0)
        side = -1 if y > 0 else 1
        cab = MeshBuilder().tube((x, y + side * mm(2.6 + CABLE_R), -mm(28.8)), (x, y + side * mm(2.6 + CABLE_R), -mm(20)),
                                 mm(CABLE_R), 10)
        add_part(ctx, f"Tendon Lead — Actuator {i + 1:02d}", cab, I, "cable", "forearm", "Forearm",
                 down * E * 3.1 + spread * E * 0.2, bom="Tendon, Dyneema Ø1.0 (per link)", bevel=0, purchased=True,
                 unit_cost=0.20)

    pcb = MeshBuilder().add(box(-mm(25), mm(25), -mm(13.5), mm(13.5), -mm(92), -mm(90.4)))
    for k, xx in enumerate((-mm(14), mm(0), mm(14))):
        pcb.add(box(xx - mm(1.8), xx + mm(1.8), -mm(9), -mm(6), -mm(90.4), -mm(89.6)))
    pcb.add(box(-mm(8), mm(8), mm(4), mm(12), -mm(90.4), -mm(87.4)))
    add_part(ctx, "Motor Driver PCB", pcb, I, "pcb", "forearm", "Forearm", down * E * 5.8,
             bom="10-channel motor driver + sensor hub PCB", bevel=0.1, purchased=True, unit_cost=120.0,
             label_level=0)
    leds = MeshBuilder()
    for xx in (-mm(20), -mm(17.5), -mm(15)):
        leds.add(box(xx - mm(0.6), xx + mm(0.6), mm(9.5), mm(11), -mm(90.4), -mm(89.9)))
    add_part(ctx, "PCB Status LEDs", leds, I, "accent", "forearm", "Forearm", down * E * 5.8,
             bom="Status LED array", bevel=0, purchased=True, unit_cost=0.60)
    so = MeshBuilder()
    for sx in (-1, 1):
        for sy in (-1, 1):
            so.add(cylinder(mm(1.6), -mm(104), -mm(92), "z", at=(sx * mm(22), sy * mm(10.5), 0), n=12))
    add_part(ctx, "PCB Standoffs", so, I, "steel", "forearm", "Forearm", down * E * 6.3,
             bom="M2 standoff 12 mm (set of 4)", bevel=0, purchased=True, unit_cost=1.20)
    bp = MeshBuilder().add(box(-mm(29), mm(29), -mm(17), mm(17), -mm(108), -mm(104), r=mm(5)))
    add_part(ctx, "Forearm Base Plate", bp, I, "anodized", "forearm", "Forearm", down * E * 6.8,
             bom="Forearm base plate, 7075-T6 CNC", purchased=True, unit_cost=24.0, label_level=1)


# =============================================================================
# 6. EXPLODED VIEW
#    delta_location = explode * vector (rest world space). Vectors come from
#    the part's chain depth (distal parts travel furthest along the finger
#    axis, the palm barely moves) plus its joint axis (pins, bearings and
#    screws slide out along the hinge axis).
# =============================================================================

def build_explode_drivers(ctx):
    for part in ctx.parts:
        for axis in range(3):
            k = part.explode[axis]
            if abs(k) > 1e-7:
                add_driver(part.obj, "delta_location", axis, f"e*{k:.6f}", {"e": (ctx.rig, '["explode"]')})


# =============================================================================
# 7. LABELS
#    Each callout = anchor dot (parented to its part, so it follows explode
#    and pose) + camera-facing text in a tidy column + a leader curve hooked
#    to both. Visibility is driver-based: show_labels, label_detail and
#    isolate mode.
# =============================================================================

def _font(name):
    path = bpy.utils.system_resource("DATAFILES", path=f"fonts/{name}")
    try:
        return bpy.data.fonts.load(path, check_existing=True)
    except Exception:
        return None


def world_center(obj, explode=0.0):
    corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    center = sum(corners, Vector()) / 8.0
    return center + Vector(obj.get("rh_explode", (0, 0, 0))) * explode - obj.delta_location


def build_labels(ctx, camera):
    coll = ctx.coll["labels"]
    font = _font("Inter.woff2")
    root = bpy.data.objects.new("RH_LabelRoot", None)
    coll.objects.link(root)
    cam_m = camera.matrix_world
    right, up = cam_m.col[0].xyz.normalized(), cam_m.col[1].xyz.normalized()
    center = Vector((0, 0, mm(20)))
    bpy.context.view_layer.update()

    items = []
    for part in ctx.parts:
        if part.label_level is None:
            continue
        p = world_center(part.obj, explode=1.0)
        items.append((part, p, (p - center).dot(right), (p - center).dot(up)))
    extent = max((abs(it[2]) for it in items), default=mm(100))
    column = extent + mm(22)
    size = mm(8.0)
    spacing = size * 1.9
    for side in (1, -1):
        col_items = sorted([it for it in items if (it[2] >= 0) == (side > 0)], key=lambda it: -it[3])
        # Resolve vertical collisions top-down, keeping each label near its part height.
        placed = []
        for it in col_items:
            v = it[3]
            if placed and placed[-1] - v < spacing:
                v = placed[-1] - spacing
            placed.append(v)
        for (part, p, _, _), v in zip(col_items, placed):
            _make_callout(ctx, part, center + right * side * column + up * v, side, size, font, root, camera)


def _vis_driver(ctx, obj, level, anchor):
    expr = f"1-min(1,(s*(d>={level})+i*(1-h))*(1-h*i))"
    for path in ("hide_viewport", "hide_render"):
        add_driver(obj, path, -1, expr,
                        {"s": (ctx.rig, '["show_labels"]'), "d": (ctx.rig, '["label_detail"]'),
                         "i": (ctx.rig, '["isolate"]'), "h": (anchor, '["iso_hide"]')})


def _make_callout(ctx, part, text_pos, side, size, font, root, camera):
    coll = ctx.coll["labels"]
    level = part.label_level
    # Anchor dot on the part.
    anchor_pos = world_center(part.obj)
    me = bpy.data.meshes.new("RH_LabelDot")
    v, f = uv_sphere(mm(0.9), 6, 10)
    me.from_pydata(v, [], f)
    me.materials.append(ctx.mats["leader"])
    dot = bpy.data.objects.new(f"RH_Dot_{part.obj.name}", me)
    coll.objects.link(dot)
    dot.location = anchor_pos
    dot.parent = part.obj
    dot.matrix_parent_inverse = part.obj.matrix_world.inverted()
    dot["iso_hide"] = 0
    dot["rh_label_for"] = part.obj.name

    # Text: camera-facing, left/right aligned by column.
    cu = bpy.data.curves.new(f"RH_LabelText_{part.obj.name}", "FONT")
    cu.body = part.name.upper()
    cu.size = size
    cu.align_x = "LEFT" if side > 0 else "RIGHT"
    cu.align_y = "CENTER"
    cu.offset_x = mm(3.0) * side
    if font:
        cu.font = font
    cu.materials.append(ctx.mats["label"])
    txt = bpy.data.objects.new(f"RH_Label_{part.obj.name}", cu)
    coll.objects.link(txt)
    txt.location = text_pos
    txt.parent = root
    con = txt.constraints.new("TRACK_TO")
    con.target = camera
    con.track_axis = "TRACK_Z"
    con.up_axis = "UP_Y"
    txt["rh_label_camera"] = True

    # Leader: a 2-point poly curve with its ends hooked to the dot and the text.
    lc = bpy.data.curves.new(f"RH_Leader_{part.obj.name}", "CURVE")
    lc.dimensions = "3D"
    lc.bevel_depth = mm(0.25)
    lc.bevel_resolution = 0
    sp = lc.splines.new("POLY")
    sp.points.add(1)
    sp.points[0].co = (*anchor_pos, 1.0)
    sp.points[1].co = (*text_pos, 1.0)
    lc.materials.append(ctx.mats["leader"])
    lead = bpy.data.objects.new(f"RH_Leader_{part.obj.name}", lc)
    coll.objects.link(lead)
    for idx, target, pos in ((0, dot, anchor_pos), (1, txt, text_pos)):
        hk = lead.modifiers.new(f"Hook{idx}", "HOOK")
        hk.object = target
        hk.vertex_indices_set([idx])
        # Bind against the rest position explicitly (matrix_world isn't evaluated yet).
        hk.matrix_inverse = Matrix.Translation(pos).inverted()
        hk.center = pos
    for o in (dot, txt, lead):
        _vis_driver(ctx, o, level, dot)


# =============================================================================
# 8. STUDIO: WORLD, FLOOR, LIGHTS, CAMERAS
# =============================================================================

FLOOR_Z = -240.0     # mm; leaves room for the forearm in the exploded view


def build_world(ctx):
    world = bpy.data.worlds.new("RH_World")
    bpy.context.scene.world = world
    if world.node_tree is None:
        world.use_nodes = True
    N, L = world.node_tree.nodes, world.node_tree.links
    N.clear()
    out = N.new("ShaderNodeOutputWorld")
    coords = N.new("ShaderNodeTexCoord")
    mapping = N.new("ShaderNodeMapping")
    mapping.inputs["Rotation"].default_value = (0, 0, radians(115))
    L.new(coords.outputs["Generated"], mapping.inputs["Vector"])
    env = N.new("ShaderNodeTexEnvironment")
    try:
        env.image = bpy.data.images.load(bpy.utils.system_resource("DATAFILES", path="studiolights/world/studio.exr"),
                                         check_existing=True)
    except Exception:
        pass
    L.new(mapping.outputs["Vector"], env.inputs["Vector"])
    neutral = N.new("ShaderNodeHueSaturation")          # the bundled HDRI has a warm/purple cast
    neutral.inputs["Saturation"].default_value = 0.0
    L.new(env.outputs["Color"], neutral.inputs["Color"])
    bg_env = N.new("ShaderNodeBackground")
    bg_env.inputs["Strength"].default_value = 0.30      # mostly reflections; the area lights do the lighting
    L.new(neutral.outputs["Color"], bg_env.inputs["Color"])
    # What the camera sees: a soft radial glow behind the subject fading to near-black.
    win = N.new("ShaderNodeTexCoord")
    centre = N.new("ShaderNodeVectorMath"); centre.operation = "DISTANCE"
    centre.inputs[1].default_value = (0.5, 0.58, 0.0)
    L.new(win.outputs["Window"], centre.inputs[0])
    ramp = N.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = srgb("#101B29")
    ramp.color_ramp.elements[1].position = 0.75
    ramp.color_ramp.elements[1].color = srgb(PALETTE["backdrop"])
    L.new(centre.outputs["Value"], ramp.inputs["Fac"])
    bg_cam = N.new("ShaderNodeBackground")
    L.new(ramp.outputs["Color"], bg_cam.inputs["Color"])
    path = N.new("ShaderNodeLightPath")
    studio = N.new("ShaderNodeMixShader")
    L.new(path.outputs["Is Camera Ray"], studio.inputs[0])
    L.new(bg_env.outputs[0], studio.inputs[1])
    L.new(bg_cam.outputs[0], studio.inputs[2])
    bg_bp = N.new("ShaderNodeBackground")
    bg_bp.inputs["Color"].default_value = srgb(PALETTE["bp_navy"])
    tech = N.new("ShaderNodeMixShader")
    L.new(studio.outputs[0], tech.inputs[1])
    L.new(bg_bp.outputs[0], tech.inputs[2])
    L.new(tech.outputs[0], out.inputs["Surface"])
    add_driver(tech.inputs[0], "default_value", -1, "t", {"t": (ctx.rig, '["technical_view"]')}, owner=world.node_tree)


def _grid_material(name, major=0.05, minor=0.01):
    """Blueprint grid emission (object space), 10 mm minor and 50 mm major lines."""
    mat = _new_node_material(name)
    N, L = mat.node_tree.nodes, mat.node_tree.links
    out = N.new("ShaderNodeOutputMaterial")
    tc = N.new("ShaderNodeTexCoord")
    sep = N.new("ShaderNodeSeparateXYZ")
    L.new(tc.outputs["Object"], sep.inputs[0])
    line_total = None
    for spacing, width, weight in ((minor, 0.03, 0.35), (major, 0.012, 1.0)):
        for comp in ("X", "Y"):
            m1 = N.new("ShaderNodeMath"); m1.operation = "DIVIDE"; m1.inputs[1].default_value = spacing
            L.new(sep.outputs[comp], m1.inputs[0])
            m2 = N.new("ShaderNodeMath"); m2.operation = "PINGPONG"; m2.inputs[1].default_value = 0.5
            L.new(m1.outputs[0], m2.inputs[0])
            m3 = N.new("ShaderNodeMath"); m3.operation = "LESS_THAN"; m3.inputs[1].default_value = width
            L.new(m2.outputs[0], m3.inputs[0])
            m4 = N.new("ShaderNodeMath"); m4.operation = "MULTIPLY"; m4.inputs[1].default_value = weight
            L.new(m3.outputs[0], m4.inputs[0])
            if line_total is None:
                line_total = m4
            else:
                mx = N.new("ShaderNodeMath"); mx.operation = "MAXIMUM"
                L.new(line_total.outputs[0], mx.inputs[0]); L.new(m4.outputs[0], mx.inputs[1])
                line_total = mx
    mix = N.new("ShaderNodeMix"); mix.data_type = "RGBA"
    _socket(mix.inputs, "A_Color").default_value = srgb(PALETTE["bp_navy"])
    _socket(mix.inputs, "B_Color").default_value = srgb("#2A6C9C")
    L.new(line_total.outputs[0], _socket(mix.inputs, "Factor_Float"))
    em = N.new("ShaderNodeEmission")
    L.new(_socket(mix.outputs, "Result_Color"), em.inputs["Color"])
    L.new(em.outputs[0], out.inputs["Surface"])
    return mat


def build_studio(ctx):
    coll = ctx.coll["studio"]
    s = ctx.params["hand_scale"]
    # Floor: glossy black that fades into the background with distance.
    fl = _new_node_material("RH_Floor")
    N, L = fl.node_tree.nodes, fl.node_tree.links
    out = N.new("ShaderNodeOutputMaterial")
    bsdf = N.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = srgb("#08090B")
    bsdf.inputs["Roughness"].default_value = 0.45
    bsdf.inputs["Specular IOR Level"].default_value = 0.14
    tc = N.new("ShaderNodeTexCoord")
    length = N.new("ShaderNodeVectorMath"); length.operation = "LENGTH"
    L.new(tc.outputs["Object"], length.inputs[0])
    rng = N.new("ShaderNodeMapRange")
    rng.inputs["From Min"].default_value = 0.30 * s
    rng.inputs["From Max"].default_value = 0.95 * s
    L.new(length.outputs["Value"], rng.inputs["Value"])
    bg = N.new("ShaderNodeBsdfTransparent")     # fade into the world backdrop, no horizon line
    fade = N.new("ShaderNodeMixShader")
    L.new(rng.outputs["Result"], fade.inputs[0])
    L.new(bsdf.outputs[0], fade.inputs[1]); L.new(bg.outputs[0], fade.inputs[2])
    tech_em = N.new("ShaderNodeEmission")
    tech_em.inputs["Color"].default_value = srgb(PALETTE["bp_navy"])
    tech = N.new("ShaderNodeMixShader")
    L.new(fade.outputs[0], tech.inputs[1]); L.new(tech_em.outputs[0], tech.inputs[2])
    L.new(tech.outputs[0], out.inputs["Surface"])
    add_driver(tech.inputs[0], "default_value", -1, "t", {"t": (ctx.rig, '["technical_view"]')}, owner=fl.node_tree)
    floor = MeshBuilder().add(prism(circle(0, 0, 4.0 * s, 64), AX_Z, mm(FLOOR_Z) - 0.01 * s, mm(FLOOR_Z)))
    me = bpy.data.meshes.new("RH_Floor")
    me.from_pydata(floor.verts, [], floor.faces)
    me.materials.append(fl)
    ob = bpy.data.objects.new("RH_Floor", me)
    coll.objects.link(ob)

    # Display stand (not part of the product; hidden once the hand explodes).
    stand = MeshBuilder()
    stand.add(cylinder(mm(48), mm(FLOOR_Z), mm(FLOOR_Z + 6), "z", n=64))
    stand.add(cylinder(mm(7), mm(FLOOR_Z + 6), mm(-108), "z", n=32))
    me = bpy.data.meshes.new("RH_DisplayStand")
    me.from_pydata(stand.verts, [], stand.faces)
    for poly in me.polygons:
        poly.use_smooth = True
    me.set_sharp_from_angle(angle=radians(35))
    me.materials.append(ctx.mats["titanium"])
    st = bpy.data.objects.new("RH_DisplayStand", me)
    coll.objects.link(st)
    for path in ("hide_render", "hide_viewport"):
        add_driver(st, path, -1, "e>0.001", {"e": (ctx.rig, '["explode"]')})

    # Lights: large soft key, two rim strips (one cyan-tinted), low fill, top.
    def area(name, loc, target, power, size, size_y=None, color=(1, 1, 1)):
        ld = bpy.data.lights.new(name, "AREA")
        ld.energy = power * s * s
        ld.color = color
        if size_y:
            ld.shape = "RECTANGLE"
            ld.size, ld.size_y = size * s, size_y * s
        else:
            ld.size = size * s
        lo = bpy.data.objects.new(name, ld)
        coll.objects.link(lo)
        lo.location = Vector(loc) * s
        d = (Vector(target) * s - lo.location)
        lo.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
        return lo
    # Watts are calibrated for a ~0.3 m subject 1 m from the lights.
    area("RH_Key", (0.95, -0.35, 0.70), (0, 0, 0.03), 4.5, 0.7)
    area("RH_RimWhite", (-0.65, 0.60, 0.40), (0, 0, 0.05), 7.0, 0.10, 1.0)
    area("RH_RimCyan", (0.75, 0.55, 0.05), (0, 0, 0.02), 4.5, 0.10, 0.9, color=(0.45, 0.85, 1.0))
    area("RH_Fill", (-0.60, -0.80, 0.10), (0, 0, 0.0), 1.2, 0.9)
    area("RH_Top", (0.0, -0.1, 0.9), (0, 0, 0.05), 1.8, 0.5)


def add_camera(ctx, name, loc, target, lens=85.0, ortho_scale=None):
    cam = bpy.data.cameras.new(name)
    cam.lens = lens
    cam.clip_start = 0.005
    cam.clip_end = 50.0
    if ortho_scale:
        cam.type = "ORTHO"
        cam.ortho_scale = ortho_scale
    ob = bpy.data.objects.new(name, cam)
    ctx.coll["studio"].objects.link(ob)
    ob.location = loc
    ob.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return ob


def build_cameras(ctx):
    s = ctx.params["hand_scale"]
    t = Vector((0, 0, 0.035)) * s
    cams = {
        "hero": add_camera(ctx, "RH_CAM_Hero", Vector((-0.42, -0.78, 0.26)) * s, t, 70),
        "exploded": add_camera(ctx, "RH_CAM_Exploded", Vector((-0.75, -1.25, 0.35)) * s, Vector((0, 0, 0.01)) * s, 60),
        "labeled": add_camera(ctx, "RH_CAM_Labeled", Vector((0.18, 1.45, 0.20)) * s, Vector((0, 0, 0.015)) * s, 55),
    }
    bpy.context.scene.camera = cams["hero"]
    ctx.cameras = cams
    return cams


# =============================================================================
# 9. BILL OF MATERIALS, MASS AND COST, SPEC DATA
# =============================================================================

# Cost model for parts made to order (USD, low-volume prototype pricing).
PRINT_COST_PER_CM3 = 0.45      # MJF PA12-CF
PRINT_HANDLING = 2.50          # per printed part


def measure_volume_cm3(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    ev = obj.evaluated_get(dg)
    bm = bmesh.new()
    bm.from_mesh(ev.to_mesh())
    ev.to_mesh_clear()
    vol = abs(bm.calc_volume(signed=False))
    bm.free()
    return vol * 1e6     # m^3 -> cm^3


def compute_bom(ctx):
    rows = {}
    parts_out = []
    for part in ctx.parts:
        vol = measure_volume_cm3(part.obj)
        spec = MATERIAL_SPECS[part.material]
        mass = part.catalog_mass_g if part.catalog_mass_g else vol * (spec["density"] or 1.0)
        if part.unit_cost is not None:
            cost = part.unit_cost
        else:
            cost = PRINT_HANDLING + vol * PRINT_COST_PER_CM3
        part.obj["rh_mass_g"] = round(mass, 2)
        part.obj["rh_cost_usd"] = round(cost, 2)
        key = (part.bom, spec["label"])
        row = rows.setdefault(key, dict(item=part.bom, material=spec["label"], qty=0, unit_mass_g=0.0,
                                        mass_g=0.0, unit_cost=0.0, cost=0.0,
                                        source="Purchased / outsourced" if part.purchased else "In-house print",
                                        assemblies=set()))
        row["qty"] += 1
        row["mass_g"] += mass
        row["cost"] += cost
        row["assemblies"].add(part.assembly)
        parts_out.append(dict(name=part.name, bom=part.bom, material=spec["label"], assembly=part.assembly,
                              volume_cm3=round(vol, 3), mass_g=round(mass, 2), cost_usd=round(cost, 2)))
    table = []
    for row in rows.values():
        row["unit_mass_g"] = row["mass_g"] / row["qty"]
        row["unit_cost"] = row["cost"] / row["qty"]
        row["assemblies"] = ", ".join(sorted(row["assemblies"]))
        table.append(row)
    table.sort(key=lambda r: (-r["cost"]))
    return table, parts_out


def joint_spec(ctx):
    joints = []
    for ch in ctx.chains:
        joints += [(f"{ch.name} CMC (cupping)", ROM["finger_cmc"], "Palm Cup") if ch.row["cup"] > 0 else None,
                   (f"{ch.name} MCP abduction", ROM["finger_abd"], "Finger Spread"),
                   (f"{ch.name} MCP flexion", ROM["finger_mcp"], f"{ch.name} Flexor"),
                   (f"{ch.name} PIP flexion", ROM["finger_pip"], f"{ch.name} Flexor (coupled)"),
                   (f"{ch.name} DIP flexion", ROM["finger_dip"], f"{ch.name} Flexor (coupled)")]
    joints += [("Thumb CMC flexion", ROM["thumb_cmc_x"], "Thumb Opposition"),
               ("Thumb CMC axial rotation", ROM["thumb_cmc_y"], "Thumb Opposition (coupled)"),
               ("Thumb CMC abduction", ROM["thumb_cmc_z"], "Thumb Opposition (coupled)"),
               ("Thumb MCP flexion", ROM["thumb_mcp"], "Thumb Flexor"),
               ("Thumb IP flexion", ROM["thumb_ip"], "Thumb Flexor (coupled)"),
               ("Wrist pitch", ROM["wrist_pitch"], "Wrist Pitch"),
               ("Wrist yaw", ROM["wrist_yaw"], "Wrist Yaw")]
    return [dict(joint=j[0], min_deg=j[1][0], max_deg=j[1][1], actuator=j[2]) for j in joints if j]


def overall_dimensions(ctx):
    bpy.context.view_layer.update()
    mn, mx = Vector((1e9,) * 3), Vector((-1e9,) * 3)
    for part in ctx.parts:
        for c in part.obj.bound_box:
            w = part.obj.matrix_world @ Vector(c)
            mn = Vector(map(min, mn, w))
            mx = Vector(map(max, mx, w))
    size = (mx - mn) / _S
    return dict(width_mm=round(size.x, 1), thickness_mm=round(size.y, 1), length_mm=round(size.z, 1),
                min=[round(v / _S, 1) for v in mn], max=[round(v / _S, 1) for v in mx])


def write_reports(ctx, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    table, parts = compute_bom(ctx)
    total_mass = sum(r["mass_g"] for r in table)
    total_cost = sum(r["cost"] for r in table)
    joints = joint_spec(ctx)
    actuators = actuator_names(ctx)
    tip = ctx.chains[min(1, len(ctx.chains) - 1)]
    spec = dict(
        product=PRODUCT_NAME,
        params=ctx.params,
        dimensions=overall_dimensions(ctx),
        joints_total=len(joints),
        dof_total=len(joints),
        actuators_total=len(actuators),
        actuators=actuators,
        actuation="Tendon-driven (Dyneema Ø1.0 over grooved pulleys), under-actuated coupled flexion, "
                  "elastic extension",
        joints=joints,
        mass_g=round(total_mass, 1),
        cost_usd=round(total_cost, 0),
        part_count=len(ctx.parts),
        unique_parts=len(table),
        fingers=[dict(name=ch.name, meta_mm=round(ch.links[0]["length"] + ch.links[1]["length"], 1),
                      prox_mm=ch.links[2]["length"], mid_mm=ch.links[3]["length"], dist_mm=ch.links[4]["length"],
                      width_mm=round(ch.dims["width"], 1)) for ch in ctx.chains],
        thumb=dict(meta_mm=ctx.thumb.links[0]["length"], prox_mm=ctx.thumb.links[1]["length"],
                   dist_mm=ctx.thumb.links[2]["length"]),
        presets=json.loads(ctx.rig["rh_presets"]),
        mass_by_assembly={},
        cost_by_source={},
    )
    for p in parts:
        spec["mass_by_assembly"][p["assembly"]] = round(spec["mass_by_assembly"].get(p["assembly"], 0) + p["mass_g"], 1)
    for r in table:
        spec["cost_by_source"][r["source"]] = round(spec["cost_by_source"].get(r["source"], 0) + r["cost"], 1)

    import csv
    with open(os.path.join(out_dir, "bom.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["Item", "Qty", "Material", "Source", "Unit mass (g)", "Total mass (g)", "Unit cost (USD)",
                    "Total cost (USD)", "Used in"])
        for r in table:
            w.writerow([r["item"], r["qty"], r["material"], r["source"], f"{r['unit_mass_g']:.2f}",
                        f"{r['mass_g']:.1f}", f"{r['unit_cost']:.2f}", f"{r['cost']:.2f}", r["assemblies"]])
        w.writerow([])
        w.writerow(["TOTAL", sum(r["qty"] for r in table), "", "", "", f"{total_mass:.1f}", "", f"{total_cost:.2f}", ""])
    with open(os.path.join(out_dir, "bom.md"), "w", encoding="utf-8") as fh:
        fh.write(f"# {PRODUCT_NAME} — Bill of Materials\n\n")
        fh.write(f"Generated by `hand_generator.py` from the model itself: volumes are measured from the mesh and "
                 f"multiplied by material density; catalogue masses are used for purchased motors.\n\n")
        fh.write(f"**{len(ctx.parts)} parts · {len(table)} unique line items · "
                 f"{total_mass:.0f} g estimated mass · ${total_cost:,.0f} estimated component cost** "
                 f"(prototype quantities, excludes assembly labour)\n\n")
        fh.write("| Item | Qty | Material | Source | Mass (g) | Cost (USD) |\n|---|---:|---|---|---:|---:|\n")
        for r in table:
            fh.write(f"| {r['item']} | {r['qty']} | {r['material']} | {r['source']} | {r['mass_g']:.1f} | "
                     f"{r['cost']:.2f} |\n")
    with open(os.path.join(out_dir, "spec.json"), "w", encoding="utf-8") as fh:
        json.dump(spec, fh, indent=2)
    with open(os.path.join(out_dir, "parts.json"), "w", encoding="utf-8") as fh:
        json.dump(parts, fh, indent=1)
    return spec, table


# =============================================================================
# 10. UI PANEL MODULE
#     Stored in the .blend as a registered text module ("rh_ui.py"), so the
#     panel, grip presets and click-to-isolate come back every time the file
#     is opened (Blender asks you to allow scripts once).
# =============================================================================

UI_MODULE_SOURCE = r'''
import bpy, json
from bpy.app.handlers import persistent

RIG = "RH_Rig"
_preset_cache = []
_state = {"last": None, "busy": False}


def rig():
    return bpy.data.objects.get(RIG)


def preset_items(self, context):
    r = rig()
    names = list(json.loads(r["rh_presets"]).keys()) if r and "rh_presets" in r else ["Open"]
    _preset_cache[:] = [(n, n, f"Snap the hand into the {n} grasp") for n in names]
    return _preset_cache


def apply_preset(name):
    r = rig()
    if not r:
        return
    for key, value in json.loads(r["rh_presets"]).get(name, {}).items():
        r[key] = value
    r.update_tag()
    for area in (bpy.context.screen.areas if bpy.context.screen else []):
        area.tag_redraw()


def on_preset(self, context):
    apply_preset(self.rh_preset)


class RH_OT_apply_preset(bpy.types.Operator):
    bl_idname = "tendra.apply_preset"
    bl_label = "Apply Grasp"
    bl_options = {"REGISTER", "UNDO"}
    name: bpy.props.StringProperty()

    def execute(self, context):
        apply_preset(self.name)
        return {"FINISHED"}


class RH_OT_regenerate(bpy.types.Operator):
    """Rebuild the whole hand from the embedded generator with the parameters below"""
    bl_idname = "tendra.regenerate"
    bl_label = "Regenerate Hand"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        text = bpy.data.texts.get("hand_generator.py")
        if not text:
            self.report({"ERROR"}, "hand_generator.py text block not found")
            return {"CANCELLED"}
        sc = context.scene
        params = {"hand_scale": sc.rh_gen_scale, "finger_length_ratio": sc.rh_gen_finger_length,
                  "palm_width_mm": sc.rh_gen_palm_width, "num_fingers": sc.rh_gen_fingers}
        ns = {"__name__": "tendra_regenerate"}
        exec(compile(text.as_string(), "hand_generator.py", "exec"), ns)
        ns["build"](params)
        return {"FINISHED"}


def _spec_lines(obj):
    lines = [obj.get("rh_name", obj.name), f"Material: {obj.get('rh_material', '-')}"]
    if "rh_mass_g" in obj:
        lines.append(f"Mass: {obj['rh_mass_g']:.2f} g   Cost: ${obj.get('rh_cost_usd', 0):.2f}")
    if "rh_dof" in obj:
        lines.append(f"DOF: {obj['rh_dof']}")
    lines.append(f"Assembly: {obj.get('rh_assembly', '-')}")
    return lines


@persistent
def isolate_handler(scene, depsgraph=None):
    r = rig()
    if r is None or _state["busy"]:
        return
    try:
        active = bpy.context.view_layer.objects.active
    except Exception:
        return
    enabled = bool(r.get("isolate", False))
    target = active if (enabled and active is not None and active.get("rh_part")) else None
    if target is _state["last"]:
        return
    _state["busy"] = True
    try:
        for ob in bpy.data.objects:
            if ob.get("rh_part"):
                want = 0.0 if (target is None or ob is target) else 1.0
                if ob.get("dim") != want:
                    ob["dim"] = want
                    ob.update_tag()
            elif "rh_label_for" in ob:
                want = 0 if (target is None or ob["rh_label_for"] == target.name) else 1
                if ob.get("iso_hide") != want:
                    ob["iso_hide"] = want
        callout = bpy.data.objects.get("RH_SpecCallout")
        if callout:
            callout.hide_viewport = callout.hide_render = target is None
            if target is not None:
                callout.data.body = "\n".join(_spec_lines(target))
                cam = scene.camera
                right = cam.matrix_world.col[0].xyz.normalized() if cam else target.matrix_world.col[0].xyz
                corners = [target.matrix_world @ __import__("mathutils").Vector(c) for c in target.bound_box]
                centre = sum(corners, __import__("mathutils").Vector()) / 8.0
                callout.location = centre + right * 0.03 * r.get("gen_hand_scale", 1.0)
        _state["last"] = target
    finally:
        _state["busy"] = False


class RH_PT_main(bpy.types.Panel):
    bl_label = "TENDRA-H1 Robotic Hand"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "TENDRA"

    def draw(self, context):
        layout, r, sc = self.layout, rig(), context.scene
        if r is None:
            layout.label(text="Run hand_generator.py first", icon="ERROR")
            return
        box = layout.box()
        box.label(text="Assembly", icon="MOD_EXPLODE")
        box.prop(r, '["explode"]', text="Explode Factor", slider=True)
        row = box.row(align=True)
        row.prop(r, '["show_labels"]', text="Show Labels", toggle=True)
        row.prop(r, '["technical_view"]', text="Technical View", toggle=True)
        box.prop(r, '["label_detail"]', text="Label Detail (0 key / 1 all)")

        box = layout.box()
        box.label(text="Grasp", icon="HAND")
        box.prop(sc, "rh_preset", text="Preset")
        grid = box.grid_flow(columns=3, align=True)
        for name in json.loads(r["rh_presets"]).keys():
            grid.operator("tendra.apply_preset", text=name).name = name
        col = box.column(align=True)
        for key in r.keys():
            if key.startswith("curl_"):
                col.prop(r, f'["{key}"]', text=key[5:].title() + " Curl", slider=True)
        col.prop(r, '["thumb_curl"]', text="Thumb Curl", slider=True)
        col.prop(r, '["thumb_opposition"]', text="Thumb Opposition", slider=True)
        col.prop(r, '["spread"]', text="Spread", slider=True)
        col.prop(r, '["palm_cup"]', text="Palm Cup", slider=True)
        col = box.column(align=True)
        col.prop(r, '["wrist_pitch"]', text="Wrist Pitch °")
        col.prop(r, '["wrist_yaw"]', text="Wrist Yaw °")

        box = layout.box()
        box.label(text="Inspect", icon="VIEWZOOM")
        box.prop(r, '["isolate"]', text="Click-to-Isolate", toggle=True)
        ob = context.active_object
        if ob is not None and ob.get("rh_part"):
            for line in _spec_lines(ob):
                box.label(text=line)
        else:
            box.label(text="Select any part to inspect it")

        box = layout.box()
        box.label(text="Generator", icon="MODIFIER")
        col = box.column(align=True)
        col.prop(sc, "rh_gen_scale")
        col.prop(sc, "rh_gen_finger_length")
        col.prop(sc, "rh_gen_palm_width")
        col.prop(sc, "rh_gen_fingers")
        box.operator("tendra.regenerate", icon="FILE_REFRESH")


CLASSES = (RH_OT_apply_preset, RH_OT_regenerate, RH_PT_main)


def register():
    for cls in CLASSES:
        if not hasattr(bpy.types, cls.__name__):
            bpy.utils.register_class(cls)
    S = bpy.types.Scene
    S.rh_preset = bpy.props.EnumProperty(name="Grasp Preset", items=preset_items, update=on_preset)
    S.rh_gen_scale = bpy.props.FloatProperty(name="Hand Scale", default=1.0, min=0.5, max=2.0)
    S.rh_gen_finger_length = bpy.props.FloatProperty(name="Finger Length Ratio", default=1.0, min=0.6, max=1.6)
    S.rh_gen_palm_width = bpy.props.FloatProperty(name="Palm Width (mm)", default=84.0, min=60.0, max=120.0)
    S.rh_gen_fingers = bpy.props.IntProperty(name="Fingers", default=4, min=1, max=6)
    for h in list(bpy.app.handlers.depsgraph_update_post):
        if getattr(h, "__name__", "") == "isolate_handler":
            bpy.app.handlers.depsgraph_update_post.remove(h)
    bpy.app.handlers.depsgraph_update_post.append(isolate_handler)


register()
'''


def install_ui(ctx):
    text = bpy.data.texts.get("rh_ui.py") or bpy.data.texts.new("rh_ui.py")
    text.from_string(UI_MODULE_SOURCE)
    text.use_module = True
    # Keep a copy of this generator inside the .blend for the Regenerate button.
    src = None
    if "__file__" in globals() and os.path.isfile(globals()["__file__"]):
        with open(globals()["__file__"], encoding="utf-8") as fh:
            src = fh.read()
    gen = bpy.data.texts.get("hand_generator.py")
    if src:
        gen = gen or bpy.data.texts.new("hand_generator.py")
        gen.from_string(src)
    try:
        exec(compile(UI_MODULE_SOURCE, "rh_ui.py", "exec"), {"__name__": "rh_ui"})
        sc = bpy.context.scene
        sc.rh_gen_scale = ctx.params["hand_scale"]
        sc.rh_gen_finger_length = ctx.params["finger_length_ratio"]
        sc.rh_gen_palm_width = ctx.params["palm_width_mm"]
        sc.rh_gen_fingers = int(ctx.params["num_fingers"])
    except Exception as exc:          # UI registration is optional in background mode
        print("UI registration skipped:", exc)

    # Spec callout used by click-to-isolate.
    cu = bpy.data.curves.new("RH_SpecCallout", "FONT")
    cu.size = mm(4.5)
    font = _font("DejaVuSansMono.woff2")
    if font:
        cu.font = font
    cu.materials.append(ctx.mats["label"])
    ob = bpy.data.objects.new("RH_SpecCallout", cu)
    ctx.coll["labels"].objects.link(ob)
    ob.hide_viewport = ob.hide_render = True
    if bpy.context.scene.camera:
        con = ob.constraints.new("DAMPED_TRACK")
        con.target = bpy.context.scene.camera
        con.track_axis = "TRACK_Z"


# =============================================================================
# 11. BUILD
# =============================================================================

class BuildContext:
    def __init__(self, params):
        self.params = params
        self.parts = []
        self.coll = {}
        self.mats = {}
        self.rig = None

    def assembly_collection(self, name):
        key = f"asm_{name}"
        if key not in self.coll:
            c = bpy.data.collections.new(f"RH {name}")
            self.coll["parts"].children.link(c)
            self.coll[key] = c
        return self.coll[key]


def clear_previous():
    """Remove everything a previous run created (idempotent re-runs)."""
    for ob in list(bpy.data.objects):
        if ob.name.startswith("RH_"):
            bpy.data.objects.remove(ob, do_unlink=True)
    for coll in list(bpy.data.collections):
        if coll.name.startswith("RH") or coll.name == PRODUCT_NAME:
            bpy.data.collections.remove(coll)
    for datablocks in (bpy.data.meshes, bpy.data.curves, bpy.data.armatures, bpy.data.materials,
                       bpy.data.lights, bpy.data.cameras, bpy.data.worlds, bpy.data.node_groups):
        for block in list(datablocks):
            if block.name.startswith("RH_") and block.users == 0:
                datablocks.remove(block)


def build(params=None, reports_dir=None):
    """Generate the complete hand. Returns the BuildContext."""
    global _S
    p = dict(DEFAULT_PARAMS)
    p.update(params or {})
    p["num_fingers"] = int(max(1, min(6, p["num_fingers"])))
    _S = 0.001 * p["hand_scale"]

    scene = bpy.context.scene
    if scene.get("rh_built"):
        clear_previous()
    else:
        # First run in a fresh file: remove the default cube/camera/light.
        for ob in list(bpy.data.objects):
            if ob.name in ("Cube", "Camera", "Light"):
                bpy.data.objects.remove(ob, do_unlink=True)
    scene["rh_built"] = True
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "MILLIMETERS"

    ctx = BuildContext(p)
    root = bpy.data.collections.new(PRODUCT_NAME)
    scene.collection.children.link(root)
    for key, name in (("rig", "RH Rig"), ("parts", "RH Parts"), ("labels", "RH Labels"), ("studio", "RH Studio")):
        c = bpy.data.collections.new(name)
        root.children.link(c)
        ctx.coll[key] = c

    compute_layout(ctx)
    ctx.dorsal_y = max(ch.dims["R"][0] for ch in ctx.chains)
    build_armature(ctx)
    build_materials(ctx)
    for ch in ctx.chains:
        build_finger(ctx, ch)
    build_thumb(ctx)
    build_palm(ctx)
    build_wrist(ctx)
    build_forearm(ctx)
    build_explode_drivers(ctx)
    build_world(ctx)
    build_studio(ctx)
    cams = build_cameras(ctx)
    bpy.context.view_layer.update()
    build_labels(ctx, cams["labeled"])
    install_ui(ctx)
    bpy.context.view_layer.update()

    scene.render.fps = 30
    try:
        scene.render.engine = "CYCLES"
    except TypeError:
        pass
    if reports_dir:
        write_reports(ctx, reports_dir)
    else:
        compute_bom(ctx)       # still stamp mass/cost onto parts for inspection
    print(f"[{PRODUCT_NAME}] built {len(ctx.parts)} parts, {len(ctx.rig.data.bones)} bones, "
          f"{len(ctx.chains)} fingers + thumb")
    return ctx


def _parse_cli():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    flags = {"--scale": "hand_scale", "--finger-length": "finger_length_ratio", "--palm-width": "palm_width_mm",
             "--fingers": "num_fingers", "--explode-dist": "explode_distance_mm"}
    params, save, reports = {}, None, None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in flags:
            params[flags[a]] = float(argv[i + 1])
            i += 2
        elif a == "--save":
            save = argv[i + 1]; i += 2
        elif a == "--reports":
            reports = argv[i + 1]; i += 2
        elif a == "--no-bevel":
            params["bevel"] = False; i += 1
        else:
            i += 1
    return params, save, reports


if __name__ == "__main__":
    params, save_path, reports_dir = _parse_cli()
    build(params, reports_dir)
    if save_path:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(save_path))
