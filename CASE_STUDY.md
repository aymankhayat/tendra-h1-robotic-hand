# Case study: TENDRA-H1

**Ayman Khayat** · [LinkedIn](https://www.linkedin.com/in/ayman-khayat-350b4b335) · [Live demo](https://aymankhayat.github.io/tendra-h1-robotic-hand/)

## Brief

Design a fully articulated, tendon-driven robotic hand whose **only source of truth is code**. The same script must produce the geometry, the posable rig, an exploded view, part labels, a bill of materials with mass and cost, verified-clean poses, renders and a web viewer. Change a parameter and everything must regenerate consistently.

## Outcome at a glance

| Metric | Value |
|---|---|
| Core code | ~3,300 lines of Python (`hand_generator.py` 2,220 · `pipeline.py` 1,100) |
| Generated parts | 309 (64 unique BOM line items) |
| Rig | 27 bones, 25 joints, 1,095 drivers, 10 tendon actuators (under-actuated) |
| Envelope / mass / cost | 298 × 152 × 69 mm · 457 g · $1,441 (prototype component cost) |
| Verification | 0 interferences at rest and across 6 grasp presets, 1 connected assembly |
| Web export | 2.7 MB GLB, ~80k triangles, 11 independently scrubbable animation clips |

---

## Challenge 1: a thumb that genuinely opposes

**Problem.** The first opposition mapping was a guess, and it swung the thumb *outward* in Pinch and Fist. A thumb that just copies the fingers' motion fails the one thing a hand model is judged on.

**Approach.**
- Rebuilt the thumb on its own CMC ball joint, with a rest axis and pad direction rotated out of the palm plane. Its flexion axis ends up roughly 60° from the fingers'.
- Wrote a forward-kinematics tuner (`tools/tune_thumb.py`) using the generated bone rest matrices. It scores thumb-pad to index-pad distance, opposed pad normals and link-to-link penetration.
- A coarse grid search (10° steps ≈ 17 mm of tip travel at a 100 mm lever arm) was too blunt, so I added a continuous pattern-search refinement and a sweep over CMC placement.

**Result.** The CMC opposition mapping settled at 58° / 50° / 70°. A second tuner (`tools/tune_presets.py`, 441 configurations per grasp) found thumb values with **zero penetration** for every preset. The remaining pad gap is the rigid link bodies meeting, which is physically correct for a robot finger.

## Challenge 2: verification you can trust

**Problem.** "No overlapping or floating parts" needed proof, not eyeballing, across 309 parts and 6 poses.

**Approach and iterations.**
1. BVH checks with a nearest-face-normal inside test flagged **64** "interferences" at rest. Most were false positives from thin annular light rings.
2. Switched to three-ray parity voting, which is independent of normal orientation: **16** flagged, now mostly real.
3. The remaining false hits came from composite link meshes (touching cheeks plus web), where rays cross shared internal faces. Running parity per closed solid island brought false positives to **0**.
4. Floating-part check: a contact graph where BVH triangle overlap counts as contact, so a pin through a bore connects correctly. Result: one connected component.
5. A **sensitivity test** re-ran a known-bad pose. It still reported the thumb passing 2.2 mm through the index finger, so the zero result isn't a blind checker.

**Real defects it caught and I fixed:** outer gearmotors clipping the forearm rails (1.23 mm), plate screws inside the wiring manifold (1.2 mm), the CMC socket cutting the carpal frame (0.78 mm), and the thumb passing 3.6 mm through the index finger in Fist.

## Challenge 3: Blender rig → interactive web

**Problem.** Blender custom-property drivers don't exist in glTF, so the rig's controls would be lost on export.

**Approach.** Each control (explode, four finger curls, spread, palm cup, thumb curl, thumb opposition, wrist pitch, wrist yaw) is baked into its own NLA track over frames 0–60. In three.js, each clip's `time` is set directly from a slider.

**The bug.** The exporter samples *every* bone into *every* clip (81 tracks each), so the eleven clips averaged each other: a full index curl rendered at **8.5° instead of 85°**. The fix was to strip static tracks at load, which leaves 3 tracks per finger clip, so controls combine independently.

**Also fixed:** exported anisotropic metals blew out to white in three.js, because the extension needs tangents the mesh doesn't carry. I disabled anisotropy at load and rebalanced environment intensity.

## Challenge 4: photoreal materials at true scale

**Problem.** The first Cycles renders were washed out even with near-black albedo (0.24%).

**Diagnosis.** A controlled test (lights off, then lights ÷100) showed the lights were about **100× too strong** for a 0.3 m subject. The bundled studio HDRI also added a colour cast to every glossy surface.

**Fix.** Recalibrated the three-point rig, desaturated the HDRI to reflections only, and used one emissive accent colour with restrained bloom.

---

## Engineering decisions worth discussing in an interview

- **Why tendon-driven:** actuators move to the forearm, fingers stay slim and the hand stays light (457 g). The cost is coupled, under-actuated flexion, and I chose that trade deliberately.
- **Exploded-view semantics:** each part's offset is its kinematic depth plus its local joint axis, applied as `delta_location` in rest world space, so the exploded view survives posing.
- **Mass and cost from geometry:** evaluated-mesh volume × material density, with catalogue masses for purchased motors. Cost uses per-cm³ MJF print pricing plus quoted prices for machined and off-the-shelf parts.
- **Honest limits:** the 3-finger variant's *Point* preset still interferes (its thumb values were tuned for four fingers), and the cost is a prototype estimate that excludes labour.

## Tools

Blender 5.2 (bpy, bmesh, mathutils BVH) · Cycles / EEVEE · Python · glTF 2.0 · three.js · headless Edge (PDF) · Higgsfield (concept imagery from the renders)
