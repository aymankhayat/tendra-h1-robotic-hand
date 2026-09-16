# LinkedIn & CV kit: TENDRA-H1

Copy-ready text for your profile, posts and CV. Everything here has been through a humanizing pass: no em dashes, no AI-slop vocabulary, no structural tells. Checks are local heuristics, not an AI detector.
Voice: technical deep-dive, no emoji. Nothing here is posted for you. You post it.

Live site: https://aymankhayat.github.io/tendra-h1-robotic-hand/
Repo: https://github.com/aymankhayat/tendra-h1-robotic-hand
Case study: https://github.com/aymankhayat/tendra-h1-robotic-hand/blob/main/CASE_STUDY.md

---

## 1. Launch post (text + video)

**Hooks considered**
1. #10 The Receipt: "My collision checker flagged 64 clashes. Most weren't real." *(shipping this one: a concrete number, an honest failure, and it leads into real engineering)*
2. #17 Time Anchor: "Rendering a finger curl took 85 degrees in Blender and 8.5 in the browser."
3. #2 Number Reveal: "309 parts, one Python script, zero hand modelling."

**Attach:** `output/animation/tendra_h1_showcase_4x5_0001-0240.mp4` (upload natively; 4:5 takes the most feed space).

```
I built a robotic hand from one Python script. My collision checker flagged 64 clashes on the first run.

Most weren't real. Working out which ones were took longer than generating the hand.

TENDRA-H1 is a 25-joint, tendon-driven hand built entirely through Blender's Python API: 309 parts, 27 bones and 1,095 drivers. Nothing is modelled by hand, so I had to prove it's mechanically sound.

The checker tests every part against every other part with BVH trees, at rest and in six grasp poses.

Version 1 used the nearest face normal to decide what counts as "inside". Thin light rings on the bearings broke it. 64 flags.

Version 2 cast three rays and counted crossings. That dropped it to 16, and now most were real: gearmotors clipping the forearm rails by 1.2 mm, and a thumb passing 3.6 mm through the index finger in a fist.

Version 3 ran the ray test per closed solid. Links built from touching blocks share internal faces, and those faces were flipping the count. Zero false flags.

Then I re-ran a pose I knew was broken. It still caught the thumb at 2.2 mm.

A checker that reports zero means nothing until you've watched it fail.

When you write verification code, how do you prove the checker itself works?

(Live 3D viewer and code in the first comment.)

#Blender #Python #Robotics
```

**First comment (post it yourself right after publishing; links in the body get suppressed):**
```
Try it in the browser (explode it, pose it, click any part): https://aymankhayat.github.io/tendra-h1-robotic-hand/
Code and full case study: https://github.com/aymankhayat/tendra-h1-robotic-hand
```

```
POST READY
hook: #10 The Receipt
length: 1,321 characters
humanizer: 94/100 PASS (burstiness 99, specificity 100, slop 100, fingerprint 100, voice 90)
```

## 2. Follow-up posts (one a week; drafted on request)

Each stands alone. Don't reference the launch post.

| Week | Hook | Angle |
|---|---|---|
| 2 | #17 Time Anchor: "A finger curl was 85 degrees in Blender and 8.5 in the browser." | Drivers don't survive glTF, so each control became a clip; the exporter sampled every bone into every clip and three.js averaged them. |
| 3 | #3 Mistake Confession: "I guessed the thumb's opposition angles. It swung the wrong way." | Forward-kinematics tuner, why a 10 degree grid is 17 mm of error at the fingertip, pattern search, 441 configs per grasp. |
| 4 | #11 Myth Bust: "Dark renders aren't a material problem." | 0.24% albedo still came out white; a lights-off test found the lights were 100x too strong at real scale. |

## 3. Carousel (document post): copy for approval

10 slides, 1080x1350. Your name sits small in a corner of every slide. **Approve or edit this copy and I'll build the PDF.**

1. **Cover:** "One script. 309 parts." / A robotic hand with zero hand modelling. *(hero image)*
2. **Stake:** "Code as the only source of truth." / Change 4 numbers and the whole hand rebuilds, rig included.
3. **Anatomy first:** "Built from real hand data." / Link lengths from adult anthropometry; hyperextension capped at 5 to 15 degrees.
4. **The thumb:** "A thumb that actually opposes." / Its own CMC joint, with angles found by forward-kinematics search.
5. **Exploded view:** "Parts move by kinematic depth." / Fingertips travel furthest; pins slide out along their hinge axis.
6. **Verification:** "64 flags, then 0." / Ray-parity per solid island; a known-bad pose still gets caught at 2.2 mm.
7. **Engineering outputs:** "457 g. $1,441." / Mass and cost measured from the geometry; dimensioned drawings. *(ortho image)*
8. **Web:** "Rig controls, rebuilt for glTF." / 11 baked clips; fixed curls rendering at 10% of their angle.
9. **Recap:** Generator, rig, thumb search, collision checks, BOM and cost, live 3D site.
10. **CTA:** "Explode it yourself." / Link in the comments.

Text above the carousel:
```
I wanted to know how far one Python script could take a mechanical design. This is the answer, slide by slide.

Live viewer and code in the comments.
```

## 4. LinkedIn profile

### Projects section
**Name:** TENDRA-H1: parametric robotic hand · **Date:** Sep 2026 · **URL:** the live site

```
A 25-joint, tendon-driven robotic hand generated entirely from Python in Blender. Change the hand scale, finger length, palm width or finger count and the script rebuilds all 309 parts, the rig and its drivers, the exploded view and a bill of materials with mass and cost.

I wanted to see how far one script could go: from anatomy tables to a verified, costed, web-ready assembly. What I built:
- A 2,200-line generator for the geometry, a 27-bone rig with human joint limits, and six grasp presets (open, fist, pinch, point, power grip, OK sign)
- An opposable thumb whose joint angles I found with a forward-kinematics search
- Automated BVH collision checks across every pose: 0 interferences, 1 connected assembly
- Mass (457 g) and cost ($1,441) estimated from the model geometry, plus a product datasheet
- A three.js site where you can explode the hand, pose it and click any part for its specs
```

### Featured section (in this order)
1. **Link:** the live site (the rich preview card shows automatically)
2. **Post:** your launch post with the 4:5 video, once it's live
3. **Media:** `output/TENDRA-H1_datasheet.pdf` titled "TENDRA-H1 datasheet"
4. **Link:** the GitHub repo

### Skills to add (pin the top 3 that match the role you're applying for)
Blender Python API (bpy) · Procedural Modeling · Rigging · Kinematics · Python · Computational Geometry · three.js · glTF · Product Rendering (Cycles) · Technical Documentation · Cost Estimation · Design Verification

### Headline options (swap in your real title)
- `{your role} | Robotics and mechanical design in code | Blender Python, kinematics, three.js`
- `{your role} | I build parametric mechanisms: generate, verify, render, ship to the web`

## 5. CV bullets (pick the block for the job)

**Mechanical / robotics**
- Designed a 25-joint tendon-driven robotic hand (309 parts, 457 g, $1,441 estimated BOM) with human range-of-motion limits, a 2-DOF wrist and an opposable thumb tuned by forward-kinematics search.
- Built automated BVH interference and floating-part checks across 6 grasp poses; found and fixed 4 real clashes (up to 3.6 mm) and reached zero interferences.
- Generated mass and cost estimates directly from CAD geometry (volume x density, MJF and CNC pricing) and a one-page product datasheet with dimensioned orthographic drawings.

**Technical artist / 3D**
- Wrote a 2,200-line Blender Python generator that builds a fully rigged robotic hand from 4 parameters: 27 bones, 1,095 drivers, exploded view, camera-facing labels and a blueprint shading mode.
- Produced Cycles product renders, dimensioned orthographic views and an EEVEE explode-and-reassemble animation from a headless pipeline, including light calibration at real-world scale.

**Software / creative developer**
- Built a headless Python pipeline (1,100 lines) that verifies, renders and exports a Blender model to glTF, baking 11 rig controls into independently scrubbable animation clips.
- Shipped a three.js portfolio site with a live 3D lab (explode, grasp presets, click-to-inspect part specs); fixed a clip-averaging bug that rendered finger curls at 10% of their real angle.

**Student / internship (one line)**
- Built and deployed TENDRA-H1, a parametric robotic hand generated from one Blender Python script, with automated collision checks, a BOM with cost estimate and an interactive web viewer (aymankhayat.github.io/tendra-h1-robotic-hand).

## 6. Interview prep
Use [CASE_STUDY.md](../CASE_STUDY.md). Know these four stories cold: thumb search, checker iterations, glTF clip averaging, light calibration. Also be ready for "what would you change?": the 3-finger variant's Point preset, and a cost estimate that excludes labour.

## Posting notes
- Links go in the first comment, never the post body.
- After the site preview image changes, refresh LinkedIn's cache at https://www.linkedin.com/post-inspector/ (paste the site URL).
- Reply to real comments within the first hour. Answer questions in the reply itself, not in DMs.
