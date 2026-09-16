# Higgsfield prompt pack — TENDRA-H1

Use the Blender renders as **reference images** so the AI output stays true to the real design.
Pick one reference per prompt. Keep the cyan accent to one colour; don't let the model invent new colours.

## 1. LinkedIn hero still (image → image, 4:5)
**Reference:** `output/renders/01_hero_assembled_4x5.png`

> Product photograph of a matte black carbon-fibre robotic hand with exposed mechanical skeleton, brushed steel pins and small glowing cyan ring lights at each joint, standing on a slim titanium display post. Dark studio, soft cool key light from the upper right, thin rim light outlining the fingers, faint cyan haze in the background, shallow depth of field, 85mm lens, high-end industrial design photography. Keep the geometry identical to the reference.

## 2. Exploded-view "engineering drama" still (16:9)
**Reference:** `output/renders/03_exploded_16x9.png`

> Exploded technical assembly of a robotic hand floating in darkness: every finger link, pin, bearing and screw separated along its axis in perfect alignment, fine volumetric light rays, tiny cyan light guides glowing, subtle dust particles, cinematic keynote-reveal style, ultra sharp, no text.

## 3. Blueprint-to-reality transition (video, 5–8 s)
**Start frame:** `output/renders/04_labeled_technical_16x9.png` · **End frame:** `output/renders/03_exploded_16x9.png`

> Camera slowly pushes in as the navy blueprint wireframe fills in with real materials: matte black links, brushed steel hardware, cyan joint lights powering on one by one from the wrist to the fingertips. Smooth, precise, no morphing of the parts.

## 4. Grip moment (video, 4 s)
**Start frame:** `output/renders/05_grasp_open.png` · **End frame:** `output/renders/05_grasp_power_grip.png`

> The robotic hand closes smoothly into a power grip, tendons tightening, joint lights pulsing once as each finger reaches position. Locked-off camera, dark studio, product commercial pacing.

## 5. Human–robot context shot (image, 16:9)
**Reference:** `output/renders/02_dorsal_assembled_4x5.png`

> An engineer's hand in a nitrile glove reaching toward the TENDRA-H1 robotic hand on a lab bench under dim cool light, oscilloscope glow in the soft-focus background, documentary tone, realistic, no text or logos.

### Tips
- Keep "no text, no logos" in the prompt; AI-generated lettering on a technical piece looks fake.
- If fingers gain or lose joints, lower the creativity/strength and re-use the reference.
- Post the AI shot as the hook image, and put the real Blender renders, the live viewer link and the repo in the carousel or comments. The provable engineering is what makes the post credible.
