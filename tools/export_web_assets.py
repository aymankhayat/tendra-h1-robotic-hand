"""Copy site media into web/assets: renders and Higgsfield images become web-sized JPEGs.

Runs inside Blender (uses its image API, so no Pillow needed):
    blender -b --factory-startup -P tools/export_web_assets.py
"""
import os
import shutil

import bpy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REN = os.path.join(ROOT, "output", "renders")
AI = os.path.join(ROOT, "output", "higgsfield")
DST = os.path.join(ROOT, "web", "assets")
os.makedirs(DST, exist_ok=True)

IMAGES = {  # destination name: (source path, max width)
    "ai_hero_rings.jpg": (os.path.join(AI, "hero_rings.png"), 1900),
    "ai_lab_engineer.jpg": (os.path.join(AI, "lab_engineer.png"), 1500),
    "ai_exploded_drama.jpg": (os.path.join(AI, "exploded_drama.png"), 1600),
    "hero_assembled.jpg": (os.path.join(REN, "01_hero_assembled_4x5.png"), 1000),
    "labeled_technical.jpg": (os.path.join(REN, "04_labeled_technical_16x9.png"), 1600),
    "variant_3finger.jpg": (os.path.join(REN, "07_variant_3finger_hero_4x5.png"), 900),
    "variant_3finger_exploded.jpg": (os.path.join(REN, "07_variant_3finger_exploded_4x5.png"), 900),
}
for view in ("front", "side", "top"):
    IMAGES[f"ortho_{view}.jpg"] = (os.path.join(REN, f"06_ortho_{view}.png"), 900)
for grasp in ("open", "fist", "pinch", "point", "power_grip", "ok_sign"):
    IMAGES[f"grasp_{grasp}.jpg"] = (os.path.join(REN, f"05_grasp_{grasp}.png"), 720)

scene = bpy.context.scene
settings = scene.render.image_settings
settings.file_format = "JPEG"
settings.quality = 84
settings.color_mode = "RGB"
for name, (src, max_w) in IMAGES.items():
    if not os.path.exists(src):
        print("missing", src)
        continue
    img = bpy.data.images.load(src)
    w, h = img.size
    if w > max_w:
        img.scale(max_w, round(h * max_w / w))
    img.save_render(os.path.join(DST, name), scene=scene)
    print("wrote", name, img.size[:])

for src, name in ((os.path.join(ROOT, "output", "animation", "tendra_h1_showcase_16x9_0001-0240.mp4"), "showcase_16x9.mp4"),
                  (os.path.join(ROOT, "output", "TENDRA-H1_datasheet.pdf"), "TENDRA-H1_datasheet.pdf")):
    if os.path.exists(src):
        shutil.copyfile(src, os.path.join(DST, name))
        print("copied", name)
