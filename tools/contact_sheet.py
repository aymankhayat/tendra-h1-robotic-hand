"""Tile PNGs into one review sheet: blender -b -P tools/contact_sheet.py -- out.png cols cell a.png b.png ..."""
import bpy, sys, numpy as np
args = sys.argv[sys.argv.index("--") + 1:]
out, cols, cell = args[0], int(args[1]), int(args[2])
files = args[3:]
rows = (len(files) + cols - 1) // cols
sheet = np.zeros((rows * cell, cols * cell, 4), dtype=np.float32); sheet[..., 3] = 1
for i, f in enumerate(files):
    img = bpy.data.images.load(f)
    w, h = img.size
    px = np.array(img.pixels[:], dtype=np.float32).reshape(h, w, 4)
    ys = (np.arange(cell) * h / cell).astype(int); xs = (np.arange(cell) * w / cell).astype(int)
    r, c = divmod(i, cols)
    sheet[(rows - 1 - r) * cell:(rows - r) * cell, c * cell:(c + 1) * cell] = px[ys][:, xs]
res = bpy.data.images.new("sheet", cols * cell, rows * cell, alpha=True)
res.pixels = sheet.ravel()
res.filepath_raw = out; res.file_format = "PNG"; res.save()
