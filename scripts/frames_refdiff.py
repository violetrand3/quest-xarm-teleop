#!/usr/bin/env python3
"""How much does each saved frame differ from the first second of the recording? (reads files only)

For <dir>/<lab monotonic ns>.jpg: mean absolute grey-level difference from the median of the first
frames, over the whole (downsized) image. A camera or scene that moves and comes back shows a
bump that returns to the baseline. Output: JSON list of {t, diff} on stdout.
"""
import json
import os
import sys

import cv2
import numpy as np

d = sys.argv[1]
names = sorted((f for f in os.listdir(d) if f.endswith(".jpg")), key=lambda x: int(x[:-4]))
small = []
for f in names:
    img = cv2.imread(os.path.join(d, f), cv2.IMREAD_GRAYSCALE)
    if img is not None:
        small.append((int(f[:-4]), cv2.GaussianBlur(cv2.resize(img, (320, 240)), (5, 5), 0).astype(np.int16)))
if not small:
    print("[]")
    sys.exit(0)
t0 = small[0][0]
ref = np.median(np.stack([s for t, s in small if t - t0 <= 1_000_000_000]), axis=0)
print(json.dumps([{"t": t, "diff": round(float(np.abs(s - ref).mean()), 3)} for t, s in small]))
