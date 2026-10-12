#!/usr/bin/env python3
"""Measure the jaw opening in saved wrist-camera frames (runs on the lab host, reads files only).

For each frame <lab monotonic ns>.jpg: the gap, in pixels, between the two blue jaws in a band
near the bottom of the image, and the frame-to-frame change. Output: JSON list on stdout.
"""
import json
import os
import sys

import cv2
import numpy as np

d = sys.argv[1]
out, prev = [], None
for f in sorted(os.listdir(d), key=lambda x: int(x[:-4])):
    img = cv2.imread(os.path.join(d, f))
    if img is None:
        continue
    h, w = img.shape[:2]
    band = cv2.cvtColor(img[int(0.80 * h):int(0.95 * h)], cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(band, (95, 90, 60), (125, 255, 255)).mean(axis=0) > 60  # column is "jaw" if mostly blue
    cols = np.flatnonzero(blue)
    gap = None
    if len(cols) > 1:  # the opening is the widest run of non-jaw columns between jaw columns (the jaws need not be centred)
        steps = np.diff(cols)
        gap = int(steps.max()) if steps.max() > 3 else 0
    small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (160, 120)).astype(np.int16)
    motion = None if prev is None else float(np.abs(small - prev).mean())
    prev = small
    out.append({"t": int(f[:-4]), "gap_px": gap, "motion": motion, "w": w})
json.dump(out, sys.stdout)
