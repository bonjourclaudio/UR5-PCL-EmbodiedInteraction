"""Image -> engraving strokes. Geometry only, no robot.

    load_gray(path)                      -> 2D float array 0..1 (0 = black)
    hatch_strokes(gray, width_mm, ...)   -> line marks [((x0, y0), (x1, y1), 1.0)] in mm
    dot_marks(gray, width_mm, ...)       -> dot marks  [((x, y), (x, y), darkness)] (stipple)
    newest_image(folder)                 -> path of the most recent image in a folder
    demo_image()                         -> a synthetic test picture (no webcam needed)
    write_svg(strokes, w, h, path)       -> preview of exactly what will be engraved

Canvas coordinates: mm, origin at the image's top-left, x to the right, y down.

The dev image has no OpenCV/Pillow guaranteed, so PNG and PGM/PPM are decoded
here with stdlib + numpy. If Pillow happens to be importable, it is used (then
JPEG works too). Otherwise convert JPEGs on the laptop first, e.g. on macOS:
    sips -s format png snapshot.jpg --out students/claudio/wax/inbox/snapshot.png
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

IMAGE_SUFFIXES = (".png", ".pgm", ".ppm", ".pnm", ".jpg", ".jpeg")


# ---------------------------------------------------------------- loading --
def load_gray(path) -> np.ndarray:
    """Grayscale image as float array, 0.0 = black, 1.0 = white."""
    path = Path(path)
    try:
        from PIL import Image  # optional, only if present in the image
        with Image.open(path) as im:
            return np.asarray(im.convert("L"), dtype=np.float64) / 255.0
    except ImportError:
        pass
    data = path.read_bytes()
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        rgb = _decode_png(data)
    elif data[:2] in (b"P2", b"P3", b"P5", b"P6"):
        rgb = _decode_pnm(data)
    else:
        raise ValueError(f"{path.name}: only PNG / PGM / PPM without Pillow — convert it to PNG first")
    return _to_gray(rgb)


def _to_gray(img: np.ndarray) -> np.ndarray:
    img = img.astype(np.float64) / 255.0
    if img.ndim == 2:
        return img
    if img.shape[2] in (2, 4):          # alpha: composite on white
        a = img[..., -1:]
        img = img[..., :-1] * a + (1 - a)
    if img.shape[2] == 1:
        return img[..., 0]
    return img[..., 0] * 0.299 + img[..., 1] * 0.587 + img[..., 2] * 0.114


def _decode_pnm(data: bytes) -> np.ndarray:
    tokens, pos = [], 2
    while len(tokens) < 3:               # width, height, maxval (skip comments)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos)
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(int(data[pos:end]))
        pos = end
    w, h, maxval = tokens
    ch = 3 if data[:2] in (b"P3", b"P6") else 1
    if data[:2] in (b"P5", b"P6"):
        dtype = ">u2" if maxval > 255 else np.uint8
        arr = np.frombuffer(data[pos + 1:], dtype=dtype, count=w * h * ch)
    else:
        arr = np.array(data[pos:].split()[:w * h * ch], dtype=np.int64)
    arr = (arr.astype(np.float64) * 255.0 / maxval).astype(np.uint8)
    return arr.reshape(h, w, ch) if ch == 3 else arr.reshape(h, w)


def _decode_png(data: bytes) -> np.ndarray:
    pos, idat, palette, ihdr = 8, [], None, None
    while pos < len(data):
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", chunk)
        elif kind == b"PLTE":
            palette = np.frombuffer(chunk, np.uint8).reshape(-1, 3)
        elif kind == b"IDAT":
            idat.append(chunk)
        elif kind == b"IEND":
            break
    w, h, depth, ctype, _, _, interlace = ihdr
    if interlace or depth not in (8, 16):
        raise ValueError("PNG: interlaced or <8-bit PNGs not supported — re-save as a plain 8-bit PNG")
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}[ctype]
    bpp = channels * depth // 8                       # bytes per pixel
    stride = w * bpp
    raw = zlib.decompress(b"".join(idat))
    out = np.zeros((h, stride), np.uint8)
    prev = np.zeros(stride, np.int64)
    for y in range(h):
        f = raw[y * (stride + 1)]
        line = np.frombuffer(raw, np.uint8, stride, y * (stride + 1) + 1).astype(np.int64)
        if f == 1:                                    # Sub: running sum per channel
            line = np.cumsum(line.reshape(w, bpp), axis=0).reshape(-1) % 256
        elif f == 2:                                  # Up
            line = (line + prev) % 256
        elif f in (3, 4):                             # Average / Paeth: sequential
            cur = line.tolist()
            up = prev.tolist()
            for i in range(stride):
                a = cur[i - bpp] if i >= bpp else 0
                b = up[i]
                if f == 3:
                    cur[i] = (cur[i] + ((a + b) >> 1)) & 255
                else:
                    c = up[i - bpp] if i >= bpp else 0
                    p = a + b - c
                    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                    pred = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                    cur[i] = (cur[i] + pred) & 255
            line = np.array(cur, np.int64)
        out[y] = line
        prev = line
    if depth == 16:
        out = out.reshape(h, w * channels, 2)[..., 0]  # keep the high byte
    img = out.reshape(h, w, channels)
    if ctype == 3:
        img = palette[img[..., 0]]
    return img


def newest_image(folder) -> Path:
    files = [p for p in Path(folder).glob("*") if p.suffix.lower() in IMAGE_SUFFIXES]
    if not files:
        raise FileNotFoundError(f"no image in {folder} — drop a PNG there or pass --image / --demo-image")
    return max(files, key=lambda p: p.stat().st_mtime)


def demo_image(size: int = 240) -> np.ndarray:
    """Test picture: a ring, a filled disc and a diagonal bar on white."""
    yy, xx = np.mgrid[0:size, 0:size] / size
    r = np.hypot(xx - 0.5, yy - 0.5)
    img = np.ones((size, size))
    img[(r > 0.38) & (r < 0.45)] = 0.0
    img[np.hypot(xx - 0.38, yy - 0.40) < 0.10] = 0.0
    img[(np.abs(xx - yy) < 0.04) & (r < 0.33)] = 0.0
    return img


# ---------------------------------------------------------------- strokes --
def _resample(gray: np.ndarray, rows: int, cols: int) -> np.ndarray:
    """Area-average resample to rows x cols (integral image)."""
    h, w = gray.shape
    s = np.zeros((h + 1, w + 1))
    s[1:, 1:] = gray.cumsum(0).cumsum(1)
    ys = np.linspace(0, h, rows + 1).astype(int)
    xs = np.linspace(0, w, cols + 1).astype(int)
    ys[1:] = np.maximum(ys[1:], ys[:-1] + 1).clip(max=h)
    xs[1:] = np.maximum(xs[1:], xs[:-1] + 1).clip(max=w)
    y0, y1, x0, x1 = ys[:-1, None], ys[1:, None], xs[None, :-1], xs[None, 1:]
    area = np.maximum((y1 - y0) * (x1 - x0), 1)
    return (s[y1, x1] - s[y0, x1] - s[y1, x0] + s[y0, x0]) / area


def hatch_strokes(gray: np.ndarray, width_mm: float, *, hatch_mm: float = 2.0,
                  pixel_mm: float = 1.0, threshold: float = 0.5,
                  min_len_mm: float = 1.5, invert: bool = False):
    """Horizontal hatch lines through the dark parts of the image.

    Every `hatch_mm` a row is sampled; consecutive dark samples (gray < threshold)
    along the row become one straight stroke. Rows alternate direction (snake)
    so travel moves stay short. Returns (strokes, width_mm, height_mm).
    """
    h, w = gray.shape
    height_mm = width_mm * h / w
    rows = max(1, int(height_mm / hatch_mm))
    cols = max(2, int(width_mm / pixel_mm))
    small = _resample(gray, rows, cols)
    dark = (small > threshold) if invert else (small < threshold)
    px = width_mm / cols
    strokes = []
    for r in range(rows):
        y = (r + 0.5) * height_mm / rows
        runs, c = [], 0
        line = dark[r]
        while c < cols:
            if line[c]:
                start = c
                while c < cols and line[c]:
                    c += 1
                if (c - start) * px >= min_len_mm:
                    runs.append(((start + 0.5) * px, (c - 0.5) * px))
            else:
                c += 1
        if r % 2:                                     # snake: right-to-left on odd rows
            runs = [(b, a) for a, b in reversed(runs)]
        strokes += [((a, y), (b, y), 1.0) for a, b in runs]
    return strokes, width_mm, height_mm


def _blur3(a: np.ndarray) -> np.ndarray:
    p = np.pad(a, 1, mode="edge")
    return sum(p[i:i + a.shape[0], j:j + a.shape[1]] for i in range(3) for j in range(3)) / 9.0


def dot_marks(gray: np.ndarray, width_mm: float, *, dot_mm: float = 2.5, gamma: float = 1.0,
              min_dark: float = 0.08, invert: bool = False, levels: bool = True,
              sharpen: float = 1.0):
    """Stipple / dot-work: error-diffusion (Floyd-Steinberg) on a dot_mm grid.

    Before dithering, on the dot grid: `levels` stretches the 2nd..98th
    percentile to full black..white (webcam pictures are mostly grey), and
    `sharpen` boosts edges (unsharp mask) so features survive the coarse grid.
    Darkness = 1 - gray (or gray with invert), shaped by gamma (>1 = fewer dots
    in the mid-tones). Each cell either gets a dot or not; dark regions get
    dense dots, light ones sparse. Each dot also carries its cell's darkness
    (0..1) so the caller can melt darker spots longer = bigger dots.
    Cells lighter than min_dark stay empty (clean background).
    Rows are visited in snake order. Returns (marks, width_mm, height_mm)
    with marks = [((x, y), (x, y), darkness)].
    """
    h, w = gray.shape
    height_mm = width_mm * h / w
    rows = max(1, int(round(height_mm / dot_mm)))
    cols = max(1, int(round(width_mm / dot_mm)))
    small = _resample(gray, rows, cols)
    if levels:
        lo, hi = np.percentile(small, (2, 98))
        if hi - lo > 1e-3:
            small = (small - lo) / (hi - lo)
    if sharpen:
        small = small + sharpen * (small - _blur3(small))
    dark = np.clip(small if invert else 1.0 - small, 0.0, 1.0) ** gamma
    err = dark.copy()
    marks = []
    for r in range(rows):
        order = range(cols) if r % 2 == 0 else range(cols - 1, -1, -1)
        step = 1 if r % 2 == 0 else -1                    # diffuse along the scan direction
        for c in order:
            old = err[r, c]
            on = old >= 0.5 and dark[r, c] >= min_dark
            q = old - (1.0 if on else 0.0)
            if on:
                marks.append((((c + 0.5) * width_mm / cols, (r + 0.5) * height_mm / rows),) * 2
                             + (float(dark[r, c]),))
            if 0 <= c + step < cols:
                err[r, c + step] += q * 7 / 16
            if r + 1 < rows:
                if 0 <= c - step < cols:
                    err[r + 1, c - step] += q * 3 / 16
                err[r + 1, c] += q * 5 / 16
                if 0 <= c + step < cols:
                    err[r + 1, c + step] += q * 1 / 16
    return marks, width_mm, height_mm


def write_svg(strokes, width_mm: float, height_mm: float, path, dot_r_mm: float = 1.0) -> Path:
    """Preview: strokes / dots in black (dot size ~ darkness), travel in light red."""
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_mm}mm" height="{height_mm}mm" '
             f'viewBox="0 0 {width_mm:.2f} {height_mm:.2f}">',
             f'<rect width="{width_mm:.2f}" height="{height_mm:.2f}" fill="#f3e6c4"/>']
    last = None
    for (x0, y0), (x1, y1), dark in strokes:
        if last:
            parts.append(f'<line x1="{last[0]:.2f}" y1="{last[1]:.2f}" x2="{x0:.2f}" y2="{y0:.2f}" '
                         f'stroke="#e88" stroke-width="0.15"/>')
        if (x0, y0) == (x1, y1):
            parts.append(f'<circle cx="{x0:.2f}" cy="{y0:.2f}" r="{dot_r_mm * (0.4 + 0.6 * dark):.2f}" '
                         f'fill="#222"/>')
        else:
            parts.append(f'<line x1="{x0:.2f}" y1="{y0:.2f}" x2="{x1:.2f}" y2="{y1:.2f}" '
                         f'stroke="#222" stroke-width="0.8" stroke-linecap="round"/>')
        last = (x1, y1)
    parts.append("</svg>")
    path = Path(path)
    path.write_text("\n".join(parts))
    return path
