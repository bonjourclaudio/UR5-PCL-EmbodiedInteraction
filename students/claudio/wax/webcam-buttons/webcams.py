import base64
import io
import json
import os
import queue
import re
import threading
import time
from datetime import datetime
from html.parser import HTMLParser
from urllib.parse import urljoin

import requests
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None

try:
    from PIL import Image
except ImportError:
    Image = None

# =====================================================================
# THE LINKS OF THE 4 WEBCAMS
# Use the link of the overview PAGE, the one you open in the browser,
# for example: "http://200.46.196.243/view/viewer_index.shtml?id=1572"
# The script opens the page, looks for <img id="image0"> and saves that image.
# If a login is needed, put user and password in quotes, otherwise None.
# =====================================================================
CAMERAS = {
    "1": {
        "name": "cam1",
        "url": "http://www.insecam.org/en/view/167618/",   # webcam 1
        "user": None, "password": None,
    },
    "2": {
        "name": "cam2",
        "url": "http://www.insecam.org/en/view/212604/",   # webcam 2
        "user": None, "password": None,
    },
    "3": {
        "name": "cam3",
        "url": "http://www.insecam.org/en/view/759172/",   # webcam 3
        "user": None, "password": None,
    },
    "4": {
        "name": "cam4",
        "url": "http://www.insecam.org/en/view/1012200/",   # webcam 4
        "user": None, "password": None,
    },
}

IMG_ID = "image0"      # id of the image to look for in the page

SERIAL_PORT = None     # None = find the Arduino automatically; or e.g. "/dev/ttyACM0", "/dev/cu.usbmodem1401"
BAUD = 115200
DEBUG_SERIAL = True    # True = print every line that arrives from the Arduino (for testing)

# Engraving station on ANOTHER computer: its address, without a trailing /,
# e.g. "http://192.168.1.150:8770". Frames and status are then sent there.
# Leave "" when the station runs on this computer (it reads frames/ directly).
WAX_URL = ""
# =====================================================================

BASE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE, "frames")
TIMEOUT_S = 10
COOLDOWN_S = 0.5

last_press = {}
lock = threading.Lock()

# Physical button (Arduino pin) -> webcam. Created with "m" + Enter.
BUTTON_MAP_FILE = os.path.join(BASE, "button_map.json")
BUTTON_MAP = {}
mapping_queue = queue.Queue()
mapping = threading.Event()

# Status for the engraving station page (engrave_server.py): Arduino, last button press,
# last download per webcam. Written every 2 s to capture_status.json (and sent to WAX_URL).
STATUS_FILE = os.path.join(BASE, "capture_status.json")
STATUS = {"arduino": {"connected": False, "port": None, "error": "starting"},
          "last_press": None, "cams": {}}


def now_iso():
    return datetime.now().isoformat(timespec="seconds")


def write_status():
    while True:
        STATUS["heartbeat"] = now_iso()
        try:
            tmp = STATUS_FILE + ".tmp"
            with open(tmp, "w") as f:
                json.dump(STATUS, f)
            os.replace(tmp, STATUS_FILE)
        except OSError:
            pass
        if WAX_URL:
            try:
                requests.post(WAX_URL.rstrip("/") + "/api/capture_status", json=STATUS, timeout=2)
            except requests.RequestException:
                pass
        time.sleep(2)


class FindImg(HTMLParser):
    """Collects all <img> tags of the page."""
    def __init__(self):
        super().__init__()
        self.imgs = []

    def handle_starttag(self, tag, attrs):
        if tag == "img":
            self.imgs.append(dict(attrs))


def http_get(url, cam, stream=False, referer=None):
    headers = {"User-Agent": "Mozilla/5.0"}
    if referer:
        headers["Referer"] = referer
    if not cam.get("user"):
        return requests.get(url, timeout=TIMEOUT_S, headers=headers, stream=stream)
    r = None
    for auth in (HTTPDigestAuth(cam["user"], cam["password"]),
                 HTTPBasicAuth(cam["user"], cam["password"])):
        r = requests.get(url, auth=auth, timeout=TIMEOUT_S,
                         headers=headers, stream=stream)
        if r.status_code != 401:
            return r
    return r


def first_jpeg_from_stream(r):
    """If the image is an MJPEG stream, extract the first frame."""
    buf = b""
    for chunk in r.iter_content(4096):
        buf += chunk
        start = buf.find(b"\xff\xd8")
        if start != -1:
            end = buf.find(b"\xff\xd9", start)
            if end != -1:
                return buf[start:end + 2]
        if len(buf) > 5_000_000:
            break
    return None


def fetch_image(cam):
    """Open the page, find <img id="image0"> and download that image.
    Returns (bytes, image_url)."""
    page = http_get(cam["url"], cam)
    if page.status_code != 200:
        raise RuntimeError(f"page: HTTP {page.status_code}")

    parser = FindImg()
    parser.feed(page.text)
    img = next((i for i in parser.imgs if i.get("id") == IMG_ID), None)
    if img is None:
        ids = [i.get("id") for i in parser.imgs]
        raise RuntimeError(
            f'no <img id="{IMG_ID}"> in the page. '
            f"img tags found (ids): {ids}. If the list is empty, the image "
            f"is created by JavaScript and needs another method.")

    src = img.get("src") or img.get("data-src")
    if not src:
        raise RuntimeError(f'<img id="{IMG_ID}"> has no src in the page '
                           f"(set by JavaScript?). Tag: {img}")

    # image embedded in the page (data:image/jpeg;base64,...)
    if src.startswith("data:"):
        return base64.b64decode(src.split(",", 1)[1]), "data-uri"

    img_url = urljoin(page.url, src)
    r = http_get(img_url, cam, stream=True, referer=page.url)
    if r.status_code != 200:
        raise RuntimeError(f"image {img_url}: HTTP {r.status_code}")
    ctype = r.headers.get("Content-Type", "")
    data = first_jpeg_from_stream(r) if "multipart" in ctype else r.content
    if not data or not data.startswith((b"\xff\xd8", b"\x89PNG")):
        raise RuntimeError(f"{img_url} is not a valid image ({ctype})")
    return data, img_url


def save_frame(cam):
    name = cam["name"]
    t0 = time.time()
    try:
        data, img_url = fetch_image(cam)
        if WAX_URL:
            send_to_station(data, name)          # the station stores it in its frames/
        else:
            folder = os.path.join(OUTPUT_DIR, name)
            os.makedirs(folder, exist_ok=True)
            fname = datetime.now().strftime("frame_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
            path = os.path.join(folder, fname)
            with open(path, "wb") as f:
                f.write(data)
            print(f"[{name}] saved in {time.time() - t0:.1f}s: {path}")
        print(f"        (image from: {img_url})")
        STATUS["cams"][name] = {"ok_at": now_iso(), "error": None, "error_at": None}
    except Exception as e:
        print(f"[{name}] ERROR: {e}")
        st = STATUS["cams"].setdefault(name, {"ok_at": None})
        st.update(error=str(e)[:200], error_at=now_iso())


def send_to_station(data, name):
    """Send the frame to the engraving station on another computer (its /api/upload)."""
    if Image is None:
        raise RuntimeError("Pillow missing: run 1_INSTALL_ONCE.command again")
    im = Image.open(io.BytesIO(data)).convert("RGB")
    buf = io.BytesIO()
    im.save(buf, "PNG")
    fname = f"{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    r = requests.post(WAX_URL.rstrip("/") + "/api/upload", data=buf.getvalue(),
                      headers={"X-Filename": fname}, timeout=20)
    j = r.json()
    if j.get("error"):
        raise RuntimeError(f"station: {j['error']}")
    print(f"[{name}] sent to the station as {j.get('name', fname)}")


def capture(code):
    """Fetch a frame from webcam `code` (called for every button press)."""
    cam = CAMERAS.get(code)
    if not cam:
        print(f"Webcam {code}: not configured")
        return None
    now = time.time()
    with lock:
        if now - last_press.get(code, 0) < COOLDOWN_S:
            return None
        last_press[code] = now
    t = threading.Thread(target=save_frame, args=(cam,), daemon=True)
    t.start()
    return t


def load_button_map():
    try:
        with open(BUTTON_MAP_FILE) as f:
            BUTTON_MAP.update({str(k): str(v) for k, v in json.load(f).items()})
    except (OSError, ValueError):
        pass


def handle_button(n):
    """Called for every 'Bn' received from the Arduino."""
    if mapping.is_set():
        mapping_queue.put(n)
        return
    cam = BUTTON_MAP.get(n, n)
    extra = f" -> webcam {cam}" if cam != n else ""
    print(f"[Arduino] button {n} pressed{extra}")
    STATUS["last_press"] = {"button": n, "cam": cam, "at": now_iso()}
    capture(cam)


def map_buttons():
    """Guided setup: press the buttons from left to right."""
    names = ["the FIRST from the LEFT", "the SECOND from the left",
             "the THIRD from the left", "the LAST one, on the RIGHT"]
    new = {}
    mapping.set()
    try:
        for pos, label in enumerate(names, 1):
            while True:
                while not mapping_queue.empty():
                    mapping_queue.get_nowait()
                print(f"Press {label}  (it will be webcam {pos})...")
                try:
                    n = mapping_queue.get(timeout=60)
                except queue.Empty:
                    print("No signal from the Arduino for 60 seconds: "
                          "mapping cancelled (the old one stays valid).")
                    return
                if n in new:
                    print(f"  That button is already webcam {new[n]}: press a different one.")
                    time.sleep(0.5)
                    continue
                new[n] = str(pos)
                print(f"  ok: physical button {n} = webcam {pos}")
                time.sleep(0.7)            # time to lift the finger
                break
    finally:
        mapping.clear()
    BUTTON_MAP.clear()
    BUTTON_MAP.update(new)
    with open(BUTTON_MAP_FILE, "w") as f:
        json.dump(BUTTON_MAP, f)
    print("Mapping saved. From left to right: webcam 1, 2, 3, 4.\n")


PORT_NAMES = ("usbmodem", "usbserial", "ttyACM", "ttyUSB")   # macOS + Linux


def find_port():
    for p in list_ports.comports():
        if any(n in p.device for n in PORT_NAMES):
            return p.device
    return None


def listen_serial():
    """Read the B1..B4 messages from the Arduino and call handle_button()."""
    warned = False
    while True:
        port = SERIAL_PORT or find_port()
        if not port:
            STATUS["arduino"] = {"connected": False, "port": None, "error": "Arduino not found (USB?)"}
            if not warned:
                print("[Arduino] not found: connect it with the USB cable and close "
                      "the Serial Monitor. Retrying every 3 seconds "
                      "(the keyboard works meanwhile).")
                warned = True
            time.sleep(3)
            continue
        warned = False
        try:
            candidates = [p.device for p in list_ports.comports()
                          if any(n in p.device for n in PORT_NAMES)]
            if len(candidates) > 1 and not SERIAL_PORT:
                print(f"[Arduino] WARNING: several ports found {candidates}. "
                      f"Using {port}. If that is not the Mega, put the right port "
                      f"in SERIAL_PORT or unplug the other board.")
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"[Arduino] connected on {port} at {BAUD} baud")
                STATUS["arduino"] = {"connected": True, "port": port, "error": None, "since": now_iso()}
                time.sleep(2)              # the Arduino may reset when the port opens
                ser.reset_input_buffer()
                while True:
                    raw = ser.readline()
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if DEBUG_SERIAL and raw.strip():
                        print(f"[Arduino] bytes received: {raw!r}")
                    if not line:
                        continue
                    m = re.search(r"B(\d+)", line)
                    if m:
                        handle_button(m.group(1))
        except (serial.SerialException, OSError) as e:
            print(f"[Arduino] disconnected ({e}). Retrying...")
            STATUS["arduino"] = {"connected": False, "port": port, "error": str(e)[:200]}
            time.sleep(3)


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    load_button_map()
    if BUTTON_MAP:
        print("Button map loaded:", dict(sorted(BUTTON_MAP.items())),
              "(physical button -> webcam)\n")
    print(f"Frames are sent to the station at {WAX_URL}\n" if WAX_URL else
          f"Frames are saved in: {OUTPUT_DIR}\n")
    threading.Thread(target=write_status, daemon=True).start()
    if serial is None:
        STATUS["arduino"] = {"connected": False, "port": None, "error": "pyserial not installed"}
        print("pyserial not installed: run 1_INSTALL_ONCE.command again "
              "to enable the Arduino. For now only the keyboard works.\n")
    else:
        threading.Thread(target=listen_serial, daemon=True).start()
    print("Type 1, 2, 3 or 4 + Enter to save from that webcam.")
    print("a + Enter = all 4 at once.  m + Enter = map the buttons.  q + Enter = quit.\n")
    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == "q":
            break
        if cmd == "m":
            map_buttons()
        elif cmd == "a":
            threads = [capture(c) for c in CAMERAS]
            for t in threads:
                if t:
                    t.join()
        elif cmd:
            capture(cmd)
