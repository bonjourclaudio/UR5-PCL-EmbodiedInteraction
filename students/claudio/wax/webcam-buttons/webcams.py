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
# QUI SOTTO METTI I LINK DELLE 4 WEBCAM
# Metti il link della PAGINA con l'overview, quello che apri nel browser,
# per esempio: "http://200.46.196.243/view/viewer_index.shtml?id=1572"
# Lo script apre la pagina, cerca <img id="image0"> e salva quell'immagine.
# Se serve login scrivi user e password tra virgolette, altrimenti None.
# =====================================================================
CAMERAS = {
    "1": {
        "name": "cam1",
        "url": "http://www.insecam.org/en/view/167618/",   # QUI IP 1
        "user": None, "password": None,
    },
    "2": {
        "name": "cam2",
        "url": "http://www.insecam.org/en/view/212604/",   # QUI IP 2
        "user": None, "password": None,
    },
    "3": {
        "name": "cam3",
        "url": "http://www.insecam.org/en/view/759172/",   # QUI IP 3
        "user": None, "password": None,
    },
    "4": {
        "name": "cam4",
        "url": "http://www.insecam.org/en/view/1012200/",   # QUI IP 4
        "user": None, "password": None,
    },
}

IMG_ID = "image0"      # id dell'immagine da cercare nella pagina

SERIAL_PORT = None     # None = trova l'Arduino da solo; oppure "/dev/cu.usbmodem1401"
BAUD = 115200
DEBUG_SERIALE = True   # True = stampa ogni riga che arriva dall'Arduino (per i test)

# QUI WAX: l'indirizzo che apri nel browser per la pagina "Wax canvas",
# senza / finale. Esempio: "http://localhost:8080"
# Con "" le foto vengono solo salvate in frames/ e non inviate.
WAX_URL = ""
# =====================================================================

BASE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE, "frames")
TIMEOUT_S = 10
COOLDOWN_S = 0.5

last_press = {}
lock = threading.Lock()

# Mappatura bottone fisico (pin Arduino) -> webcam. Si crea con "m" + Invio.
MAPPA_FILE = os.path.join(BASE, "mappa_bottoni.json")
MAPPA = {}
mappa_queue = queue.Queue()
mappando = threading.Event()


class TrovaImg(HTMLParser):
    """Raccoglie tutti i tag <img> della pagina."""
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


def primo_jpeg_da_stream(r):
    """Se l'immagine è uno stream MJPEG, estrae il primo frame."""
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


def ottieni_immagine(cam):
    """Apre la pagina, trova <img id="image0"> e scarica quell'immagine.
    Ritorna (bytes, url_immagine)."""
    page = http_get(cam["url"], cam)
    if page.status_code != 200:
        raise RuntimeError(f"pagina: HTTP {page.status_code}")

    parser = TrovaImg()
    parser.feed(page.text)
    img = next((i for i in parser.imgs if i.get("id") == IMG_ID), None)
    if img is None:
        ids = [i.get("id") for i in parser.imgs]
        raise RuntimeError(
            f'nella pagina non trovo <img id="{IMG_ID}">. '
            f"Tag img trovati (id): {ids}. Se la lista è vuota, l'immagine "
            f"viene creata da JavaScript: dimmelo e uso un altro metodo.")

    src = img.get("src") or img.get("data-src")
    if not src:
        raise RuntimeError(f'<img id="{IMG_ID}"> non ha un src nella pagina '
                           f"(viene impostato da JavaScript?). Tag: {img}")

    # immagine incorporata nella pagina (data:image/jpeg;base64,...)
    if src.startswith("data:"):
        return base64.b64decode(src.split(",", 1)[1]), "data-uri"

    img_url = urljoin(page.url, src)
    r = http_get(img_url, cam, stream=True, referer=page.url)
    if r.status_code != 200:
        raise RuntimeError(f"immagine {img_url}: HTTP {r.status_code}")
    ctype = r.headers.get("Content-Type", "")
    data = primo_jpeg_da_stream(r) if "multipart" in ctype else r.content
    if not data or not data.startswith((b"\xff\xd8", b"\x89PNG")):
        raise RuntimeError(f"{img_url} non è un'immagine valida ({ctype})")
    return data, img_url


def salva_frame(cam):
    nome = cam["name"]
    t0 = time.time()
    try:
        data, img_url = ottieni_immagine(cam)
        cartella = os.path.join(OUTPUT_DIR, nome)
        os.makedirs(cartella, exist_ok=True)
        fname = datetime.now().strftime("frame_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
        path = os.path.join(cartella, fname)
        with open(path, "wb") as f:
            f.write(data)
        print(f"[{nome}] salvato in {time.time() - t0:.1f}s: {path}")
        print(f"        (immagine presa da: {img_url})")
        invia_a_wax(data, nome)
    except Exception as e:
        print(f"[{nome}] ERRORE: {e}")


def invia_a_wax(data, nome):
    """Manda la foto alla pagina Wax canvas (stessa chiamata del tasto upload)."""
    if not WAX_URL:
        return
    if Image is None:
        print("        [Wax] manca Pillow: riapri 1_INSTALLA_UNA_VOLTA.command")
        return
    try:
        im = Image.open(io.BytesIO(data)).convert("RGB")
        buf = io.BytesIO()
        im.save(buf, "PNG")
        fname = f"{nome}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        r = requests.post(WAX_URL.rstrip("/") + "/api/upload", data=buf.getvalue(),
                          headers={"X-Filename": fname}, timeout=20)
        j = r.json()
        if j.get("error"):
            print(f"        [Wax] ERRORE: {j['error']}")
        else:
            print(f"        [Wax] inviata come {j.get('name', fname)}")
    except Exception as e:
        print(f"        [Wax] ERRORE: {type(e).__name__}: {e}")


def scatta(codice):
    """Questa funzione la chiamerà Arduino in futuro."""
    cam = CAMERAS.get(codice)
    if not cam:
        print(f"Webcam {codice}: non configurata")
        return None
    now = time.time()
    with lock:
        if now - last_press.get(codice, 0) < COOLDOWN_S:
            return None
        last_press[codice] = now
    t = threading.Thread(target=salva_frame, args=(cam,), daemon=True)
    t.start()
    return t


def carica_mappa():
    try:
        with open(MAPPA_FILE) as f:
            MAPPA.update({str(k): str(v) for k, v in json.load(f).items()})
    except (OSError, ValueError):
        pass


def gestisci_bottone(n):
    """Chiamata a ogni 'Bn' ricevuto da Arduino."""
    if mappando.is_set():
        mappa_queue.put(n)
        return
    cam = MAPPA.get(n, n)
    extra = f" -> webcam {cam}" if cam != n else ""
    print(f"[Arduino] bottone {n} premuto{extra}")
    scatta(cam)


def mappa_bottoni():
    """Procedura guidata: premi i bottoni da sinistra a destra."""
    nomi = ["il PRIMO da SINISTRA", "il SECONDO da sinistra",
            "il TERZO da sinistra", "l'ULTIMO, il piu' a DESTRA"]
    nuova = {}
    mappando.set()
    try:
        for pos, nome in enumerate(nomi, 1):
            while True:
                while not mappa_queue.empty():
                    mappa_queue.get_nowait()
                print(f"Premi {nome}  (sara' la webcam {pos})...")
                try:
                    n = mappa_queue.get(timeout=60)
                except queue.Empty:
                    print("Nessun segnale dall'Arduino per 60 secondi: "
                          "mappatura annullata (la vecchia resta valida).")
                    return
                if n in nuova:
                    print(f"  Quel bottone e' gia' la webcam {nuova[n]}: "
                          "premi uno diverso.")
                    time.sleep(0.5)
                    continue
                nuova[n] = str(pos)
                print(f"  ok: bottone fisico {n} = webcam {pos}")
                time.sleep(0.7)            # il tempo di staccare il dito
                break
    finally:
        mappando.clear()
    MAPPA.clear()
    MAPPA.update(nuova)
    with open(MAPPA_FILE, "w") as f:
        json.dump(MAPPA, f)
    print("Mappatura salvata. Da sinistra a destra: webcam 1, 2, 3, 4.\n")


PORT_NAMES = ("usbmodem", "usbserial", "ttyACM", "ttyUSB")   # macOS + Linux


def trova_porta():
    for p in list_ports.comports():
        if any(n in p.device for n in PORT_NAMES):
            return p.device
    return None


def ascolta_seriale():
    """Legge i messaggi B1..B4 dall'Arduino e chiama scatta()."""
    avvisato = False
    while True:
        porta = SERIAL_PORT or trova_porta()
        if not porta:
            if not avvisato:
                print("[Arduino] non trovato: collegalo con il cavo USB e chiudi "
                      "il Serial Monitor. Riprovo ogni 3 secondi "
                      "(intanto la tastiera funziona).")
                avvisato = True
            time.sleep(3)
            continue
        avvisato = False
        try:
            candidate = [p.device for p in list_ports.comports()
                         if any(n in p.device for n in PORT_NAMES)]
            if len(candidate) > 1 and not SERIAL_PORT:
                print(f"[Arduino] ATTENZIONE: trovate piu' porte {candidate}. "
                      f"Uso {porta}. Se non e' quella della Mega, scrivi la porta "
                      f"giusta in SERIAL_PORT oppure scollega l'altra scheda.")
            with serial.Serial(porta, BAUD, timeout=1) as ser:
                print(f"[Arduino] collegato su {porta} a {BAUD} baud")
                time.sleep(2)              # l'Arduino può resettarsi all'apertura
                ser.reset_input_buffer()
                while True:
                    raw = ser.readline()
                    riga = raw.decode("utf-8", errors="ignore").strip()
                    if DEBUG_SERIALE and raw.strip():
                        print(f"[Arduino] byte ricevuti: {raw!r}")
                    if not riga:
                        continue
                    m = re.search(r"B(\d+)", riga)
                    if m:
                        gestisci_bottone(m.group(1))
        except (serial.SerialException, OSError) as e:
            print(f"[Arduino] scollegato ({e}). Riprovo...")
            time.sleep(3)


if __name__ == "__main__":
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    carica_mappa()
    if MAPPA:
        print("Mappatura bottoni caricata:", dict(sorted(MAPPA.items())),
              "(bottone fisico -> webcam)\n")
    if not WAX_URL:
        print("Wax canvas: WAX_URL è vuoto, le foto restano solo in frames/.\n")
    if serial is None:
        print("pyserial non installato: riapri 1_INSTALLA_UNA_VOLTA.command "
              "per attivare Arduino. Per ora funziona solo la tastiera.\n")
    else:
        threading.Thread(target=ascolta_seriale, daemon=True).start()
    print("Scrivi 1, 2, 3 o 4 + Invio per salvare da quella webcam.")
    print("a + Invio = tutte e 4 insieme.  m + Invio = rimappa i bottoni.  q + Invio = esci.")
    print(f"Le immagini finiscono in: {OUTPUT_DIR}\n")
    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == "q":
            break
        if cmd == "m":
            mappa_bottoni()
        elif cmd == "a":
            threads = [scatta(c) for c in CAMERAS]
            for t in threads:
                if t:
                    t.join()
        elif cmd:
            scatta(cmd)
