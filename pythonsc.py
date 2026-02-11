# calibrate_and_map.py
# Usage:
# 1) Run this with ESP connected (ESP must be running the ORIENT stream sketch).
# 2) The script auto-detects COM port. For each drum name it prompts:
#    "Point for <DRUM> and press Enter" -> it collects samples for 2 seconds.
# 3) It saves mapping.json and then enters live mode, sending keys when nearest drum changes.

import serial.tools.list_ports
import serial
import time
import json
import math
from pynput.keyboard import Controller

BAUD = 115200
SAMPLE_COUNT = 200   # ~2 seconds at ~100Hz
COLLECT_DELAY = 0.01 # sec between reads (match ESP stream rate)

# Default drums and keys (based on your Musicca screenshot). Edit if you want.
DRUMS = [
    ("crash", "y"),
    ("ride", "u"),
    ("hh_closed", "r"),
    ("hh_open", "e"),
    ("hh_foot", "c"),
    ("high_tom", "g"),
    ("low_tom", "h"),
    ("floor_tom", "j"),
    ("snare", "s"),
    ("snare_cross", "d"),
    ("bass", "x")
]

kb = Controller()

def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Connect ESP and try again.")
    for p in ports:
        desc = (p.description or "").lower()
        if "usb" in desc or "silicon" in desc or "cp210" in desc or "wch" in desc or "ftdi" in desc:
            print("Auto-selected serial:", p.device, p.description)
            return p.device
    print("Auto-selected (fallback):", ports[0].device)
    return ports[0].device

def open_serial(port):
    s = serial.Serial(port, BAUD, timeout=1)
    time.sleep(1.0)
    return s

def parse_orient_line(line):
    # expects: ORIENT <yaw> <pitch> <roll>
    parts = line.strip().split()
    if len(parts) >= 4 and parts[0].upper() == "ORIENT":
        try:
            yaw = float(parts[1])
            pitch = float(parts[2])
            roll = float(parts[3])
            return (yaw, pitch, roll)
        except:
            return None
    return None

def collect_samples(ser, count):
    samples = []
    start = time.time()
    while len(samples) < count and (time.time() - start) < (count * COLLECT_DELAY * 2):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line: continue
        o = parse_orient_line(line)
        if o:
            samples.append(o)
        time.sleep(COLLECT_DELAY)
    return samples

def mean_centroid(samples):
    # yaw is circular; compute mean via vector sum
    sx = sy = spx = spy = srx = sry = 0.0
    n = len(samples)
    if n == 0:
        return None
    for (yaw, pitch, roll) in samples:
        rad = math.radians(yaw)
        sx += math.cos(rad)
        sy += math.sin(rad)
        spx += pitch
        spy += pitch*0  # not used, but kept for symmetry
        srx += roll
        sry += roll*0
    mean_yaw = math.degrees(math.atan2(sy, sx))
    mean_pitch = spx / n
    mean_roll = srx / n
    return (mean_yaw, mean_pitch, mean_roll)

def dist(a, b):
    # distance metric between two orient tuples (yaw,pitch,roll)
    # yaw is circular: compute angular difference properly
    yaw_a, p_a, r_a = a
    yaw_b, p_b, r_b = b
    dy = min(abs(yaw_a - yaw_b), 360 - abs(yaw_a - yaw_b))
    dp = p_a - p_b
    dr = r_a - r_b
    # weight yaw more strongly (tweak multipliers if needed)
    return math.sqrt((dy*1.0)**2 + (dp*6.0)**2 + (dr*4.0)**2)

def calibrate(ser):
    mapping = {}
    print("Calibration starting. For each drum: point at it, hold steady, then press Enter.")
    for name, key in DRUMS:
        input(f"\nPoint to '{name}' and press Enter to collect samples...")
        print("Collecting... hold still.")
        samples = collect_samples(ser, SAMPLE_COUNT)
        if not samples:
            print("No samples collected for", name)
            continue
        centroid = mean_centroid(samples)
        mapping[name] = {"key": key, "centroid": centroid, "n": len(samples)}
        print(f"Recorded {len(samples)} samples for {name}, centroid yaw={centroid[0]:.2f}, pitch={centroid[1]:.2f}, roll={centroid[2]:.2f}")
    return mapping

def save_mapping(mapping, fname="mapping.json"):
    # JSON can't store tuples directly; convert lists
    out = {}
    for k,v in mapping.items():
        out[k] = {"key": v["key"], "centroid": list(v["centroid"]), "n": v["n"]}
    with open(fname, "w") as f:
        json.dump(out, f, indent=2)
    print("Saved mapping to", fname)

def load_mapping(fname="mapping.json"):
    with open(fname, "r") as f:
        raw = json.load(f)
    mapping = {}
    for k,v in raw.items():
        mapping[k] = {"key": v["key"], "centroid": tuple(v["centroid"]), "n": v.get("n",0)}
    return mapping

def live_map(ser, mapping):
    names = list(mapping.keys())
    centroids = [mapping[n]["centroid"] for n in names]
    keys = [mapping[n]["key"] for n in names]
    print("\nEntering live mode. Press Ctrl+C to quit.")
    last_idx = -1
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line: continue
        o = parse_orient_line(line)
        if not o: continue
        # find nearest centroid
        dists = [dist(o, c) for c in centroids]
        idx = int(min(range(len(dists)), key=lambda i: dists[i]))
        if idx != last_idx:
            name = names[idx]
            key = keys[idx]
            print(f"Zone -> {name} (key '{key}')   dist={dists[idx]:.2f}")
            # send key
            try:
                kb.press(key)
                kb.release(key)
            except Exception as e:
                print("Keyboard send error:", e)
            last_idx = idx

def main():
    port = find_port()
    ser = open_serial(port)
    print("Connected. Waiting for ORIENT stream...")
    # flush initial lines
    time.sleep(0.5)
    # calibration
    mapping = calibrate(ser)
    save_mapping(mapping)
    print("\nCalibration complete. You can re-run or edit mapping.json.")
    # live mode
    try:
        live_map(ser, mapping)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
