"""
analyze_run.py - Calcule les metriques d'un ou plusieurs essais.

Usage (depuis le dossier du projet) :
    # un seul essai
    python tools/analyze_run.py controllers/av_lidar_stop/run_log_180rays.csv

    # comparaison de plusieurs essais (un tableau cote a cote)
    python tools/analyze_run.py controllers/av_lidar_stop/run_log_180rays.csv controllers/av_lidar_stop/run_log_720rays.csv

    # sans argument : analyse tous les run_log*.csv du controleur
    python tools/analyze_run.py

Metriques produites :
  - success             : la voiture s'est arretee sans toucher l'obstacle
  - first_detect_m      : distance de la premiere detection de l'obstacle
  - first_detect_s      : instant de la premiere detection
  - stop_trigger_s      : instant du passage en STOP
  - stop_time_s         : instant de l'arret complet
  - stop_distance_m     : distance restante a l'obstacle a l'arret
  - min_distance_m      : distance minimale mesuree pendant l'essai
  - max_speed_kmh       : vitesse maximale atteinte
  - distance_driven_m   : distance parcourue (a partir du GPS)
  - lateral_dev_max_m   : ecart lateral maximal par rapport a la voie (y = LANE_Y)
  - first_hit_elev_deg  : (diagnostic) elevation du point touche dans le repere du LiDAR
  - pitch_at_detect_deg : (diagnostic) tangage de la voiture a la premiere detection
  - pitch_cruise_deg    : (diagnostic) tangage moyen en croisiere
  - false_hits          : mesures plus courtes que la vraie distance (faux obstacle, ex. route)
"""

import csv
import glob
import math
import os
import sys

LANE_Y = -2.0            # centre de la voie de droite dans la scene S1
STOP_SPEED_KMH = 0.5     # en dessous : on considere la voiture arretee
COLLISION_DIST_M = 1.0   # en dessous : on considere qu'il y a contact
OBSTACLE_X = 45.0        # position x du baril dans la scene S1
FALSE_HIT_MARGIN_M = 0.5 # mesure plus courte que la vraie distance de plus de 0,5 m = faux obstacle
DEFAULT_GLOB = os.path.join("controllers", "av_lidar_stop", "run_log*.csv")


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "t": float(r["time_s"]),
                "x": float(r["x"]),
                "y": float(r["y"]),
                "v": float(r["speed_kmh"]),
                "d": math.inf if r["front_dist_m"] == "inf" else float(r["front_dist_m"]),
                "state": r["state"],
                "elev": float(r["hit_elev_deg"]) if r.get("hit_elev_deg") else None,
                "pitch": float(r["pitch_deg"]) if r.get("pitch_deg") else None,
            })
    return rows


def _cruise_pitch(rows):
    """Tangage moyen de la voiture pendant la croisiere (avant detection)."""
    vals = [r["pitch"] for r in rows
            if r.get("pitch") is not None and r["state"] == "CRUISE" and r["v"] > 25]
    return round(sum(vals) / len(vals), 3) if vals else None


def _false_hits(rows):
    """Compte les mesures nettement plus courtes que la vraie distance au baril
    (ex. : le faisceau touche la route quand la voiture plonge au freinage).
    Le decalage GPS -> face du baril est estime sur les 50 premieres detections."""
    det = [r for r in rows if math.isfinite(r["d"])]
    if len(det) < 10:
        return None
    offsets = sorted(OBSTACLE_X - r["x"] - r["d"] for r in det[:50])
    k = offsets[len(offsets) // 2]
    return sum(1 for r in det if (OBSTACLE_X - r["x"] - k) - r["d"] > FALSE_HIT_MARGIN_M)


def analyze(rows):
    finite_d = [r["d"] for r in rows if math.isfinite(r["d"])]
    min_d = min(finite_d) if finite_d else math.inf

    first_det = next((r for r in rows if math.isfinite(r["d"])), None)
    stop_trig = next((r for r in rows if r["state"] == "STOP"), None)
    stop_row = next((r for r in rows if r["state"] == "STOP" and r["v"] < STOP_SPEED_KMH), None)

    driven = 0.0
    for a, b in zip(rows, rows[1:]):
        driven += math.hypot(b["x"] - a["x"], b["y"] - a["y"])

    def rnd(row, key, n=2):
        return round(row[key], n) if row else None

    return {
        "success": stop_row is not None and min_d > COLLISION_DIST_M,
        "first_detect_m": rnd(first_det, "d"),
        "first_detect_s": rnd(first_det, "t"),
        "stop_trigger_s": rnd(stop_trig, "t"),
        "stop_time_s": rnd(stop_row, "t"),
        "stop_distance_m": rnd(stop_row, "d"),
        "min_distance_m": round(min_d, 2) if math.isfinite(min_d) else None,
        "max_speed_kmh": round(max(r["v"] for r in rows), 2),
        "distance_driven_m": round(driven, 2),
        "lateral_dev_max_m": round(max(abs(r["y"] - LANE_Y) for r in rows), 3),
        # diagnostic (vide si le log ne contient pas ces colonnes)
        "first_hit_elev_deg": round(first_det["elev"], 3) if first_det and first_det.get("elev") is not None else None,
        "pitch_at_detect_deg": round(first_det["pitch"], 3) if first_det and first_det.get("pitch") is not None else None,
        "pitch_cruise_deg": _cruise_pitch(rows),
        "false_hits": _false_hits(rows),
    }


def main(paths):
    if not paths:
        paths = sorted(glob.glob(DEFAULT_GLOB))
    if not paths:
        sys.exit(f"Aucun fichier de log trouve ({DEFAULT_GLOB}).")

    results = {}
    for p in paths:
        rows = load(p)
        if not rows:
            print(f"[ATTENTION] {p} est vide, ignore.")
            continue
        name = os.path.splitext(os.path.basename(p))[0].replace("run_log_", "")
        results[name] = analyze(rows)

    names = list(results)
    keys = list(next(iter(results.values())))
    col = max(12, *(len(n) for n in names)) + 2
    print("metrique".ljust(20) + "".join(n.rjust(col) for n in names))
    print("-" * (20 + col * len(names)))
    for k in keys:
        print(k.ljust(20) + "".join(str(results[n][k]).rjust(col) for n in names))


if __name__ == "__main__":
    main(sys.argv[1:])
