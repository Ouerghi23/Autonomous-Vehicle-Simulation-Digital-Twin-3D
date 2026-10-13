"""
av_lidar_stop.py - Premier controleur Webots pour le projet
"Autonomous Vehicle Simulation - Digital Twin".

Ce que fait le script :
  1. Fait rouler la voiture a vitesse de croisiere.
  2. Lit le LiDAR et calcule la distance de l'obstacle le plus proche devant.
  3. Ralentit puis s'arrete si un obstacle est trop proche.
  4. Enregistre les donnees (temps, position, vitesse, distance) dans un CSV
     pour le futur dashboard (un fichier par resolution du LiDAR).

Prerequis dans la scene Webots :
  - Un vehicule (ex : TeslaModel3) dont le champ "controller" = "av_lidar_stop"
  - Dans sensorsSlotFront : un noeud Lidar nomme "lidar"
  - Dans sensorsSlotTop   : un noeud GPS nomme "gps"
"""

import csv
import math
import os
import sys

from vehicle import Driver

# ----------------------------------------------------------------------
# Parametres du scenario (garde-les identiques entre les essais)
# ----------------------------------------------------------------------
CRUISE_SPEED_KMH = 30.0     # vitesse de croisiere
SLOW_DOWN_DIST_M = 20.0     # en dessous : on ralentit
STOP_DIST_M = 8.0           # en dessous : on s'arrete
FRONT_SECTOR_DEG = 30.0     # largeur du cone surveille devant la voiture
MAX_SIM_TIME_S = 60.0       # duree maximale d'un essai
LOG_PREFIX = "run_log"       # le fichier final sera run_log_<N>rays.csv

# ----------------------------------------------------------------------
# Initialisation
# ----------------------------------------------------------------------
driver = Driver()
timestep = int(driver.getBasicTimeStep())

lidar = driver.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()  # pour connaitre la hauteur du point touche (diagnostic)

gps = driver.getDevice("gps")
gps.enable(timestep)

# Centrale inertielle optionnelle (presente seulement dans la scene de diagnostic)
imu = None
try:
    if any(driver.getDeviceByIndex(i).getName() == "imu" for i in range(driver.getNumberOfDevices())):
        imu = driver.getDevice("imu")
        imu.enable(timestep)
except Exception:
    imu = None

h_res = lidar.getHorizontalResolution()
fov = lidar.getFov()  # en radians

# Indices des rayons du LiDAR qui couvrent le cone avant
half_sector = math.radians(FRONT_SECTOR_DEG) / 2.0
center = h_res // 2
half_width = int((half_sector / fov) * h_res)
front_start = max(0, center - half_width)
front_end = min(h_res, center + half_width + 1)

driver.setSteeringAngle(0.0)  # route droite pour ce premier test

# Un fichier par configuration de LiDAR : les essais ne s'ecrasent pas entre eux
# Nom de l'essai : fourni par la scene (champ controllerArgs), sinon deduit du LiDAR
RUN_LABEL = sys.argv[1] if len(sys.argv) > 1 else f"{h_res}rays{'_diag' if imu else ''}"
LOG_FILE = f"{LOG_PREFIX}_{RUN_LABEL}.csv"
log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), LOG_FILE)
log_file = open(log_path, "w", newline="")
writer = csv.writer(log_file)
writer.writerow(["time_s", "x", "y", "speed_kmh", "front_dist_m", "state",
                 "hit_z_m", "hit_elev_deg", "gps_z_m", "pitch_deg"])

print(f"[INFO] LiDAR : {h_res} rayons, FOV = {math.degrees(fov):.0f} deg")
print(f"[INFO] Log ecrit dans : {log_path}")


def front_distance():
    """Distance (m) de l'obstacle le plus proche dans le cone avant,
    et indice du rayon correspondant (None si rien n'est detecte)."""
    ranges = lidar.getRangeImage()
    if not ranges:
        return float("inf"), None
    best_d, best_i = float("inf"), None
    for i in range(front_start, front_end):
        r = ranges[i]
        if math.isfinite(r) and 0 < r < best_d:
            best_d, best_i = r, i
    return best_d, best_i


def hit_height(index):
    """Hauteur (z) et angle d'elevation du point touche, dans le repere du LiDAR.
    z > 0 : le point est au-dessus du LiDAR ; z < 0 : en dessous."""
    if index is None:
        return None, None
    # LiDAR a une seule nappe : l'indice du rayon est aussi l'indice dans la nappe 0
    p = lidar.getLayerPointCloud(0)[index]
    horiz = math.hypot(p.x, p.y)
    if horiz <= 0:
        return None, None
    return p.z, math.degrees(math.atan2(p.z, horiz))


# ----------------------------------------------------------------------
# Boucle principale
# ----------------------------------------------------------------------
state = "CRUISE"
first_seen = False
try:
    while driver.step() != -1:
        t = driver.getTime()
        if t > MAX_SIM_TIME_S:
            print("[INFO] Duree maximale atteinte, fin de l'essai.")
            break

        dist, ray = front_distance()
        hz, elev = hit_height(ray)
        if hz is not None and not first_seen:
            first_seen = True
            print(f"[DIAG] Premiere detection : {dist:.2f} m, point a z = {hz:+.3f} m "
                  f"par rapport au LiDAR (elevation {elev:+.2f} deg)"
                  + ("" if imu is None else f", tangage voiture {math.degrees(imu.getRollPitchYaw()[1]):+.2f} deg"))

        if dist < STOP_DIST_M:
            state = "STOP"
            driver.setCruisingSpeed(0.0)
            driver.setBrakeIntensity(1.0)
        elif dist < SLOW_DOWN_DIST_M:
            state = "SLOW"
            # vitesse proportionnelle a la distance restante
            ratio = (dist - STOP_DIST_M) / (SLOW_DOWN_DIST_M - STOP_DIST_M)
            driver.setBrakeIntensity(0.0)
            driver.setCruisingSpeed(max(5.0, CRUISE_SPEED_KMH * ratio))
        else:
            state = "CRUISE"
            driver.setBrakeIntensity(0.0)
            driver.setCruisingSpeed(CRUISE_SPEED_KMH)

        x, y, gz = gps.getValues()
        pitch = math.degrees(imu.getRollPitchYaw()[1]) if imu else None
        speed = driver.getCurrentSpeed()
        speed = 0.0 if math.isnan(speed) else speed

        writer.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}",
                         f"{speed:.2f}",
                         "inf" if math.isinf(dist) else f"{dist:.2f}",
                         state,
                         "" if hz is None else f"{hz:.3f}",
                         "" if elev is None else f"{elev:.3f}",
                         f"{gz:.3f}",
                         "" if pitch is None else f"{pitch:.3f}"])

        # Affichage console toutes les ~0.5 s
        if int(t * 1000) % 500 < timestep:
            d_txt = "aucun" if math.isinf(dist) else f"{dist:5.1f} m"
            print(f"t={t:5.1f}s  v={speed:5.1f} km/h  obstacle={d_txt}  etat={state}")
finally:
    log_file.close()
