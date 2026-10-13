# Autonomous Vehicle Simulation — Digital Twin 3D

A 3D simulation of an autonomous vehicle in **Webots**. The car drives, detects obstacles with a **LiDAR**, slows down and stops before a collision. Every run is logged and analyzed with reproducible metrics.

> ▶ **Online demo:** _webots.cloud link coming soon_
> 🎥 **Video:** _link coming soon_

## Scenario S1 — Static obstacle

| Parameter | Value |
|---|---|
| Road | Straight, 2 lanes, 140 m |
| Vehicle | Tesla Model 3, right lane |
| Obstacle | Barrel 45 m ahead |
| Cruise speed | 30 km/h |
| Slow down | below 20 m |
| Stop | below 8 m |
| Random seed | 42 |

## How it works

```
LiDAR  ──►  Perception: closest obstacle in a 30° cone ahead
GPS    ──►  Position (x, y)
                     │
                     ▼
         Decision: CRUISE → SLOW → STOP
                     │
                     ▼
         Control: speed + braking (Driver API)
                     │
                     ▼
         CSV log ──► analyze_run.py ──► metrics
```

## Project structure

```
av-digital-twin/
├── Dockerfile                 # Webots version used on webots.cloud
├── webots.yaml                # webots.cloud publishing settings
├── worlds/
│   ├── av_lidar_stop.wbt      # test 1: LiDAR with 180 rays
│   ├── av_lidar_stop_720.wbt  # test 2: 720 rays
│   ├── av_lidar_stop_1440.wbt # test 3: 1440 rays
│   ├── av_lidar_stop_diag.wbt # test 4: 720 rays + inertial unit
│   └── av_lidar_stop_tilt.wbt # test 5: LiDAR tilted to offset the car's pitch
├── controllers/
│   └── av_lidar_stop/
│       └── av_lidar_stop.py   # perception, decision, control, logging
└── tools/
    └── analyze_run.py         # computes the metrics
```

## Run it

1. Install **Webots R2025a** and **Python 3.11+**.
2. Open `worlds/av_lidar_stop.wbt` in Webots.
3. Press ▶. The log is saved in `controllers/av_lidar_stop/`, one file per test.
4. Compare all recorded runs:

```bash
python tools/analyze_run.py
```

## Metrics

| Metric | Meaning |
|---|---|
| `success` | Full stop without touching the obstacle |
| `first_detect_m` | Distance at which the obstacle is first detected |
| `stop_time_s` | Time until the car is fully stopped |
| `stop_distance_m` | Distance left to the obstacle when stopped |
| `min_distance_m` | Smallest distance measured |
| `max_speed_kmh` | Top speed |
| `distance_driven_m` | Distance driven |
| `lateral_dev_max_m` | Largest deviation from the lane center |

## Results: LiDAR study

| Test | LiDAR setup | First detection | Stopping distance | Success |
|---|---|---|---|---|
| 1 | 180 rays | 17.3 m | 7.86 m | ✅ |
| 2 | 720 rays | 33.5 m | 7.86 m | ✅ |
| 3 | 1440 rays | same as test 2 | 7.9 m | ✅ |
| 4 | 720 rays + inertial unit | 33.5 m (pitch −0.56°) | 7.9 m | ✅ |
| 5 | 720 rays, LiDAR tilted −0.56° | 36.8 m (pitch −1.04° while accelerating) | 7.86 m | ✅ |

**What the tests show**

- **Resolution.** Going from 180 to 720 rays doubles the detection range (17.3 → 33.5 m). Going to 1440 rays changes nothing: above 720 rays, resolution is no longer the limit.
- **Pitch.** The car's nose rises slightly while driving (−0.56°). Over 33.5 m, the beam climbs 0.33 m and passes over the barrel, which is 0.90 m tall.
- **Compensation.** Tilting the LiDAR down raises detection to 36.8 m. But the car tilts more while accelerating (−1.04°), so the beam passes over the barrel again.

**Conclusion.** A fixed correction is not enough, because the car's pitch changes when it accelerates and brakes. Reliable detection needs a multi-layer LiDAR that filters out the ground. The chosen setting is 720 rays: 1440 adds nothing and costs more computing time.

## Limitations

This simulation is a development and testing environment. It does not guarantee the safety of a real vehicle.

## Author

Chaima Ouerghi
