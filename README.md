# Autonomous Vehicle Simulation — Digital Twin 3D

Simulation 3D d'un véhicule autonome dans **Webots** : la voiture roule, détecte les obstacles avec un **LiDAR**, adapte sa vitesse et s'arrête avant la collision. Chaque essai est enregistré puis analysé avec des métriques reproductibles.

> ▶ **Démo en ligne :** _lien webots.cloud à ajouter_
> 🎥 **Vidéo :** _lien à ajouter_

## Scénario S1 — Obstacle statique

| Paramètre | Valeur |
|---|---|
| Route | Ligne droite, 2 voies, 140 m |
| Véhicule | Tesla Model 3, voie de droite (y = −2 m) |
| Obstacle | Baril à 45 m devant |
| Vitesse de croisière | 30 km/h |
| Ralentissement | sous 20 m |
| Arrêt | sous 8 m |
| Graine aléatoire | 42 |

## Architecture

```
LiDAR (180–1440 rayons)   ──►  Perception : distance minimale dans un cône avant de 30°
GPS                        ──►  Position (x, y)
                                      │
                                      ▼
                      Décision : CRUISE → SLOW → STOP
                                      │
                                      ▼
              Contrôle : vitesse de croisière + freinage (API Driver)
                                      │
                                      ▼
                     Log CSV  ──►  analyze_run.py  ──►  Métriques
```

## Structure du dépôt

```
av-digital-twin/
├── Dockerfile                 # version de Webots utilisée sur webots.cloud
├── webots.yaml                # publication sur webots.cloud
├── worlds/
│   ├── av_lidar_stop.wbt      # scène S1, LiDAR 180 rayons (essai 1)
│   ├── av_lidar_stop_720.wbt  # scène S1, LiDAR 720 rayons (essai 2)
│   ├── av_lidar_stop_1440.wbt # scène S1, LiDAR 1440 rayons (essai 3)
│   ├── av_lidar_stop_diag.wbt # scène S1, LiDAR 720 + centrale inertielle (essai 4)
│   └── av_lidar_stop_tilt.wbt # scène S1, LiDAR incliné pour compenser le tangage (essai 5)
├── controllers/
│   └── av_lidar_stop/
│       └── av_lidar_stop.py   # perception, décision, contrôle, log
└── tools/
    └── analyze_run.py         # calcul des métriques
```

## Lancer en local

1. Installer **Webots R2025a** et **Python 3.11+**.
2. Ouvrir `worlds/av_lidar_stop.wbt` dans Webots.
3. Lancer la simulation (▶). Le log est écrit dans `controllers/av_lidar_stop/run_log_<N>rays.csv` (un fichier par résolution du LiDAR).
4. Calculer les métriques :

```bash
# compare automatiquement tous les essais enregistres
python tools/analyze_run.py
```

## Métriques

| Métrique | Description |
|---|---|
| `success` | Arrêt complet sans contact avec l'obstacle |
| `first_detect_m` | Distance de la première détection de l'obstacle |
| `stop_time_s` | Temps avant l'arrêt complet |
| `stop_distance_m` | Distance restante à l'obstacle à l'arrêt |
| `min_distance_m` | Distance minimale mesurée |
| `max_speed_kmh` | Vitesse maximale atteinte |
| `distance_driven_m` | Distance parcourue |
| `lateral_dev_max_m` | Écart latéral maximal par rapport au centre de la voie |

## Résultats

| Essai | LiDAR | Première détection | Distance d'arrêt | Réussite |
|---|---|---|---|---|
| 1 | 180 rayons | 17,3 m | 7,86 m | ✅ |
| 2 | 720 rayons | 33,5 m | 7,86 m | ✅ |
| 3 | 1440 rayons | identique à l'essai 2 | 7,9 m | ✅ |
| 4 | 720 rayons + diagnostic | 33,5 m (tangage −0,56°) | 7,9 m | ✅ |
| 5 | 720 rayons, LiDAR incliné −0,56° | 36,8 m (tangage −1,04° en accélération) | 7,9 m | ✅ |

**Conclusion intermédiaire.** Passer de 180 à 720 rayons double la distance de détection (17,3 → 33,5 m). Passer à 1440 rayons ne change plus rien : au-delà de 720 rayons, la résolution horizontale n'est plus le facteur limitant. L'essai 4 mesure un tangage de −0,56° (avant relevé) : le faisceau monte de 0,33 m sur 33,5 m et passe au-dessus du baril (haut à 0,90 m), ce qui correspond à un LiDAR à ≈ 0,57 m du sol, cohérent avec la géométrie du véhicule. L'essai 5 compense cette inclinaison : la détection passe à 36,8 m, mais elle a lieu pendant l'accélération, où la voiture se cabre davantage (−1,04°). Le faisceau remonte alors de ≈ 0,5° et passe de nouveau au-dessus du baril.

**Conclusion de l'étude LiDAR.** Une compensation fixe ne suffit pas, car l'assiette du véhicule varie avec l'accélération et le freinage. Une détection robuste demande un LiDAR multi-nappes avec filtrage du sol : c'est l'amélioration retenue pour la suite. Le réglage conservé est 720 rayons (1440 n'apporte rien et coûte plus en calcul).

## Feuille de route

- [x] S1 — Obstacle statique, arrêt d'urgence
- [ ] Suivi de voie sur route courbe
- [ ] Scénarios S2 à S5 (obstacle mobile, piéton, intersection, météo)
- [ ] Dashboard Digital Twin en temps réel (robot window)
- [ ] Publication sur webots.cloud

## Limites

La simulation est un environnement de développement et de test. Elle ne garantit pas la sécurité d'un véhicule réel.

## Auteure

Chaima Ouerghi
