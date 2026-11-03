# Plan de Réalisation et Protocole de Test (PLAN.md) — FastDrink S5-GI

> **Approche** : Test-Driven Development (TDD) & Architecture Microservices C++17  
> **Simulation** : Jumeau numérique Godot Engine 4 (`simulation/`) connecté en TCP natif (port 9000)  
> **Cible Embarquée** : SunFounder PiCar-S (Raspberry Pi ARM64 / Linux) piloté par `/Services`  
> **Règle Git** : Commits atomiques réguliers, validation rigoureuse avant push.

---

## 📋 Vue d'Ensemble des Jalons

| Jalon | Intitulé | Objectif Technique | Statut | Critère de Succès (Pass/Fail) |
| :---: | :--- | :--- | :---: | :--- |
| **0** | **Reset & Organisation du Répertoire** | Intégration propre de `CodeDepart` et mise en place de `/Services` | **VALIDÉ ✅** | Godot 4 fonctionnel, suite de compilation C++ opérationnelle |
| **1** | **Pont de Communication TCP (Godot $\leftrightarrow$ C++)** | Liaison temps réel 50 Hz via `SimBridge` (port TCP 9000) | **VALIDÉ ✅** | Échange bidirectionnel continu, reconnexion automatique sans perte de trame |
| **2** | **Physique Bille & Accélération Bornée** | Surveillance 3D et bridage strict $\mathbf{a \le 0{,}735\text{ m/s}^2}$ (`MotionProfiler`) | **VALIDÉ ✅** | Accélérations et freinages fluides sans éjection de la bille |
| **3** | **Suivi de Ligne & Virages ($R \ge 120\text{ mm}$)** | Régulateur PD barycentrique (`LineFollowerController`) sur ruban 18 mm | **VALIDÉ ✅** | Négociation des virages sans décrochage ($v \le 0{,}297\text{ m/s}$) |
| **4** | **Ultrasons HC-SR04 & Arrêt Précision** | Pipeline numérique (Médian + Kalman 1D) et arrêt cible $\pm 30\text{ mm}$ (S1) | **VALIDÉ ✅** | Rejet des anomalies et immobilisation progressive à 20 cm |
| **5** | **Évitement d'Obstacle & Recul Guidé** | Machine à états finis (`FSM`) pour contournement (S4) et recul $< 200\text{ mm}$ (S5) | **VALIDÉ ✅** | Déviation latérale, réacquisition du ruban et recul autonome validés |
| **6** | **Qualification Finale & Déploiement Réel** | Validation sur piste inconnue/mystère (S8) et déploiement PiCar matériel | **EN COURS 🔄** | Bouclage complet autonome des parcours d'évaluation |

---

## 🔍 Focus Détaillé par Jalon

### Jalon 0 : Organisation & Architecture
* [x] Importation conforme des scènes Godot 4 (`simulation/`).
* [x] Mise en place de l'architecture microservices modulaire en C++17 sous [`Services/`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services).
* [x] Nettoyage complet des fichiers temporaires, doublons et squelettes inutilisés.
* [x] Configuration de la chaîne de compilation native et cross-compilation ARM64 (`build.sh`, CMake).

### Jalon 1 : Pont Réseau Bidirectionnel (Godot ↔ `SimBridge` C++)
* [x] Service [`SimBridge`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services/SimBridge) écoutant sur le port TCP `9000`.
* [x] Client Godot [`network_client.gd`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/simulation/scripts/network_client.gd) transmettant télémétrie capteurs et recevant les consignes moteur/direction à 50 Hz.
* [x] Synchronisation ultra-rapide par mémoire partagée POSIX SHM (`/dev/shm/picar_shm`).
* [x] Test de résilience et reconnexion automatique validé (`test_sim_bridge.cpp`).

### Jalon 2 : Physique Bille & Profilage Cinématique ($a \le 0{,}735\text{ m/s}^2$)
* [x] Surveillance de déversement de la bille dans Godot (`RigidBody3D` + `ball_monitor.gd`).
* [x] Module C++ [`motion_profiler.hpp`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services/libs/shared-headers/control/motion_profiler.hpp) garantissant $\Delta v \le a_{\max} \cdot \Delta t = 0{,}0147\text{ m/s}$ par cycle de 20 ms.
* [x] Vitesse critique en courbe bornée : $v \le \sqrt{a_{\max} \cdot R_{\min}} = \sqrt{0{,}735 \cdot 0{,}12} \approx 0{,}297\text{ m/s}$.
* [x] Tests unitaires cinématiques validés à 100 % (`test_control.cpp`).

### Jalon 3 : Suivi de Ligne Optique & Virages ($R \ge 120\text{ mm}$)
* [x] Contrôleur PD unifié [`line_follower.hpp`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services/libs/shared-headers/control/line_follower.hpp) avec calcul d'écart barycentrique sur 5 capteurs TCRT5000.
* [x] Modulation automatique de vitesse en virage sous la limite critique centrifuge.
* [x] Détection fiable de la barre d'arrivée en T (`[1, 1, 1, 1, 1]`) déclenchant l'arrêt de fin de course.
* [x] Mémoire de trajectoire anti-décrochage lors des transitions rapides.

### Jalon 4 : Capteur Ultrasons, Filtrage Numérique & Arrêt $\pm 30\text{ mm}$
* [x] Pipeline numérique à double étage dans [`ultrasonic_pipeline.hpp`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services/libs/shared-headers/filters/ultrasonic_pipeline.hpp) :
  * Filtre médian à fenêtre glissante (5 échantillons) pour éliminer les valeurs aberrantes.
  * Filtre de Kalman 1D adaptatif compensé par la vitesse instantanée du véhicule.
* [x] Profil d'approche sécuritaire calculé par conservation d'énergie : $v_{\max}(d) = \sqrt{2 \cdot a_{\max} \cdot d}$.
* [x] Tests unitaires de filtrage validés à 100 % (`test_filters.cpp`).

### Jalon 5 : Évitement d'Obstacle (S4) & Recul Guidé (S5)
* [x] Machine à états finis complète dans [`Services/FSM/src/FSM.cpp`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services/FSM/src/FSM.cpp) :
  * **`CollisionRisk` $\to$ `Stopped`** : Immobilisation complète à $20\text{ cm}$ de l'obstacle sans chute de bille.
  * **`RightDeviation` (S4)** : Braquage à droite (+25°) à $0{,}15\text{ m/s}$ pendant 1,2 s pour contourner l'obstacle.
  * **`LineSearching` (S4)** : Contre-braquage (-24°) pour recouper la piste et réacquérir le ruban noir.
  * **`LineDetected` (S4)** : Verrouillage des capteurs optiques et reprise nominale du suivi en ligne droite.
  * **`Reverse` (S5)** : Marche arrière autonome guidée sur ruban noir mesurée à **$144\text{ mm}$** ($< 200\text{ mm}$ requis).
* [x] Tests unitaires FSM validés à 100 % (`test_control.cpp`).

### Jalon 5.1 : Hub de Télémétrie & Supervision (`TelemetryService`)
* [x] Service [`TelemetryService`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/Services/TelemetryService) agrégeant la SHM POSIX et publiant `shared::car::CarTelemetry`.
* [x] Calcul en direct de l'accélération réelle ($a = \Delta v / \Delta t$) avec alerte de violation FastDrink ($a > 0{,}735\text{ m/s}^2$).
* [x] Intégration odométrique continue ($d = \int |v| \, dt$) et Watchdog anti-blocage (détection de gel heartbeat $> 500\text{ ms}$).
* [x] Dashboard console dynamique (5 Hz) et diffusion réseau UDP broadcast (port 9002).
* [x] Tests unitaires de télémétrie validés à 100 % (`test_telemetry.cpp`).

---

## 🎯 Prochaines Actions : Jalon 6 (Qualification & Déploiement Réel)

1. **Validation dynamique en simulation Godot 4 :**
   * [x] Pile C++ lançable sous WSL2 via `Services/run_sim.ps1` (SimBridge + FSM ; Sensors/Motors volontairement non lancés en simulation). Test de fumée avec faux client Godot validé (croisière, déviation, obstacle S4, arrêt balle).
   * [ ] Exécuter le parcours complet avec obstacle dans Godot (« Localhost » → Connect) et comparer au comportement Python (tour ≈ 23 s).
2. **Essais physiques sur le robot réel (PiCar-S) :**
   * Compilation ARM64 via `./build.sh --embedded`.
   * Déploiement des binaires sur le Raspberry Pi (`/root/picar_brain`, `/root/picar_motor`, `/root/picar_sensors`).
   * Essai sur piste d'entraînement et qualification face à la piste inconnue/mystère.
