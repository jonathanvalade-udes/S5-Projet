# Simulation PiCar-S sous Webots

Ce répertoire contient l'environnement de simulation 3D du robot **PiCar-S** sous [Webots](https://cyberbotics.com/), conçu comme alternative moderne et réaliste à Godot pour le projet S5.

---

## 1. Vue d'ensemble de l'architecture

Le simulateur Webots remplace le client de rendu Godot et communique directement avec la pile logicielle C++ embarquée via le service `SimBridge` sur une liaison TCP locale (port 9000).

```
                      +---------------------------------------+
                      |            Simulateur Webots          |
                      |   - Monde 3D (picar_s_track.wbt)      |
                      |   - Robot PiCar-S (PiCarS.proto)      |
                      |   - Contrôleur Python (50 Hz)         |
                      +-------------------+-------------------+
                                          |
                                          | TCP Socket (port 9000)
                                          | JSON (50 Hz) / Commandes texte
                                          v
                      +---------------------------------------+
                      |           Service SimBridge           |
                      |   (services/SimBridge)                |
                      +-------------------+-------------------+
                                          |
                                          | Mémoire partagée IPC (/picar_shm)
                                          | + Tubes FIFO POSIX (trigger_start)
                                          v
                      +---------------------------------------+
                      |              Pile C++                 |
                      |   - FSM (Contrôleur de ligne & états) |
                      |   - TelemetryService (Télémétrie)     |
                      |   - PeripheralsService (Balle, etc.)  |
                      +---------------------------------------+
```

### Principes clés
- **Fréquence temps réel** : la boucle de contrôle et de télémétrie tourne à **50 Hz** (20 ms) ; la physique tourne à 4 ms
  (`basicTimeStep 4`) pour des contacts roue/sol stables.
- **Géométrie (CAD + FSM)** : empattement 0,14 m (= `WHEELBASE_M` du FSM), voie 0,13 m, roues de 67 mm, pivots de direction à ±50 mm (CAD), masse ~0,61 kg
  centrée vers l'essieu arrière. Pneus : frottement 0,9, aucun rebond.
- **Capteurs modélisés** :
  - **5 capteurs infrarouges (IR_0 à IR_4)**, pas de 19 mm (`irPitch`), 0,17 m devant l'essieu arrière, à 1 cm du sol.
    La table de chaque capteur est **linéaire** dans l'espace « distance / facteur de réflexion » de Webots : la sortie est
    proportionnelle à la fraction de la tache (7 mm, `irSpotDiameter`) couverte par le ruban, donc une vraie valeur continue
    `[0.0, 1.0]`, pas un 0/1. Bruit relatif 1 % (`irNoise`).
  - **1 capteur ultrasonique (SONAR)** SunFounder 25 kHz : cône d'environ 30°, portée 2 cm à 4 m.
- **Actionneurs modélisés** :
  - **Direction d'Ackermann** : un pivot par roue avant (`left_steer`, `right_steer`). La commande est l'angle bicyclette du
    FSM (±45°, R = L / tan δ) ; le contrôleur en déduit l'angle de chaque roue (intérieure 61,8° / extérieure 34,3° à 45°).
  - **Différentiel** : un moteur par roue arrière (`rear_left_motor`, `rear_right_motor`, comme les 2 moteurs TT réels),
    chacun à la vitesse de son propre rayon, $\omega = v / R$ ($R = 33{,}5$ mm).
- **Piste** : ovale 2,4 m × 1,0 m (droites 1,4 m, virages R = 0,5 m), ruban de **18 mm**.

---

## 2. Guide de démarrage rapide

### En une commande (Docker + Webots)

```bash
./webots/run_all.sh                 # circuit Godot sans obstacles, départ automatique
./webots/run_all.sh --obstacles     # circuit Godot avec les 2 obstacles
./webots/run_all.sh --oval          # ovale simple
./webots/run_all.sh --rebuild       # après une modification du code C++
```

Le script (re)démarre la pile C++ dans Docker (le FSM repart de zéro), attend qu'elle soit prête, ouvre Webots, et arrête la
pile quand on ferme Webots. Options : `--manual` (départ avec la touche S), `KEEP_STACK=1`, `WEBOTS_DEBUG=1`,
`WEBOTS_TRUTH_LOG=fichier.csv`. Détails : `./webots/run_all.sh --help`.

### En deux terminaux

### Étape 1 : Démarrer la pile C++ (SimBridge + FSM)

Depuis la racine du projet ou le dossier `services/` :

```bash
cd services
./build.sh --sim
```

*Alternative avec le script dédié (Linux / WSL2) :*
```bash
./services/run_sim.sh
```

Le FSM de simulation lit une copie de `fsm.conf` où les clés de `services/fsm.sim.conf` sont remplacées. Aucune surcharge
n'est active : la simulation utilise les gains du robot (LINE_KP=0.58 / LINE_KD=0.035). Le `fsm.conf` du robot n'est jamais
modifié ; pour essayer d'autres gains en simulation seulement, décommenter une ligne de `fsm.sim.conf`.

Cette commande initialise la mémoire partagée `/picar_shm`, démarre le service `SimBridge` en écoute sur le port TCP `9000`, puis lance la machine à états finis `FSM`.

### Étape 2 : Lancer la simulation Webots

Dans un autre terminal, exécutez le script de lancement automatique :

```bash
./webots/run_webots.sh
```

Le script détecte automatiquement Webots sous macOS (`/Applications/Webots.app`) ou Linux (`which webots`), vérifie la présence de la texture de piste et démarre la simulation.

#### Options disponibles pour `run_webots.sh` :

| Option | Description |
| :--- | :--- |
| `-h`, `--help` | Affiche l'aide complète. |
| `-s`, `--auto-start` | Démarre la course automatiquement après 2s de stabilisation. |
| `-H`, `--host <IP>` | Adresse IP de SimBridge (par défaut: `127.0.0.1`). |
| `-p`, `--port <PORT>` | Port TCP de SimBridge (par défaut: `9000`). |
| `-m`, `--mode <MODE>` | Mode d'exécution Webots (`realtime`, `fast`, `pause`). |
| `--world <FILE>` | Spécifie un fichier de monde `.wbt` personnalisé. |

*Exemple avec démarrage autonome :*
```bash
./webots/run_webots.sh --auto-start
```

---

## 3. Commandes interactives au clavier

Une fois la fenêtre Webots active, cliquez dans la vue 3D pour donner le focus, puis utilisez :

| Touche | Action | Effet système |
| :---: | :--- | :--- |
| **`S`** ou **`s`** | **Départ de la mission** | Envoie `trigger_start\n` à SimBridge, qui déclenche l'événement dans le FSM pour quitter l'état `Idle` et débuter le suivi de ligne. |
| **`Espace`** | **Arrêt d'urgence** | Envoie `emergency_stop\n` à SimBridge, qui force le FSM à basculer immédiatement en état `EmergencyStop`. |

---

## 4. Protocole de communication TCP

### Webots → SimBridge (50 Hz)
Chaque trame est un objet JSON sur une ligne terminée par `\n` :
```json
{
  "distance": 42.50,
  "velocity": 0.2500,
  "line_follower": [0, 0, 1, 0, 0],
  "line_analog": [0.0, 0.0, 0.85, 0.0, 0.0],
  "ball_dropped": false
}
```
- `distance` : Distance sonar en **centimètres**.
- `velocity` : Vitesse linéaire courante du véhicule en m/s.
- `line_follower` : Tableau de 5 entiers binaires `[G2, G1, C, D1, D2]` (1 = ruban noir détecté).
- `line_analog` : Noirceur continue `[0.0, 1.0]` (0.0 = blanc, 1.0 = noir) = fraction de la tache du capteur sur le ruban.
- `ball_dropped` : Booléen signalant si la balle a été larguée.

### SimBridge → Webots (50 Hz)
Réponse envoyée par SimBridge à chaque trame :
```json
{
  "velocity": 0.3200,
  "direction": -0.1500,
  "brake": false
}
```
- `velocity` : Vitesse linéaire commandée en m/s.
- `direction` : Angle de braquage normalisé dans `[-1.0, 1.0]` (négatif = droite, positif = gauche).
- `brake` : Booléen commandant l'arrêt immédiat des roues.

---

## 5. Structure des dossiers

```
webots/
├── README.md                           # Documentation générale du sous-système Webots
├── run_webots.sh                       # Script de détection et de lancement de Webots
├── cad/                                # Modèles 3D Collada (.dae)
│   ├── Chassis.dae                     # Châssis principal du PiCar-S
│   ├── WheelFrontLeft.dae              # Roue avant gauche
│   ├── WheelFrontRight.dae             # Roue avant droite
│   ├── WheelRearLeft.dae               # Roue arrière gauche
│   └── WheelRearRight.dae              # Roue arrière droite
├── protos/                             # Définition de composants Webots PROTO
│   └── PiCarS.proto                    # Modèle physique complet du robot PiCar-S
├── controllers/                        # Contrôleurs Webots
│   └── picar_s_controller/
│       ├── picar_protocol.py           # Logique pure de sérialisation et protocole
│       └── picar_s_controller.py       # Contrôleur Webots temps réel 50 Hz
├── worlds/                             # Mondes et scènes Webots
│   ├── picar_s_track.wbt               # Monde principal avec piste et éclairage
│   └── textures/
│       └── line_track.png              # Texture haute résolution du circuit ovale
├── tools/                              # Utilitaires et scripts de génération
│   └── generate_track_texture.py       # Générateur de la texture du circuit
└── tests/                              # Suite de tests unitaires et d'intégration
    ├── test_cad_assets.py              # Validation des fichiers CAD Collada
    ├── test_proto_syntax.py            # Validation syntaxique et physique de PiCarS.proto
    ├── test_protocol.py                # Tests unitaires du protocole et contrôleur
    ├── test_world_track.py             # Validation du monde et de la texture
    └── test_e2e_mock_bridge.py         # Test d'intégration de bout en bout (SimBridge mock)
```

---

## 6. Exécution des tests

L'ensemble des tests du sous-système Webots s'exécute sans dépendre de l'installation de Webots (via des mocks modulaires et découplés) :

```bash
# Lancer tous les tests Webots
python3 -m unittest discover -s webots/tests -p "test_*.py" -v
```

### Circuit de Godot, bille et support

`worlds/picar_s_course.wbt` reprend le circuit de Godot (demi-tours, virages à 90°, 2 obstacles, barre d'arrivée en T,
ruban de 18 mm) avec la **bille physique** (verre, Ø 20 mm, 10 g) dans la coupelle du support (`cad/Plaquette.obj`, bol de
Godot). Le robot y est superviseur : `ball_dropped` vient du moteur physique (`ball_tracker.py`).

```bash
./webots/run_webots.sh --world webots/worlds/picar_s_course.wbt --auto-start
python3 webots/tools/ball_lab.py                                          # banc d'essai de la bille
python3 webots/tools/run_closed_loop.py --course --external --duration 60 # course contre la vraie pile (temps reel)
```

Régénérer les fichiers dérivés de Godot (Blender requis) :
```bash
blender -b godot/assets/plaquette_blender_scale.blend --python webots/tools/export_godot_assets.py -- plaquette
blender -b godot/assets/parcours.blend --python webots/tools/export_godot_assets.py -- parcours
python3 webots/tools/generate_course_texture.py
```

### Mesure en boucle fermée (Webots réel, sans Docker)

`tools/run_closed_loop.py` compile `tools/loop_bridge.cpp` (un pont TCP qui applique le **vrai** `LineFollowerController` et la
vitesse en courbe d'Ackermann du FSM), lance Webots sans rendu, puis résume la vérité terrain (GPS/IMU) : écart latéral au
ruban, temps passé en butée de braquage, à-coups de braquage.

```bash
python3 webots/tools/run_closed_loop.py --duration 60 --out /tmp/run          # réglages par défaut
python3 webots/tools/run_closed_loop.py --proto irPitch=0.015 --out /tmp/p15  # autre pas de capteurs
WEBOTS_CALIBRATION=cal.json python3 webots/tools/run_closed_loop.py           # autre calibration dynamique
LOOP_BRIDGE_GAP_FIX=1 python3 webots/tools/run_closed_loop.py                 # émule le correctif FSM proposé
```

Balayage de robustesse (gains contre les paramètres réels inconnus, plusieurs Webots en parallèle) :

```bash
python3 webots/tools/sweep_robustness.py --grid full --gains 0.35:0.035 0.58:0.035 --jobs 6 --out /tmp/sweep
python3 webots/tools/sweep_robustness.py --grid coarse --track-radius 0.25 --out /tmp/sweep_tight   # virages serrés
```

`WEBOTS_TRUTH_LOG=fichier.csv` fait écrire au contrôleur la vérité terrain à chaque pas (position, cap, vitesse, commande, IR).

Pour lancer uniquement le test de bout en bout SimBridge :
```bash
python3 -m unittest webots/tests/test_e2e_mock_bridge.py -v
```


---

## 6. Modèles dynamiques et calibration (fidélité du robot)

Entre la consigne du FSM et les moteurs Webots, `controllers/picar_s_controller/picar_dynamics.py` ajoute : latence
supplémentaire (35 ms, en plus des ~20-40 ms déjà présents dans la chaîne simulée), zone morte du moteur (20 % de puissance),
réponse du premier ordre, le **limiteur de braquage de `MotorActuator`** (175 °/s sous 15°, 300 °/s au-delà), le servo
(500 °/s, SF0180 à 4,8 V, jeu) et le rapport servo → roue (`steer_ratio`), et la
**bille FastDrink** calculée comme dans `godot/scripts/ball_monitor.gd` (a_crit = g·tanθ = 0,7358 m/s²). La coupelle (1,5 mm de
profondeur) est trop fine pour le moteur physique ; la bille est donc déduite de l'accélération du modèle de châssis.
Actif par défaut dans `run_webots.sh` (`WEBOTS_DYNAMICS=0` pour le désactiver). Le télémètre renvoie `ball_dropped`.

**Les valeurs par défaut sont PROVISOIRES** (`measured=false`). Seule la vitesse de 0,32 m/s à 50 % est mesurée. Exemple de
sensibilité : le pic d'accélération au départ vaut 3,2 × le seuil avec tau = 0,05 s, 1,2 × avec 0,15 s et 0,8 × avec 0,30 s.
Il faut donc mesurer sur le vrai robot (échelon, paliers de puissance, vitesse du servo), puis :

```bash
python3 webots/tools/calibrate_dynamics.py --step step.csv --plateaus plateaus.csv --steer steer.csv
```

Le script écrit `controllers/picar_s_controller/calibration.json` (procédure de mesure dans son en-tête).
Tant que `measured` est faux, la simulation sert à **comparer** des réglages, pas à fixer leurs valeurs finales.
