# Outil de Rejeu, Modélisation & Analyse CSV de Télémétrie FastDrink / PiCar-S

Cet outil complet permet de charger, modéliser et visualiser en détail les fichiers CSV générés par la pile de télémétrie embarquée FastDrink (`tools/telemetryClient` / `CarTelemetry` 78 colonnes).

---

## Fonctionnalités Principales

### 1. Modélisation 2D du Comportement du Robot (Thing #1)
- **Reconstruction Odométrique Ackermann (Bicycle Model)** :
  - Calcule la trajectoire continue $(x, y, \theta)$ en mètres à partir des encodeurs de propulsion et de l'angle de braquage de direction $\delta$.
- **Rendu Visuel 2D Dynamique (Canvas)** :
  - Affichage à l'échelle du châssis PiCar-S.
  - **Roues directrices avant orientées dynamiquement** selon `steering_current_angle`.
  - **Barre 5 capteurs TCRT5000** sur le pare-choc avant avec diodes électroluminescentes reflétant en temps réel l'intensité optique du ruban noir ($S_0 \dots S_4$).
  - **Cône de détection sonar ultrason** s'étendant à la distance réelle mesurée (`sonar_calc_dist_cm`), avec transition visuelle vert / orange / rouge pulsant en cas de proximité d'obstacle (< 25 cm).
  - **Trace de trajectoire complète colorée par état FSM** (ex: vert pour `LineFollowingForward`, bleu/violet pour les déviations, orange pour `CollisionRisk`, rouge pour `Stopped`).
- **Contrôles de Rejeu Chronologique** :
  - Lecture / Pause (Touche `Espace`).
  - Défilement temporel pas-à-pas (+/- 1 trame ou +/- 10 trames avec touches `←` / `→`).
  - Vitesse réglable : `0.25x`, `0.5x`, `1x`, `2x`, `5x`.
  - Mode boucle.
  - Navigation caméra : Pan à la souris, Zoom molette, « Suivre Robot », « Vue Globale ».

### 2. Graphiques Détaillés Synchronisés (Thing #2)
7 panneaux de graphiques synchronisés avec curseur d'inspection temporel lié au scrubber :
1. **Machine à États FSM & Modes Moteurs** : `fsm_current_state`, `fsm_requested_state`, `motors_state` avec détection des transitions.
2. **Suiveur de Ligne 5 Voies** : Voies $S_0, S_1, S_2, S_3, S_4$ commutables en valeurs **Normalisées [0..1]**, **Brutes ADC [0..4095]** ou **Correcteur**.
3. **Sonar Ultrasons** : Distance brute `sonar_raw_dist_cm` vs Distance filtrée par EMA `sonar_calc_dist_cm` + seuil de risque collision (20 cm).
4. **Propulsion & Moteurs** : Consignes cible vs appliqué (moteur gauche et moteur droit), encodeurs de roues.
5. **Direction & Servomoteurs** : Angle cible vs angle appliqué (degrés), registres de comparaison CCR PWM.
6. **Régulateurs PID Moteurs** : Évolution des coefficients $K_p, K_d, K_i$.
7. **Diagnostic & Daemons IPC** : Statuts des 4 daemons de supervision et codes d'erreur système actifs.

---

## Utilisation

### Option A : Lancement via le Serveur Python (Recommandé)
```bash
cd tools/csvReader

# Lancer avec le fichier d'exemple inclus
./run_viewer.py

# Ou lancer directement avec votre propre fichier CSV
./run_viewer.py /chemin/vers/mon_fichier_telemetrie.csv
```
Le visualiseur s'ouvre automatiquement dans votre navigateur par défaut à l'adresse `http://localhost:8080/index.html`.

### Option B : Glisser-Déposer Direct (100% Hors-Ligne)
Ouvrez directement `tools/csvReader/index.html` dans n'importe quel navigateur (Chrome, Firefox, Edge) et glissez-déposez n'importe quel fichier `.csv` sur la zone dédiée.

### Option C : Analyse Statistique en Ligne de Commande (CLI)
Pour une inspection rapide d'un log de roulage sans interface graphique :
```bash
./analyze.py sample_telemetry.csv
```

### Option D : Générer un Nouvel Échantillon de Test
```bash
./generate_sample_csv.py
```

---

## Structure des Fichiers
- `index.html` : Interface web complète et tableau de bord.
- `style.css` : Thème sombre moderne, instrumentation télémétrique et visualisations réactives.
- `app.js` : Moteur de parsing CSV, modèle cinématique 2D, rendu Canvas et synchronisation Chart.js.
- `lib/chart.umd.min.js` : Bibliothèque graphique Chart.js embarquée localement (garantit un fonctionnement 100% hors-ligne sans connexion Internet).
- `run_viewer.py` : Serveur HTTP local léger et lanceur de navigateur.
- `analyze.py` : Analyseur statistique CLI textuel.
- `generate_sample_csv.py` : Générateur d'échantillons de roulage complets (78 colonnes).
- `sample_telemetry.csv` : Log d'exemple pré-généré pour test immédiat.
