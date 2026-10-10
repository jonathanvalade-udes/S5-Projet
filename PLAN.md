# Plan de Réalisation et Protocole de Test (PLAN.md) — FastDrink S5-GI

> **Approche** : Test-Driven Development (TDD) & Jalons Incrémentaux  
> **Source de départ** : `/Users/jonathanvalade/Downloads/CodeDepart`  
> **Règle Git** : Commits atomiques réguliers autorisés, **AUCUN PUSH DIRECT** sans validation explicite.

---

## 📋 Vue d'Ensemble des Jalons

| Jalon | Intitulé | Objectif Technique | Critère de Succès (Pass/Fail) |
| :---: | :--- | :--- | :--- |
| **0** | **Reset & Import Propre** | Remplacer l'existant par le `CodeDepart` officiel de l'enseignant | Godot s'ouvre sans erreur, environnement Python prêt |
| **1** | **Pont WebSocket (Ping-Pong)** | Valider la communication bidirectionnelle JSON (Godot $\leftrightarrow$ Python) | Échange continu stable à 20–50 Hz sans déconnexion |
| **2** | **Physique Bille & Accélération** | Intégrer la coupelle/bille et borner $a \le 0{,}735\text{ m/s}^2$ | Accélération et freinage sans chute de bille |
| **3** | **Suivi de Ligne & Virages** | Asservissement optique (S2) et rayon minimal $120\text{ mm}$ (S3) | 3 tours complets autonomes sans décrochage |
| **4** | **Ultrasons & Arrêt Cible** | Filtrage numérique (S7) et arrêt précis à $\pm 30\text{ mm}$ (S1) | 10 arrêts consécutifs dans la plage d'erreur $\pm 30\text{ mm}$ |
| **5** | **Évitement & Marche Arrière** | Contournement d'obstacle (S4) et recul $< 200\text{ mm}$ (S5) | Contournement autonome et retour stable sur la ligne |
| **6** | **Qualification & Piste Mystère** | Robustesse zéro-hardcoding (S8) et symétrie code réel/simu (S10) | Validation des 4 parcours (20 min) et piste inconnue |

---

## 🔍 Focus Détaillé : Étape 0 — Reset & Import Propre de CodeDepart

### Objectifs
1. Nettoyer l'espace de travail en préservant la documentation (`documentation/`), les livrables PDF, [**`PRD.md`**](file:///Users/jonathanvalade/Documents/Github/S5-Projet/PRD.md) et ce fichier [**`PLAN.md`**](file:///Users/jonathanvalade/Documents/Github/S5-Projet/PLAN.md).
2. Importer l'arborescence officielle depuis `/Users/jonathanvalade/Downloads/CodeDepart` :
   * `Dossier_GoDot/` $\longrightarrow$ `simulation/`
   * `Dossier_PiCar/` $\longrightarrow$ `robot/`
3. Nettoyer les fichiers résiduels temporaires ou caches macOS (`.DS_Store`, etc.).
4. Commit local de l'état initial : `feat: import du code de depart officiel (CodeDepart)`.

### Check-list d'exécution Étape 0
- [x] Sauvegarde de sécurité interne.
- [x] Nettoyage des dossiers `simulation/` et `robot/` actuels.
- [x] Copie conforme de `CodeDepart/Dossier_GoDot` dans `simulation/`.
- [x] Copie conforme de `CodeDepart/Dossier_PiCar` dans `robot/`.
- [x] Vérification de l'intégrité des fichiers importés (présence de `main.tscn`, `project.godot`, bibliothèques picar).
- [x] Commit Git local : `c335394`.

---

## 🔍 Focus Détaillé : Étape 1 — Le Pont de Communication (Godot ↔ Python)

### Objectifs
Établir une liaison WebSocket bidirectionnelle robuste et asynchrone entre le simulateur Godot 4 et le contrôleur Python sur `127.0.0.1:8765` (ou Wi-Fi), sans aucune logique complexe de navigation pour l'instant.

### Architecture des Échanges (Spécification JSON minimale)
1. **Paquet Télémétrie (Godot $\to$ Python)** :
   ```json
   {
     "packet_id": 102,
     "timestamp": 12.45,
     "distance": 85.0,
     "line_follower": [0, 0, 1, 0, 0]
   }
   ```
2. **Paquet Commande (Python $\to$ Godot)** :
   ```json
   {
     "packet_id": 102,
     "steering": 0.0,
     "speed": 0.0,
     "brake": false
   }
   ```

### Tâches d'implémentation Étape 1
1. **Côté Python (`robot/server_bridge.py`)** :
   * Créer un serveur WebSocket asynchrone (`websockets` / `asyncio`) écoutant sur le port `8765`.
   * Gérer la connexion / déconnexion propre du client sans crash.
   * Réceptionner la télémétrie reçue de Godot, logger les données (distance, capteurs de ligne).
   * Renvoyer immédiatement une réponse formatée JSON avec accusé de réception (`ack`) et consignes de test.
2. **Côté Godot (`simulation/scripts/`)** :
   * Analyser l'état actuel de `NetworkFSM` (`NetworkInitState.gd`, `NetworkProcessState.gd`, etc.).
   * Valider que le client Godot sérialise bien les données des capteurs et parse correctement la réponse JSON du serveur Python.
   * Assurer la reconnexion automatique en cas de coupure.
3. **Protocole de Test Étape 1 (Critère de validation Pass/Fail)** :
   * [x] **Test 1.1** : Démarrer le serveur Python $\rightarrow$ Démarrer le client $\rightarrow$ Connexion validée.
   * [x] **Test 1.2** : Observer l'échange continu de paquets (30 paquets séquentiels validés).
   * [x] **Test 1.3** : Valider la cadence et la résilience aux trames invalides (serveur stable, pas de crash).
   * [x] **Test 1.4** : Couper et reconnecter le client $\rightarrow$ reconnexion validée sans redémarrage serveur.
   * [x] **Commit local** : `feat(network): pont websocket bidirectionnel robuste et teste (Etape 1)` (`48227a4`).

---

## 🔍 Plan Sommaire des Étapes Suivantes

### Étape 2 : Physique Bille & Générateur de Vitesse ($a \le 0{,}735\text{ m/s}^2$)
* [x] Modélisation de la surveillance de déversement de la bille (`RigidBody3D` + `ball_monitor.gd`) dans Godot.
* [x] Module Python `motion_profiler.py` appliquant des rampes trapézoïdales bornées à $a \le 0{,}735\text{ m/s}^2$.
* [x] Calcul de vitesse limite en virage $v \le \sqrt{a \cdot R_{\min}} \approx 0{,}297\text{ m/s}$.
* [x] **Tests unitaires validés** (`test_motion_profiler.py`) : 4/4 tests réussis.
* [x] **Test d'intégration WebSocket validé** (`test_client.py`) : rampe d'accélération et arrêt d'urgence sur déversement bille.

### Étape 3 : Suivi de Ligne Optique & Virages ($R \ge 120\text{ mm}$)
* Connexion des 5 (ou 3) raycasts optiques au contrôleur de direction (PD).
* Calcul et bridage automatique de la vitesse en virage ($v \le 0{,}297\text{ m/s}$).
* **Test** : 3 tours consécutifs du circuit fermé Godot en autonomie totale.

### Étape 4 : Capteur Ultrasons, Filtrage Numérique & Arrêt $\pm 30\text{ mm}$
* Module de filtrage (médian + moyenne mobile EMA) pour rejeter le bruit ultrasons.
* Loi de décélération progressive pour immobilisation à distance cible.
* **Test** : 10 arrêts consécutifs face à un obstacle avec écart mesuré inférieur à $\pm 30\text{ mm}$.

### Étape 5 : Contournement d'Obstacle & Recul Guidé
* Machine à états finis (`FSM`) : Détection $\to$ Arrêt $\to$ Déviation $\to$ Rattrapage de ligne.
* Routine de recul guidé sur ruban noir pour distance $< 200\text{ mm}$.
* **Test** : Parcours avec obstacle bloquant la trajectoire franchi avec succès.

### Étape 6 : Qualification Finale & Validation Multi-Parcours
* Évaluation sur les 4 parcours d'essais (séance *Conduite à l'épreuve*).
* Test sur circuit aléatoire/inconnu pour garantir l'absence de hardcoding (préparation à la *piste mystère*).
* Préparation du déploiement réel sur le Raspberry Pi.
