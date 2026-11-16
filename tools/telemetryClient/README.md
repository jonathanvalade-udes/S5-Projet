# FastDrink Telemetry Client & Dashboard Web (C++17)

Outil client TCP avec tableau de bord web interactif pour la télémétrie et le contrôle du robot FastDrink (PiCar-S), conçu pour tourner en **une seule commande** sur n'importe quel ordinateur (**macOS, Linux, Windows**).

---

## 🚀 Fonctionnalités Clés

* **Protocole TCP conforme à 100%** :
  * Connexion au port **TCP 91** de `TelemetryService`.
  * Poignée de main obligatoire **Ping/Pong** avec somme de contrôle **CRC32** IEEE 802.3.
  * Réception et désérialisation du flux continue de [`CarTelemetry`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/services/libs/shared-headers/car/full_packet.hpp).
* **Interface Utilisateur Moderne (Web locale)** :
  * Accessible sur `http://localhost:8080`.
  * Visualisation en direct des 5 capteurs optiques TCRT5000 (S1 à S5).
  * Jauge de distance HC-SR04 avec seuils de danger/sécurité.
  * Affichage des accélérations, papillons PWM gauche/droit et angle de braquage.
  * Badge d'état de la machine d'état FSM (`LineFollowingForward`, `CollisionRisk`, etc.).
* **Panneau de Commandes Interactif** :
  * Boutons d'action rapide : **Départ FSM** (`trigger_start`), **Arrêt d'Urgence** (`emergency_stop`), **Export USB** (`export_usb`).
  * Panneau **Réglages du robot** : 31 réglages en 7 groupes (vitesse, suivi de ligne, obstacle, évitement en 12 étapes, moteurs, capteurs, fréquences). Chaque ligne a un titre clair, une description, la valeur par défaut, la plage conseillée, les limites du robot et la valeur actuelle lue en direct. Recherche, bouton « Appliquer les valeurs modifiées » par groupe. Voir la section *Réglages du robot* plus bas.
  * Console terminal intégrée pour envoyer n'importe quelle commande personnalisée.
* **Respect des règles du projet** :
  * En-têtes [`services/libs/shared-headers`](file:///Users/jonathanvalade/Documents/Github/S5-Projet/services/libs/shared-headers) utilisés **strictement en lecture seule**.

---

## 🐳 Lancement avec Docker (Recommandé)

Aucune dépendance locale (pas besoin de compilateur C++, X11 ou VNC).

### 1. Lancer le client

Depuis la racine du projet ou depuis `tools/telemetryClient` :

```bash
cd tools/telemetryClient
docker compose up --build
```

Ouvrez ensuite votre navigateur sur : **[http://localhost:8080](http://localhost:8080)**

### 2. Se connecter au PiCar physique (Wi-Fi)

Si le robot a une adresse IP sur votre réseau local (ex: `192.168.1.50`) :

```bash
PICAR_HOST=192.168.1.50 docker compose up
```

---

## 💻 Compilation et Exécution Locale (Sans Docker)

Si vous disposez de CMake et d'un compilateur C++17 :

```bash
cd tools/telemetryClient
mkdir -p build && cd build
cmake ..
cmake --build .

# Exécution avec arguments personnalisés
./telemetry_client --ip 127.0.0.1 --port 91 --web-port 8080
```

### Options CLI disponibles :
* `--ip <host>` : Adresse IP de `TelemetryService` (défaut : `127.0.0.1` ou variable `PICAR_HOST`).
* `--port <port>` : Port TCP du robot (défaut : `91` ou variable `PICAR_PORT`).
* `--web-port <port>` : Port du serveur web local (défaut : `8080` ou variable `WEB_PORT`).
* `--web-root <dir>` : Répertoire des fichiers web (défaut : `web`).
* `--no-web` : Désactive le serveur web (mode console pure).

---

## ⚙️ Réglages du robot (panneau web)

Le panneau est **généré** à partir d'un seul catalogue, `PARAMS`, au début de la section « RÉGLAGES DU ROBOT » de `web/app.js`.

| Information affichée | Sens |
| --- | --- |
| **Défaut** | Valeur de `fsm.conf` |
| **Conseillé** | Plage qui marche en pratique. Un avertissement orange, jamais un blocage : on peut saisir n'importe quelle valeur |
| **Limite du robot** | Seule vraie limite : le logiciel du robot ramène la valeur dans cet intervalle (message rouge sous le champ) |
| **Actuel (robot)** | Valeur renvoyée par le robot dans la télémétrie. Après « Appliquer », l'étiquette indique *Confirmé par le robot*, ou *Le robot a gardé X au lieu de Y* après 4 s |
| *Pas de retour du robot* | Réglage que la télémétrie ne renvoie pas (`OBSTACLE_SLOWDOWN_DIST_CM`, `OBSTACLE_AVOID_MARGIN_CM`, `OBSTACLE_STOP_WAIT_S`) : l'envoi n'est pas confirmé, voir la console du FSM |

Ajouter un réglage : une entrée dans `PARAMS` (champs expliqués dans l'en-tête de la section). Le service concerné doit accepter la clé.

### Vérifier que le catalogue colle au robot

```bash
node tools/telemetryClient/tests/test_params_catalogue.js
```

Le test vérifie, pour chaque réglage : la clé existe dans le code du service ; la valeur « actuelle » existe dans le JSON de `ProtocolHandler.cpp` ; le défaut est celui de `fsm.conf` ; les limites affichées sont celles du code C++ ; et tout réglage accepté par le FSM est présent dans l'interface.

### Limites actuelles du robot (FSM)

Elles ont été élargies : seules des valeurs absurdes sont refusées.

| Réglage | Limite |
| --- | --- |
| `AVOID_*` durées | 0,05 à 30 s (recherche de ligne : 0,5 à 60 s) |
| `AVOID_*` angles | 0 à 45° (limite du servo) |
| `AVOID_*` vitesses | 0,05 à 0,60 m/s |
| `AVOID_REVERSE_DIST_M` | 0,02 à 3 m |
| `MAX_POWER_PCT` | 5 à 100 % |
| `STRAIGHT_CRUISE_SPEED` | 0,05 à 2 m/s |
| `CURVE_SAFETY_FACTOR` | 0,05 à 5 |
| `OBSTACLE_STOP_DIST_CM` | au moins 2 cm |
