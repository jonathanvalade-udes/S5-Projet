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
  * Ajustement en un clic des paramètres dynamiques (`MAX_ACCELERATION`, `CRUISE_SPEED`, `MIN_CURVE_SPEED`, `LINE_KP`, `LINE_KD`).
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
