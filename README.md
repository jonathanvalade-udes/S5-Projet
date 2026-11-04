# S5-Projet : Robotique Autonome PiCar

Ce dépôt contient l'ensemble des microservices C++17 embarqués pour le robot SunFounder PiCar-S (architecture Raspberry Pi 3/4 sous Linux ARM64).

---

## 🚀 Compilation Rapide (Mac & Linux)

### 1. Pré-requis
* [Docker Desktop](https://www.docker.com/) (actif sur votre machine).
* Git & CMake (optionnel sur l'hôte, car Docker gère tout).

### 2. Compilation des binaires embarqués pour le Raspberry Pi
Exécutez simplement depuis la racine du projet :
```bash
./services/build.sh --embedded --static
```

> **Magie du cache Docker :**
> - La **première fois**, le script détecte que l'image `alpine-picar-builder` n'existe pas et la construit automatiquement à partir de `services/Dockerfile.builder`.
> - Les **fois suivantes**, l'image est déjà prête et la compilation ne prend que **1 à 3 secondes** !

### 3. Compiler un seul service spécifique
```bash
./services/build.sh --embedded --static Motors
./services/build.sh --embedded --static Sensors
./services/build.sh --embedded --static FSM
./services/build.sh --embedded --static TelemetryService
```

### 4. (Optionnel) Pré-construire l'image Docker manuellement
```bash
./services/build.sh --build-docker
```
*Ou via Docker Compose :*
```bash
docker compose -f services/docker-compose.yml build
```

---

## 🧪 Exécution des Tests Automatisés (Docker & Linux)

Puisque le projet est standardisé sur l'environnement cible Linux (Raspberry Pi), la suite de tests (`ctest`) s'exécute directement dans le conteneur Linux sur Mac et Windows.

### Lancer tous les tests en une commande (Recommandé) :
```bash
./services/build.sh --test
```
*(Le script lance Docker en arrière-plan, compile et exécute automatiquement la suite de 7 tests de robustesse : IPC, mémoire partagée, filtres, contrôle, et FSM).*

### Ou via Docker Compose :
```bash
docker compose -f services/docker-compose.yml run builder sh -c "cmake -B build-embedded -S . -DSTATIC=ON && cmake --build build-embedded && ctest --test-dir build-embedded --output-on-failure"
```

---

## 🎮 Simulation 3D Godot via Docker (Mac & Windows)

Vous pouvez lancer la pile logicielle de simulation (`SimBridge` + `FSM`) sous Docker sans avoir besoin d'installer Linux ou WSL :

```bash
./services/build.sh --sim
# ou
docker compose -f services/docker-compose.yml up simulation
```
*Puis dans l'application Godot sur votre Mac ou PC Windows : cochez **Localhost** (`127.0.0.1:9000`) et cliquez sur **Connect**.*

---

## 📡 Déploiement sur le Robot (Raspberry Pi)

Transférer tous les binaires compilés sur le robot via SCP :
```bash
scp services/build-embedded/Sensors/Sensors \
    services/build-embedded/ServiceManager/init \
    services/build-embedded/TelemetryService/TelemetryService \
    services/build-embedded/FSM/FSM \
    services/build-embedded/Motors/Motors \
    root@10.155.175.111:/root/
```

Sur la session SSH du robot :
```bash
chmod +x /root/*
/root/init
```

---

## 🛑 Arrêt d'urgence des services sur le robot
```bash
killall -2 init FSM Sensors Motors TelemetryService 2>/dev/null; sleep 0.5; killall -9 init FSM Sensors Motors TelemetryService 2>/dev/null
```
