# Services C++ PiCar-S

Ce répertoire regroupe les microservices embarqués et la suite de tests pour le robot autonome SunFounder PiCar-S (architecture ARM64 / Linux).

---

## 🏗️ Architecture et Microservices

| Service | Rôle | Pilotes matériels / HAL |
| :--- | :--- | :--- |
| **`ServiceManager`** | Processus initial (PID 1) de démarrage et supervision des démons | Monte `/dev`, `/proc`, `/sys`, `/dev/shm` |
| **`Motors`** | Gestion de la propulsion et de la direction | TB6612 (GPIO 17/27) & PCA9685 PWM (I2C 0x40) |
| **`Sensors`** | Suiveur de ligne 5 voies et télémétrie ultrason | TCRT5000 (I2C 0x11/0x0B) & HC-SR04 (GPIO 12/16) |
| **`FSM`** | Machine à états finis de navigation autonome | Logique de commande et suivi de ligne |
| **`TelemetryService`** | Diffusion de l'état du robot et surveillance | Interface UDP / Websocket / SHM |
| **`SimBridge`** | Pont de communication pour la simulation 3D Godot | Socket TCP <-> `/dev/shm/picar_shm` |

---

## 🐳 Compilation et Tests via Docker (Mac & Windows)

Toutes les classes matérielles sont natives Linux. Sur Mac ou Windows, le développement et la validation s'effectuent via le conteneur Docker `alpine-picar-builder`.

### 1. Compiler les services
```bash
# Compilation complète pour ARM64 (statique)
./build.sh --embedded --static

# Compiler un seul service (ex: Motors)
./build.sh --embedded --static Motors
```

### 2. Exécuter la suite de tests (`ctest`)
```bash
./build.sh --test
```
Ou manuellement avec Docker Compose :
```bash
docker compose -f docker-compose.yml run builder sh -c "cmake -B build-embedded -S . -DSTATIC=ON && cmake --build build-embedded && ctest --test-dir build-embedded --output-on-failure"
```

### 3. Lancer la simulation Godot via Docker
```bash
# Lance SimBridge et FSM dans Docker avec le port 9000 exposé
./build.sh --sim

# Ou directement avec Docker Compose :
docker compose -f docker-compose.yml up simulation
```
*Puis dans Godot sur votre machine hôte (Mac ou Windows) : cocher **Localhost** (`127.0.0.1:9000`) et cliquer sur **Connect**.*

---

### Détail des tests exécutés par `ctest` :
1. **`test_filters`** : Filtres médian et EMA face aux glitchs de mesure.
2. **`test_ipc`** : Concurrence mémoire partagée (`POSIX SHM`), Watchdog et arrêt d'urgence Failsafe.
3. **`test_control`** : Calculs de correction de braquage et trajectoire.
4. **`test_sim_bridge`** : Encodage et décodage JSON pour Godot.
5. **`test_i2c`** : Parsing ADC et registres matériels.
6. **`test_telemetry`** : Synchronisation des paquets télémétriques.
7. **`test_service_manager`** : Résolution des binaires et gestion des processus.
