# Service Manager

## Description
Ce système doit agir en temps que PID 1 sous le chemin
`/sbin/init`

## Actions

1. Le système mount `/dev` et les shared libs.

2. Le système prépare les interfaces wifi et bluetooth

3. Création de `fork` pour le bluetoothd, wpa_supplicant et dbus

4. Démarrage des services pour les moteurs, FSM, télémetrie, et capteurs.

5. Fork pour sur un programme qui analyse la connection wifi pour soit offrir la programmation par bluetooth.
Si le wifi est fonctionnel, le programme peut renvoyer un code de sortie 0.
Ce programme n'est pas monitoré par le Service Manager.

NB: Si un des services termine, le système lève un flag pour annoncer une erreur et tente de repartir le service pour éviter un service zombie.
un délai de 100ms est à prévoir entre redémarrage des services.

## Compilation (CMake)

### Compilation native (Hôte)
```bash
mkdir -p build && cd build
cmake ..
cmake --build .
```
Génère l'exécutable `init` (ou `init_debug` en mode Debug).

### Compilation embarquée statique (aarch64)
```bash
mkdir -p build && cd build
cmake ..
cmake --build . --target embedded
```
Ou avec le fichier toolchain CMake :
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=../toolchain-aarch64.cmake -DSTATIC_BUILD=ON
cmake --build build
```