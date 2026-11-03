# SimBridge

Pont TCP entre la simulation Godot et la pile C++ (via la SHM `/picar_shm`).
Remplace le serveur Python `robot/main.py` (WebSocket 8765).

## Lancer la simulation (Windows + WSL2)

```powershell
.\Services\run_sim.ps1              # SimBridge + FSM
.\Services\run_sim.ps1 -Telemetry   # + TelemetryService (--no-ui)
.\Services\run_sim.ps1 -Rebuild     # force la recompilation
```

Puis dans Godot : cocher **Localhost** (ou `127.0.0.1:9000`) et **Connect**.
Sous Linux : `Services/run_sim.sh` (variables `PORT`, `TELEMETRY=1`, `REBUILD=1`, `BUILD_DIR`).

Prérequis WSL : `g++`, `make`, `cmake` (le script utilise `~/.picar-venv/bin` s'il existe).

> **Sensors et Motors ne doivent pas tourner en simulation** : sous Linux ils choisissent
> les drivers PiCar et écraseraient les capteurs fournis par Godot dans la SHM.
> SimBridge doit démarrer **avant** le FSM (il crée la SHM et la supprime en quittant).
> Après un arrêt brutal, supprimer `/dev/shm/picar_shm` (le script le fait au démarrage).

## Protocole (JSON, une ligne `\n` par message, 50 Hz)

Godot → bridge :
```json
{"velocity":0.0,"line_follower":[0,0,1,0,0],"ball_dropped":false,"distance":100.0}
```
`distance` en cm, `line_follower` = 5 capteurs [G2,G1,C,D1,D2] (1 = ruban noir).

Champ optionnel `line_analog` : noirceur continue des 5 capteurs, `0.0` = blanc … `1.0` = noir
(Godot lance 7 rayons par capteur et envoie la fraction qui touche le ruban) :
```json
{"...":"...","line_follower":[0,0,1,1,0],"line_analog":[0.0,0.0,0.62,0.38,0.0]}
```
Si `line_analog` est absent ou incomplet (≠ 5 valeurs), le pont retombe sur `line_follower` (0.0 / 1.0).
Le FSM calcule alors l'erreur de ligne comme centre de gravité de la noirceur (braquage continu).

Bridge → Godot (une réponse par trame) :
```json
{"velocity":0.3200,"direction":0.0000,"brake":false}
```
`velocity` en m/s (`target_speed/50*0.32`), `direction` normalisé [-1,1]
(Godot applique ±48°). `brake` est informatif, Godot l'ignore.

`ball_dropped` → le FSM passe en `Failure` (état terminal, arrêt). Relancer `run_sim` pour repartir.

## Limites connues
- Un seul client à la fois.
- Parseur JSON minimal (`libs/shared-headers/utils/simple_json.hpp`), accepte toute ligne non vide.
- Le FSM démarre le suivi de ligne dès son lancement, avant la connexion de Godot.
