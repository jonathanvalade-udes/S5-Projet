# Scripts Godot : à quoi sert chaque fichier

Ce dossier contient le code GDScript de la simulation. Godot simule un faux PiCar sur un faux circuit ; le vrai cerveau
(le FSM, en C++) le pilote à distance. Godot ne décide rien : il envoie ce que les capteurs « voient » et applique les
ordres reçus.

## Le trajet d'une image

```
FSM (C++) ──► SimBridge ──► network_client.gd ──► picar_raycast.gd ──► la voiture bouge
   ▲                         (TCP, port 9000)            │
   │                                                     ├─ ultrason (distance)
   └──── SimBridge ◄──── network_client.gd ◄─────────────┼─ line_follower_raycast.gd (5 capteurs de ligne)
                          (50 trames par seconde)        └─ ball_monitor.gd (la bille est-elle tombée ?)
```

1. `network_client.gd` reçoit du FSM la vitesse voulue et le braquage voulu.
2. `picar_raycast.gd` déplace la voiture selon ces ordres (avec ou sans les défauts du mode réaliste).
3. Les capteurs lisent le monde : l'ultrason (dans `picar_raycast.gd`), les 5 capteurs de ligne
   (`line_follower_raycast.gd`) et la bille (`ball_monitor.gd`).
4. `network_client.gd` renvoie toutes ces mesures au FSM, 50 fois par seconde.

## Les fichiers

| Fichier | Rôle | Rattaché à |
| --- | --- | --- |
| `main.gd` | Prépare le monde, gère les boutons, lit les options de lancement | nœud `Main` |
| `network_client.gd` | Lien TCP avec SimBridge | nœud `NetworkClient` |
| `picar_raycast.gd` | Mouvement, collisions, ultrason | nœud `PiCar` |
| `wheel_animator.gd` | Animation des roues (affichage seulement) | créé par `picar_raycast.gd` |
| `line_follower_raycast.gd` | Les 5 capteurs de ligne | nœud `sensor_array` (dans `line_follower_sensor.tscn`) |
| `ball_monitor.gd` | La bille dans la coupelle : roule-t-elle ou tombe-t-elle ? | nœud `Bille` |
| `realism.gd` | Les défauts du vrai robot (mode réaliste) | classe statique, rien à rattacher |
| `optuna_runner.gd` | Évaluation automatique d'un tour pour l'optimiseur | créé par `main.gd` si `--optuna` |

### `main.gd`
- `_ready` : donne la couche de collision 2 au ruban (les capteurs de ligne ne voient que cette couche), crée une
  collision pour les cubes `obstacle*` (couche 1), lit les options de la ligne de commande.
- `_on_connect_pressed`, `_on_start_fsm_pressed`, `_on_stop_fsm_pressed`, `_on_quit_pressed` : les quatre boutons.
  Le champ d'adresse accepte `127.0.0.1` ou `127.0.0.1:9000`.
- `_on_network_*` : affichent l'état de la connexion.
- `_physics_process` : ne fait quelque chose qu'en mode Optuna.

### `network_client.gd`
- `connect_to_server`, `disconnect_from_server` : ouvrent et ferment la connexion.
- `send_trigger_start`, `send_emergency_stop` : le départ et l'arrêt d'urgence (texte brut, une ligne).
- `_process` : lit les ordres du FSM (`_read_commands`) et envoie l'état toutes les 20 ms (`_send_state_periodically`).
- `data_to_send` : l'état envoyé (`velocity`, `distance`, `line_follower`, `line_analog`, `ball_dropped`).
  Les cinq clés sont lues par SimBridge (`services/libs/shared-headers/utils/simple_json.hpp`) : ne pas en retirer.
- `data_received` : les ordres reçus. Seuls `velocity` (m/s) et `direction` (de -1 à +1) sont utilisés.

### `picar_raycast.gd`
- `_process` : lit les ordres, les passe dans `_apply_latency` puis `_update_actual_motion` (vitesse et braquage RÉELS),
  teste la collision (`_would_collide`), avance si la voie est libre, puis anime les roues.
- Modèle d'Ackermann : seules les roues avant tournent, comme un vélo.
  `vitesse de rotation = (vitesse / empattement) × tan(braquage)`, avec un empattement de 0,14 m.
- `_apply_kick` : petit choc aléatoire (mode réaliste seulement).
- `_build_collision_box`, `_would_collide` : la voiture n'est pas un corps physique, on teste à la main si la boîte qui
  l'entoure entrerait dans un obstacle, et si oui le pas est annulé.
- `_physics_process` : le capteur ultrason. Un rayon de 0,8 m vers l'avant ; la distance est envoyée en centimètres
  (100 cm si le rayon ne touche rien).
- Valeurs lues par d'autres scripts : `actual_speed`, `actual_steer_rad`, `actual_yaw_rate` (la bille).

### `wheel_animator.gd`
- Fait rouler les 4 roues et braquer les 2 roues avant. Aucun effet sur le mouvement : c'est de l'affichage.

### `line_follower_raycast.gd`
- 5 capteurs, chacun couvre 19 mm. Chaque capteur lance 15 rayons verticaux vers le sol ; la fraction qui touche le
  ruban donne sa noirceur, de 0 (blanc) à 1 (noir).
- Envoie `line_analog` (les 5 noirceurs, lues par le FSM) et `line_follower` (les 5 bits, noirceur > 0,5, gardés
  pour le protocole).
- Les positions des capteurs viennent des 5 `Marker3D` de `line_follower_sensor.tscn`.

### `ball_monitor.gd`
- La bille ne bouge pas par le moteur physique de Godot (le nœud est gelé) : le script calcule son mouvement.
  Quand la voiture accélère, la bille subit la force d'inertie opposée ; la pente de la coupelle la ramène au centre.
- Elle tombe quand elle s'éloigne de plus de 20 mm du centre, c'est-à-dire quand l'accélération soutenue dépasse
  `g × tan θ = 0,735 m/s²` (coupelle de 1,5 mm de profondeur et 20 mm de rayon).
- `_integrate` : un pas du mouvement. `_drop`, `_fall` : la chute. `max_accel_observed` et `is_dropped` sont lus par
  `optuna_runner.gd`.

### `realism.gd`
- Désactivé par défaut. Touche **R** : active ou désactive les défauts. Réglages dans le dictionnaire `p`.
- `noise_scale` multiplie les bruits aléatoires (0 = aucun). Les défauts fixes (biais, jeu, latence, gain moteur) restent.
- Le hasard utilise une graine fixe : deux lancements donnent le même parcours.

### `optuna_runner.gd`
- Lancé par `python tools/optimizer/optimize_godot.py --mode godot`. Envoie le départ, mesure le tour, imprime
  `OPTUNA_RESULT:{...}` puis quitte (code 0 = succès, 1 = échec ou délai dépassé).

## Options de lancement

| Option | Effet |
| --- | --- |
| Touche `R` | Active ou désactive le mode réaliste |
| `REALISM=1` | Démarre en mode réaliste |
| `REALISM_SEED=<n>` | Change la graine du hasard (12345 par défaut) |
| `REALISM_<NOM>=<valeur>` | Change un réglage, par exemple `REALISM_STEER_BIAS_DEG=6` |
| `-- --time-scale <N>` | Accélère la simulation N fois |
| `-- --optuna --timeout <s>` | Mode d'évaluation automatique |

## Pièges à connaître

- **Godot est en miroir du robot réel.** SimBridge inverse le braquage et l'ordre des 5 capteurs. Si le robot corrige
  dans le mauvais sens, cherchez là (`services/SimBridge/src/SimShmBridge.cpp`).
- **Le FSM ne lit que `line_analog`.** Les bits `line_follower` n'ont aucun effet sur lui. Le réglage
  `line_flip_prob` du mode réaliste ne touche que ces bits : il est donc sans effet sur le FSM.
- **L'avant du PiCar est l'axe -X.** Avancer = translation en -X ; une accélération positive pousse la bille vers +X.
- **L'amortissement de la bille (6) est une hypothèse**, pas une mesure. Un petit dépassement bref de l'accélération
  ne fait donc pas tomber la bille.
- **Pas de physique de Godot pour la bille.** Essayée puis abandonnée : la coupelle fait 1,5 mm, bien en dessous de la
  précision du moteur physique.
