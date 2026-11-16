# Audit de fidélité — simulation Webots du PiCar-S (16 novembre 2026)

Méthode : comparaison du PROTO avec le CAD (`webots/cad/*.dae`), les constantes du FSM et du HAL (`services/`) et la
simulation Godot, puis **mesure en boucle fermée dans Webots** avec la vraie loi de suivi de ligne du FSM
(`tools/run_closed_loop.py` + `tools/loop_bridge.cpp`) et la vérité terrain GPS/IMU.

## 1. Constats et corrections

| # | Élément | Avant | Constat | Après |
|---|---|---|---|---|
| 1 | **Capteurs IR : valeurs** | Table `[0→100, 0.07→100, 0.1→950]` | Webots divise la distance par `f = 0.2 + 0.8·rouge·(1−0.5·roughness)`. Avec le sol à roughness 1, blanc → 0,0525 m, noir → 0,1575 m : la rampe de la table (0,07–0,1 m) ne couvre qu'une bande de gris inexistante dans la texture → sortie **quasi binaire** (≈ 3 niveaux avec 5 rayons). | Table linéaire entre `h/0.6` et `h/0.2` (calculée dans le PROTO) : sortie = fraction de la tache couverte par le ruban, **flottant continu**. 16 rayons, bruit 1 %. |
| 2 | **Capteurs IR : position** | z = +0,01 m (dans le volume du châssis), pas de 15 mm | Les rayons infra-rouges de Webots détectent le robot lui-même ; la plaque avant du CAD (`Body002`, z 0–2 mm) est entre les capteurs et le sol. | 1 mm sous le châssis (h = 20,5 mm), pas de **19 mm** (Godot + commentaires du FSM), paramétrable (`irPitch`, `irSpotDiameter`, `irNoise`). |
| 3 | **Largeur du ruban** | 36,6 mm mesurés dans la texture (défaut CLI 0,035) | Le PRD, le guide de calibration et `line_follower.hpp` disent 18 mm. Avec 36 mm, 3 capteurs voyaient le ruban. | Texture régénérée à **18 mm** ; défaut CLI corrigé ; test qui mesure la texture. |
| 4 | **Direction** | Un seul pivot central qui tourne tout l'essieu avant | Ce n'était pas une direction d'Ackermann ; les roues avançaient/reculaient de 46 mm à 45°. | **Ackermann** : un pivot par roue (y = ±55 mm), angles calculés pour un même centre de rotation que le modèle bicyclette du FSM (R = L/tan δ). 45° bicyclette → 61,8° intérieure / 34,3° extérieure. |
| 5 | **Propulsion** | Essieu arrière rigide, 1 moteur, 1,5 N·m | Le vrai robot a 2 moteurs TT. Un essieu rigide ripe en virage (sous-virage). 1,5 N·m = 69 m/s² possibles. | 1 moteur par roue avec **différentiel** (vitesse selon le rayon de chaque roue), 0,1 N·m et 22 rad/s par moteur (TT 1:48). |
| 6 | **Masses** | 0,94 kg (châssis 0,65 + essieu avant 0,15 + …) | Trop lourd pour un PiCar-S + Pi + 2×18650. | **0,65 kg** : châssis 0,53 (CdM à x = 0,045, vers les batteries/moteurs), roues 25–30 g, pivots 8 g. **À peser.** |
| 7 | **Géométrie** | L = 0,14, voie 0,13, R roue 33,5 mm | Cohérent avec le CAD (roues de 67 mm, châssis 234 × 110 mm) et `WHEELBASE_M` du FSM. Lookahead capteurs = 0,17 m devant l'essieu arrière, identique à Godot. | Inchangé, maintenant vérifié par test. |
| 8 | **Physique** | Pas de 20 ms, `ContactProperties` par défaut | Webots par défaut : **bounce 0,5** (roues qui rebondissent) ; 20 ms est grossier pour des roues de 25 g. | Pas physique 4 ms (contrôle maintenu à 20 ms), pneu : frottement 0,9, bounce 0. |
| 9 | **Limiteur de braquage** | 250 °/s fixe | Le vrai `MotorActuator` limite à **175 °/s sous 15° et 300 °/s au-delà**. | Reproduit exactement, puis servo à 600 °/s, jeu et rapport servo→roue `steer_ratio`. |
| 10 | **Latence** | 60 ms ajoutés (non mesurés) | La chaîne simulée a déjà 20–40 ms (pas Webots + tick FSM asynchrone) : la latence était comptée deux fois. | 35 ms ajoutés (Motors 100 Hz + trame PWM 50 Hz + temps mort du servo). **À mesurer.** |
| 11 | **Sonar** | 1 rayon, cône par défaut de 90° | HC-SR04 : 2 cm à 4 m, cône d'environ 30°. | 5 rayons, ouverture 0,52 rad, portée 2 cm à 4 m. |

Normalisation : le contrôleur (`normalize_ir`) fait la même interpolation linéaire bornée que le vrai service Sensors
(`adc_to_darkness` : blanc → 0, noir → 1). C'est correct ; c'était la table Webots en amont qui rendait les valeurs binaires.

## 2. Cause du wobble 0° / 45°

Mesures dans Webots (≈ 50–90 s de course, braquage commandé par le vrai `LineFollowerController`) :

| Configuration | Temps en butée ±45° | \|δ\| moyen | Écart latéral RMS |
|---|---|---|---|
| Avant l'audit | 62 % | 37° | 44 mm |
| Après corrections (réglages par défaut) | 49 % | 33° | 34 mm |
| Cinématique idéale (sans modèle dynamique) | **0 %** | 10° | **8 mm** |
| Latence ajoutée = 0 | 5 % | 19° | 17 mm |
| Limiteur/servo instantanés | 0,2 % | 15° | 12 mm |
| Pas des capteurs 15 mm | 60 % | 36° | 33 mm |
| Pas des capteurs 17 mm | 32 % | 26° | 25 mm |
| `steer_ratio` = 0,7 | 9 % | 19° | 17 mm |
| Correctif FSM émulé (voir ci-dessous) | 22 % | 23° | 23 mm |
| Correctif FSM + `steer_ratio` = 0,7 | **0,1 %** | 15° | **11 mm** |

**Cause n°1 : fausses pertes de ligne (FSM).** Le ruban de 18 mm est plus étroit que le pas de 19 mm. Quand il est centré
entre deux capteurs, chacun lit ~0,45 (< 0,5) **quel que soit le modèle de capteur symétrique**.
`LineFollowerController::update_analog` exige `max ≥ 0,5` pour « ligne vue » : il déclare la ligne perdue et applique
`hold_steer ≥ 0,85` (38°) du dernier côté. C'est arrivé dans **15 % des trames même en cinématique idéale**. Ces à-coups de
±38°, filtrés par le limiteur de `MotorActuator` et la latence, entretiennent l'oscillation.
**Correctif appliqué** à la demande explicite de l'utilisateur (exception à la règle d'AGENTS.md sur `services/libs/shared-headers/`) :
`has_line = (dark_count >= 1 || max_d >= 0.50f || total >= ANALOG_GAP_TOTAL_DARKNESS)` avec `ANALOG_GAP_TOTAL_DARKNESS = 0.50f`,
et un test dans `services/tests/test_control.cpp` (cas 5b : 0,45 + 0,45 → ligne vue, braquage modéré).

**Cause n°2 : gain de boucle × retard.** La stabilité dépend de quatre paramètres réels que personne n'a mesurés : pas des
capteurs, taille de la tache, rapport servo → roue et latence. Avec des valeurs plausibles (19 mm, 10 mm, 1,0, 35 ms), la boucle
oscille encore. Puisque le vrai robot ne wobble pas, au moins un de ces paramètres est différent dans la réalité. Le plus
probable était que les roues braquent moins que le servo, mais le manuel SunFounder indique qu'elles atteignent bien 45° (voir section 3). Il reste le pas des sondes, la latence réelle et la forme de la réponse du TCRT5000.

## 3. Données trouvées en ligne (2e passe) et intégrées

| Donnée | Source | Valeur retenue dans la simulation |
|---|---|---|
| Hauteur des sondes TCRT5000 | Manuel PiCar-S p. 45 : surface blanche « about 1cm away » | Capteurs à **10 mm** du sol (avant : 20,5 mm) ; tache estimée à 7 mm |
| Pas des sondes | Manuel p. 46 : ruban de 18–30 mm, « nearly the distance between two probes » ; module de 125 mm (wiki : PCB 124 × 30 mm) | **19 mm** conservé (≈ ruban de 18 mm). Aucune source ne donne la cote exacte |
| Empattement et braquage | Manuel p. 46 : à 45° de braquage des roues, rayon = empattement, minimum 138 mm | L = 0,14 m confirmé à 2 mm ; les roues atteignent bien 45° → `steer_ratio` = 1 (l'hypothèse « 0,7 » n'est pas soutenue) |
| Pivots de direction | Trous du CAD (`Chassis.dae`) à y ≈ ±50 mm | Pivots à ±50 mm, déport de roue 15 mm (avant : ±55 / 10) |
| Servo SF0180 | Manuel p. 62 : 0,12 s/60° à 4,8 V, 0,09 s à 6 V ; 2,0 kg·cm | 500 °/s, 9,1 rad/s, 0,2 N·m |
| Moteur | Manuel p. 62 : F130SA, 1:48, ≈ 200 tr/min à vide (180 mesurés) | 19 rad/s max ≈ 0,63 m/s (cohérent avec 0,32 m/s à 50 %) |
| Ultrason | Manuel p. 61 : module 25 kHz, cône d'environ 30°, peu précis | Cône 0,52 rad (déjà en place) |
| Masse | Annonces : 499 g / 1,1 lb (kit) ; + Raspberry Pi 3 (45 g) + 2 × 18650 (~90 g) | **0,61 kg** (avant : 0,65) |

Résultat en boucle fermée avec ces valeurs : temps en butée **36 %** (49 % avant cette passe, 62 % avant l'audit), écart RMS
28 mm, fausses pertes de ligne 7,2 % des trames. Avec le correctif FSM émulé : 24 %, RMS 23 mm. Aucune trame ne montre de
roue vue par les capteurs IR.

### Après le correctif FSM (course de 87 s, valeurs sourcées)

| | Temps en butée ±45° | Écart RMS | Écart max | À-coups > 15° par pas | Vraie perte de ligne |
|---|---|---|---|---|---|
| Au départ de l'audit | 62 % | 44 mm | 92 mm | — | — |
| Simulation corrigée, FSM d'origine | 36 % | 28 mm | 67 mm | 2,6 / s | — |
| **Simulation corrigée + correctif FSM** | **13,6 %** | **19,6 mm** | 48 mm | **0** | 2,1 % des trames |

Il reste une oscillation lente (~1 Hz) dans les virages, sans à-coups : elle dépend des paramètres encore inconnus
(pas des sondes, latence réelle).

## 4. À mesurer sur le vrai robot (par ordre d'impact)

1. **Pas des capteurs** (règle, 10 s) → `irPitch` dans `worlds/picar_s_track.wbt`. C'est la seule cote introuvable en ligne.
2. **Rapport servo → roue** : le manuel indique ~1 ; à confirmer en mesurant le cercle à 45° (2 tours).
3. **Latence et vitesse du servo** : échelon −45° → +45° filmé à 60 i/s (`tools/calibrate_dynamics.py --steer`).
4. **Réponse d'un capteur** : faire glisser le ruban sous un capteur par pas de 2 mm, relever la noirceur → `irSpotDiameter`.
5. **Masse totale et répartition avant/arrière** (2 balances de cuisine).

Pour comparer ensuite avec le réel : `python3 webots/tools/run_closed_loop.py --duration 60`.

## 5. Fichiers modifiés

`protos/PiCarS.proto`, `worlds/picar_s_track.wbt`, `worlds/textures/line_track.png`, `controllers/picar_s_controller/`
(`picar_protocol.py`, `picar_s_controller.py`, `picar_dynamics.py`), `tools/generate_track_texture.py`, nouveaux
`tools/loop_bridge.cpp` et `tools/run_closed_loop.py`, tests (`test_proto_syntax`, `test_world_track`, `test_protocol`,
`test_dynamics`), `README.md`. Aucun service C++ modifié.

## 6. Balayage de robustesse des gains (sans mesure)

Outil : `tools/sweep_robustness.py`, environ 700 courses Webots. Chaque couple (KP, KD) roule dans chaque combinaison plausible
des paramètres inconnus : pas des sondes 17–25 mm, latence ajoutée 20–80 ms, tache 5–12 mm. On garde le **pire cas**.
Les combinaisons à angle mort (pas > ruban + tache, ex. 25 mm / 5 mm : 30 % des trames sans ruban visible) sont exclues,
parce que le vrai robot suit la ligne.

**Constats**
- Augmenter KD aggrave toujours l'oscillation : le terme dérivé amplifie le retard et le bruit.
- Avec un pas ≥ 22 mm, l'écart reste grand (RMS 25–45 mm) quels que soient les gains : c'est une limite des capteurs
  (ruban de 18 mm à peine couvert), pas du réglage.
- Limite géométrique : sondes à 0,17 m de l'essieu arrière et braquage max 45° → le plus petit cercle suivable **au niveau des
  sondes** est R = √(0,14² + 0,17²) ≈ 0,22 m. Une piste à R = 150 mm (testée) est impossible ; l'exigence « R ≥ 120 mm » du PRD
  ne tient pas avec cette position des sondes.

**Pire cas, pas plausible (17–19 mm), toutes latences et taches**

| Gains | Piste R = 0,5 m : butée / écart max | Piste R = 0,25 m : butée / écart max | Pertes de ligne |
|---|---|---|---|
| KP 0,58 / KD 0,035 (actuel) | 60 % / ~100 mm | 69 % / 116 mm | 0 |
| KP 0,45 / KD 0,02 | 1 % / 93 mm | 0 % / 82 mm | 0 |
| KP 0,40 / KD 0,02 | 0 % / 76 mm | 0 % / 57 mm | 0 |
| **KP 0,35 / KD 0,035** | **0 % / 39 mm** | **0 % / 50 mm** | **0** |

**Lecture pour le vrai robot.** Les gains actuels (0,58) ne posent problème en simulation que si la latence réelle dépasse
~50 ms. Comme le vrai robot ne wobble pas avec 0,58, sa latence est probablement faible : c'est une indication indirecte,
pas une mesure. KP 0,35 / KD 0,035 est le réglage qui ne dépend plus des inconnues. Il n'est **pas** appliqué à `fsm.conf`
(qui pilote aussi le vrai robot) et devra être vérifié sur le robot avant adoption.

**Appliqué puis retiré** : `services/fsm.sim.conf` a porté LINE_KP=0.35 / LINE_KD=0.035 pour la simulation ; ces gains ne
passent pas le virage de R = 0,15 m du circuit de Godot (section 7). Décision : la simulation reprend les gains du robot
(0,58 / 0,035) ; `fsm.sim.conf` reste disponible, sans surcharge active. `services/run_sim.sh` copie
`fsm.conf` dans `/tmp/picar-sim-conf/`, y remplace ces clés et lance le FSM depuis ce dossier (le FSM lit `./fsm.conf` en
premier) : le vrai `fsm.conf` n'est jamais lu en écriture. Course Webots par défaut avec ces gains : **0 % en butée**, écart RMS
14,6 mm (max 24 mm) sur R = 0,5 m ; 0 %, RMS 17 mm (max 34 mm) sur R = 0,25 m.

**Problème corrigé au passage** : sous Docker (`./build.sh --sim`), seul `services/` était monté ; le FSM ne trouvait pas
`fsm.conf` (à la racine du dépôt) et tournait avec les valeurs C++ par défaut (KP 0,65, KD 0,05…). `docker-compose.yml` monte
maintenant `../fsm.conf` en lecture seule sur `/fsm.conf`.

## 7. Bille FastDrink, support et circuit de Godot

**Sources** (lues avec Blender, `tools/export_godot_assets.py`) : `godot/assets/plaquette_blender_scale.blend`,
`godot/assets/parcours.blend`, `godot/main.tscn`.

| Élément | Valeur | Dans Webots |
|---|---|---|
| Bille | verre, Ø 20 mm, 10 g (main.tscn) | Solid `DEF BALL` du monde, physique réelle |
| Support | plaque PLA 50 × 50 × 4 mm, ~9 g | `DEF CUP_PLATE` du PROTO, fixé au châssis, collision = maillage `cad/Plaquette.obj` |
| Position du support | 48 mm devant l'essieu arrière, 95 mm du sol (main.tscn) | idem |
| Coupelle | **bol** : fond 2,5 mm, rebord 4,0 mm à r = 20 mm, profil ≈ sphère de 133 mm | maillage d'origine |
| Contact verre/PLA | frottement 0,4, rebond 0,1, `rollingFriction 1e-4` | à mesurer (voir plus bas) |
| Circuit | trace Godot exact (demi-tours, virages à 90°, 2 obstacles, T d'arrivée), ruban ramené de 20 à **18 mm** | `worlds/picar_s_course.wbt`, `textures/godot_course.png` |

La coupelle n'est **pas un cône** (hypothèse de `ball_monitor.gd` et de `picar_dynamics.BallModel`). Le seuil statique au
rebord vaut g·tan(φ) ≈ 1,48 m/s², mais la bille oscille : sur un échelon d'accélération elle dépasse sa position d'équilibre.

**Banc d'essai** (`tools/ball_lab.py`, Webots) :

| Mesure | Webots | Théorie (bol de 133 mm) |
|---|---|---|
| Dérive au repos | 0,001 mm | 0 |
| Période d'oscillation (amplitude 6–7 mm) | 0,79–0,81 s | 0,83 s |
| Amortissement ζ (`rollingFriction 1e-4`) | 0,23 | Godot supposait 6/s ≈ 0,28 |
| Échelon 0,65 m/s² (rampe du FSM) | reste, 13,6 mm du centre | — |
| Échelon 0,80 m/s² | reste, 17,5 mm (limite) | — |
| Échelon 1,0 m/s² | **sort** | seuil ≈ 1,48 / (1 + dépassement) ≈ 0,9–1,2 |

Près du centre (< 2 mm), le maillage est un éventail de facettes : la bille y oscille plus vite (0,2–0,4 s). Sans effet sur
la chute, qui se joue près du rebord.

**Courses sur le circuit avec la vraie pile** (Docker : SimBridge + FSM, `run_closed_loop.py --course --external`) :

| Gains | Résultat |
|---|---|
| KP 0,35 (essai) | suit 7,9 m sans butée, **perd la ligne au virage à 90° de R ≈ 0,15 m** en haut à droite ; bille tombée à l'arrêt |
| **KP 0,58 (robot, retenu pour la simulation)** | 9,1–9,2 m, passe tous les virages, s'est arrêté une fois sur le T ; **bille tombée à ~8,4 s** (oscillation du braquage en sortie de demi-tour, en résonance avec la bille) |

Causes des chutes, toutes physiques : (1) le braquage qui oscille (période 0,5–0,8 s) excite la bille près de sa
fréquence propre ; (2) à l'arrêt, la **zone morte moteur** (consigne < 0,128 m/s → moteur coupé, valeur provisoire) donne une
décélération finale ~1 m/s² au lieu des 0,65 m/s² de la rampe ; (3) toute collision (obstacle).

**Limites géométriques** : sondes 0,17 m devant l'essieu arrière et braquage max 45° → le plus petit virage suivable
en régime établi est R ≈ 0,22 m ; le circuit a des virages de 0,15–0,17 m (PRD : R ≥ 120 mm).

**À mesurer en priorité pour que la bille soit fidèle** : amortissement de la bille (filmer la bille 2 s après une
tape sur la voiture : période et décroissance), zone morte et décélération moteur à l'arrêt (filmer un arrêt), constante de
temps moteur (échelon).

