# Guide Pratique de Calibration Physique — PiCar-S & Protection Bille (FastDrink)

Ce guide s'adresse aux étudiants et ingénieurs travaillant sur le véhicule physique **SunFounder PiCar-S**.
Le véhicule ne possédant **aucun encodeur de roues ni IMU**, la fidélité du modèle cinématique et la stabilité de la bille reposent sur des calibrations expérimentales rigoureuses.

---

## 1. Calibration des 5 Capteurs Optiques de Suivi de Ligne

Le module suiveur de ligne à 5 photodiodes est raccordé sur le bus I2C (`0x11` ou `0x12`).

### Procédure :
1. **Pose sur fond blanc** (sol de l'arène / piste blanche) :
   - Placer les 5 capteurs sur la surface blanche réfléchissante.
   - Relever les valeurs ADC brutes (typiquement ~800-950).
2. **Pose sur ruban noir** (ruban isolant 18 mm) :
   - Centrer le capteur 0, puis 1, 2, 3, 4 successivement sur le noir.
   - Relever les valeurs ADC brutes (typiquement ~100-250).
3. **Mise à jour des seuils** :
   - Le seuil de binarisation optimal se situe à la demi-somme :
     $$\text{Seuil}_i = \frac{\text{ADC}_{\text{blanc}, i} + \text{ADC}_{\text{noir}, i}}{2}$$
   - La noirceur continue $s_i \in [0.0, 1.0]$ est calculée par :
     $$s_i = \operatorname{clamp}\left(\frac{\text{ADC}_{\text{blanc}, i} - \text{ADC}_i}{\text{ADC}_{\text{blanc}, i} - \text{ADC}_{\text{noir}, i}}, 0.0, 1.0\right)$$
   - L'estimateur barycentrique continu calcule la position réelle de la ligne sans à-coups :
     $$e = \frac{\sum_{i=0}^{4} w_i s_i}{\sum_{i=0}^{4} s_i} \quad \text{avec } w = [-2, -1, 0, 1, 2]$$

---

## 2. Centrage du Servomoteur et Mesure des Rayons de Virage

L'angle de commande envoyé au servomoteur ne correspond pas exactement à l'angle réel des roues directrices en raison des jeux mécaniques de biellette.

### Procédure :
1. **Centrage du zéro mécanique (Offset)** :
   - Déposer le véhicule sur une ligne droite de 3 mètres.
   - Commander une propulsion à faible vitesse ($30\%$) avec un angle de $0^\circ$.
   - Si le véhicule dérive à droite, régler un offset négatif (`STEERING_OFFSET=-3.0` dans `motors.conf`).
   - Si le véhicule dérive à gauche, régler un offset positif (`STEERING_OFFSET=+2.5`).
2. **Mesure des rayons de courbure réels $R = f(\delta_{\text{servo}})$** :
   - Tracer un cercle au sol pour des angles de commande fixés : $+15^\circ$, $+25^\circ$, $+35^\circ$, $+45^\circ$.
   - Faire rouler le véhicule sur 2 tours complets et mesurer le diamètre au mètre ruban :
     $$R_{\text{mesuré}} = \frac{\text{Diamètre}}{2}$$
   - Comparer avec la formule d'Ackermann théorique :
     $$R_{\text{Ackermann}} = \frac{L}{\tan(\delta)} \quad (L = 0.14\text{ m})$$
   - Si un écart constant est constaté, ajuster `m_wheelbase_m` ou la table dans `VehicleModel`.

---

## 3. Étalonnage PWM $\leftrightarrow$ Vitesse Réelle ($m/s$)

Sans encodeur, la propulsion CC dépend de la tension de batterie, de la zone morte des ponts en H (TB6612FNG) et des frottements au sol.

### Procédure :
1. Baliser une distance de mesure $D = 2.0\text{ m}$ sur sol plat, précédée d'une zone d'élan de $0.5\text{ m}$.
2. Utiliser une batterie chargée à $\ge 7.4\text{ V}$.
3. Pour chaque palier de PWM (20%, 30%, 40%, 50%, 60%, 70%) :
   - Chronométrer le temps de passage $\Delta t$ sur les 2 mètres.
   - Calculer la vitesse réelle :
     $$v = \frac{2.0}{\Delta t} \quad (\text{m/s})$$
4. **Mise à jour de la table de calibration dans `VehicleModel`** :
   - Renseigner la table avec les points réels mesurés :
     ```cpp
     m_calibration_table = {
         {0.0f,  0.00f},
         {20.0f, v_20},
         {30.0f, v_30},
         {40.0f, v_40},
         {50.0f, v_50},
         ...
     };
     ```
   - La zone morte (`DEFAULT_DEADBAND_PWM`) correspond au pourcentage minimal pour lequel les roues commencent tout juste à tourner au sol.

---

## 4. Protocole de Stabilité de la Bille (FastDrink)

La coupelle ne disposant pas de rebord haut, la bille est soumise à l'accélération résultante :
$$a_{\text{résultante}} = \sqrt{a_x^2 + a_y^2} \le a_{\text{bille,max}}$$

### Essais progressifs :
1. **Accélération et freinage en ligne droite ($a_y = 0$)** :
   - Démarrer à l'arrêt et accélérer jusqu'à $v = 0.40\text{ m/s}$ avec $a_{\text{accel}} = 0.65\text{ m/s}^2$.
   - Freiner jusqu'à l'arrêt complet avec $a_{\text{brake}} = 0.70\text{ m/s}^2$.
   - Vérifier que la bille ne sort pas de son creux d'équilibre lors du freinage.
2. **Virage à rayon constant ($a_x = 0, a_y = \frac{v^2}{R}$)** :
   - Tourner sur un rayon de $R = 0.15\text{ m}$ à vitesse croissante :
     $$v_{\max} = \sqrt{a_{\text{bille,max}} \cdot R}$$
   - Pour $a_{\text{bille,max}} = 0.70\text{ m/s}^2$ et $R = 0.15\text{ m}$ : $v_{\max} \approx 0.32\text{ m/s}$.
3. **Virages en S et chocs directionnels (Jerk $\frac{da}{dt}$)** :
   - Si la bille oscille ou est éjectée lors du changement d'appui gauche/droite, réduire le gain $K_d$ ou augmenter le facteur de sécurité de virage (`CURVE_SAFETY_FACTOR = 0.80`).

---

## 5. Synthèse des Paramètres Configurables (`fsm.conf`)

| Clé de configuration | Rôle | Valeur recommandée |
|---|---|---|
| `STRAIGHT_CRUISE_SPEED` | Vitesse de croisière en ligne droite ($m/s$) | `0.35` à `0.45` |
| `MIN_CURVE_SPEED` | Vitesse minimale plancher dans virage serré ($m/s$) | `0.15` à `0.18` |
| `MAX_ACCELERATION` | Seuil maximal de la bille $a_{\text{bille,max}}$ ($m/s^2$) | `0.70` |
| `CURVE_SAFETY_FACTOR` | Facteur de marge sur la vitesse en virage | `0.85` |
| `LINE_KP` | Gain proportionnel de direction | `0.65` |
| `LINE_KD` | Gain dérivé (anti-rebond) | `0.05` |
| `OBSTACLE_STOP_DIST_CM` | Distance d'arrêt sécuritaire devant obstacle ($cm$) | `20.0` |
