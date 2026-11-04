# Spécifications Fonctionnelles et Techniques (PRD) — Projet FastDrink S5-GI

> **Document Type** : Product Requirements Document (PRD) — Volet Technique  
> **Système cible** : Véhicule autonome SunFounder PiCar-S (Raspberry Pi) & Environnement de simulation Godot Engine  
> **Source documentaire** : `GuideEtudiant.pdf`, `CaracteristiqueProjet.pdf`, `CalculAcceleration.pdf`, `SchemaCommunication.png`  
> **Objectif** : Spécifier l'ensemble des exigences, de l'architecture logicielle, de la dynamique physique, de la simulation et des protocoles de tests réels nécessaires à la réalisation complète du système sans préjuger de l'implémentation existante.

---

## 1. Contexte & Objectifs Techniques

Le projet consiste à concevoir un véhicule autonome capable d'assurer le service de breuvages sur circuit balisé tout en garantissant l'intégrité de son chargement (représenté par une bille logée dans une coupelle).

Le système se compose de deux piliers interopérables :
1. **Un jumeau numérique sous Godot Engine** reproduisant la dynamique du robot, les capteurs et la physique de la bille.
2. **Un robot physique SunFounder PiCar-S** embarquant un Raspberry Pi, asservi sur ses capteurs embarqués (ultrasons, suiveur de ligne optique, suiveur de lumière).

> **Règle d'or d'ingénierie (Spécification S10)** :  
> Le code de décision, la machine à états et les algorithmes de régulation doivent être **strictement identiques** et partagés entre la simulation Godot et le robot physique. Seule la couche d'acquisition/actionnement (Hardware Abstraction Layer) varie.

---

## 2. Spécifications & Contraintes Physiques et Fonctionnelles

### 2.1 Contraintes Cinématiques et Dynamiques
* **Seuil d'accélération maximale admissible ($a_{\max}$)** :
  $$\mathbf{a \le 0{,}735\text{ m/s}^2}$$
  *Calcul validé par conservation de l'énergie mécanique ($m \cdot a \cdot d = m \cdot g \cdot \Delta h$ avec $\Delta h = 1{,}50\text{ mm}$ et $d = 20\text{ mm}$).*
  Toute consigne d'accélération ou de freinage linéaire dépassant cette valeur provoque l'éjection de la bille et l'échec de la mission.
* **Vitesse maximale en virage ($v_{\text{virage}}$)** :
  L'accélération centrifuge ne doit pas dépasser le seuil critique :
  $$a_{\text{centrifuge}} = \frac{v^2}{R} \le 0{,}735\text{ m/s}^2 \implies v_{\text{virage}} \le \sqrt{0{,}735 \cdot R_{\min}}$$
  Pour le rayon minimal autorisé $R_{\min} = 120\text{ mm} = 0{,}12\text{ m}$ :
  $$v_{\max,\text{courbe}} \le \sqrt{0{,}735 \cdot 0{,}12} \approx 0{,}297\text{ m/s}\quad (29{,}7\text{ cm/s})$$

### 2.2 Exigences de Navigation & Déplacement (S1 à S6)
| Réf. | Intitulé | Exigence quantitative / qualitative |
| :--- | :--- | :--- |
| **S1** | **Arrêt de précision** | Arrêt devant un obstacle cible avec une tolérance stricte de **$\pm 30\text{ mm}$**. |
| **S2** | **Suivi de ligne droite** | Maintien et guidage sur ruban isolant noir de largeur **$18\text{ mm}$**. |
| **S3** | **Suivi de ligne courbe** | Négociation fluide de virages jusqu'à un rayon minimal de **$120\text{ mm}$** sans décrochage. |
| **S4** | **Évitement d'obstacle** | Détection d'un obstacle fixe, sortie contrôlée de trajectoire, contournement sécuritaire et ré-acquisition autonome de la ligne-guide. |
| **S5** | **Marche arrière guidée** | Recul autonome le long du ruban noir sur une distance **$< 200\text{ mm}$**. |
| **S6** | **Maintien du chargement** | Maintien continu de la bille dans la coupelle pour l'ensemble des manœuvres. |

### 2.3 Exigences Matérielles & Logicielles (S7 à S10)
| Réf. | Intitulé | Exigence quantitative / qualitative |
| :--- | :--- | :--- |
| **S7** | **Capteurs exclusifs** | Perception limitée strictement aux 3 capteurs : capteur à ultrasons HC-SR04, module suiveur de ligne optique (3 canaux TCRT5000), suiveur de lumière (photorésistances). |
| **S8** | **Zéro Hardcoding** | Navigation 100 % réactive et dynamique basée sur les lectures capteurs en continu. Aucune coordonnée ou géométrie de circuit pré-enregistrée. |
| **S9** | **Modélisation préalable** | Simulation obligatoire sous Godot & Blender validant le comportement dynamique avant tout essai sur piste. |
| **S10**| **Symétrie algorithmique** | Mêmes algorithmes de décision, régulation et filtrage sur le robot physique et dans la simulation. |

---

## 3. Architecture Système & Flux de Données

### 3.1 Découpage Architectural (HAL - Hardware Abstraction Layer)
Afin de respecter la spécification **S10**, l'architecture logicielle doit séparer rigoureusement la logique décisionnelle du matériel :

```
                  ┌────────────────────────────────────────┐
                  │    Core Autonome (Module Partagé)      │
                  │   - Machine à États Finis (FSM)        │
                  │   - Asservissement / PID Direction     │
                  │   - Générateur de profil de vitesse    │
                  │     (Rampes bornées à 0.735 m/s²)      │
                  │   - Routines : Suivi, Évitement, Recul │
                  └───────────────────▲────────────────────┘
                                      │
              ┌───────────────────────┴───────────────────────┐
              │                                               │
┌─────────────▼───────────────┐               ┌───────────────▼─────────────┐
│    Driver Simulation Godot  │               │   Driver Matériel Robot     │
│ - Client / Serveur WebSocket│               │ - GPIO / I2C (RPi)          │
│ - Dé-sérialisation JSON     │               │ - HC-SR04 / TCRT5000 / ADC  │
│ - Envoi consignes virtuelles│               │ - Contrôleur PWM Moteurs    │
└─────────────────────────────┘               └─────────────────────────────┘
```

### 3.2 Protocole de Communication WebSocket (JSON)
* **Canal** : WebSocket bidirectionnel sur TCP/IP (`localhost` ou réseau local Wi-Fi).
* **Fréquence recommandée** : $20\text{ Hz}$ à $50\text{ Hz}$ ($20\text{ ms}$ à $50\text{ ms}$ par cycle).
* **Payload : Capteurs $\to$ Contrôleur (`TelemetryMessage`)** :
  ```json
  {
    "timestamp_ms": 12450,
    "ultrasound_distance_mm": 245.5,
    "line_sensors": [0, 1, 0],
    "light_sensors": [512, 620],
    "ball_status": {
      "inside_cup": true,
      "distance_from_center_mm": 2.1
    }
  }
  ```
* **Payload : Contrôleur $\to$ Actionneurs (`CommandMessage`)** :
  ```json
  {
    "steering_angle_deg": -12.5,
    "target_speed_m_s": 0.22,
    "acceleration_m_s2": 0.50,
    "gear": "FORWARD",
    "emergency_brake": false
  }
  ```

---

## 4. Module Simulation (Blender & Godot)

### 4.1 Modélisation 3D (Blender)
* **Châssis PiCar** : Dimensions fidèles (empattement, voie des roues, position relative du train avant directionnel et du train arrière propulseur).
* **Support Bille & Coupelle** :
  * Diamètre de la cuvette : $40\text{ mm}$ ($r = 20\text{ mm}$).
  * Profondeur centrale : $1{,}50\text{ mm}$.
  * Rayon de courbure interne : $140\text{ mm}$.
  * Bille : Sphère rigide calibrée.
* **Colliders & Inertie** :
  * Définir le centre de gravité ($CoG$) réel du véhicule pour reproduire le roulis et le tangage lors des accélérations.
  * Exportation au format `glTF 2.0`.

### 4.2 Environnement Physique & Dynamique (Godot)
* **Contrôleur de véhicule (`VehicleBody3D` ou modèle cinématique personnalisé)** :
  * Angle de braquage limité selon les caractéristiques physiques du servo de direction.
  * Frottement pneu/sol paramétrable pour reproduire la friction du revêtement réel.
* **Physique de la bille (`RigidBody3D`)** :
  * Détection continue de perte de contact ou de franchissement de la bordure ($h > 1{,}50\text{ mm}$).
  * Émission d'un signal `ball_dropped` déclenchant une alerte immédiate.
* **Émulation des Capteurs** :
  * **Suiveur de ligne** : 3 `RayCast3D` verticaux pointant vers le sol, détectant la texture ou le calque de collision du ruban de $18\text{ mm}$.
  * **Capteur ultrasons** : Cône de détection angulaire (15° à 30°) via `ShapeCast3D` simulant la diffusion des ondes sonores et les réflexions sur les obstacles.
  * **Capteur de lumière** : Détection de l'intensité d'une source lumineuse (`OmniLight3D` ou `SpotLight3D`).
* **Circuits de test virtuels** :
  * Tracés fermés et ouverts avec virages de rayons variables ($R \ge 120\text{ mm}$, virages en S).
  * Lignes interrompues avec obstacles rectangulaires et cylindriques sur le tracé.
  * Ligne droite d'arrêt d'urgence avec obstacle terminal.

---

## 5. Module Traitement du Signal & Méthodes Numériques

### 5.1 Filtrage du Capteur Ultrasons
Le capteur ultrasons HC-SR04 présente du bruit haute fréquence, des pics parasites (glitches) et des faux rebonds :
1. **Filtre A (Filtre Médian glissant)** :
   * Fenêtre glissante de taille $N = 5$ ou $7$.
   * Élimination stricte des lectures aberrantes ponctuelles ($0\text{ mm}$ ou out-of-range).
2. **Filtre B (Moyenne Mobile exponentielle EMA ou Filtre Passe-Bas IIR)** :
   $$y_k = \alpha \cdot x_k + (1 - \alpha) \cdot y_{k-1}$$
   * Lissage du signal pour éviter les à-coups sur la régulation de vitesse.
3. **Filtre C (Kalman 1D - Option avancée)** :
   * Fusion entre la prédiction du modèle cinématique ($v \cdot \Delta t$) et la mesure du capteur ultrasons.
4. **Livrable d'évaluation** : Code d'exécution, courbes comparatives (Bruit vs Filtré) et étude du compromis latence/atténuation.

### 5.2 Étalonnage par Méthodes Numériques (Régression)
* **Banc d'essai** : Mesure de $M \ge 10$ points étalons sur une plage de $50\text{ mm}$ à $1000\text{ mm}$ avec règle millimétrique.
* **Modélisation mathématique** :
  * Résolution des équations normales ($A^T A \hat{x} = A^T b$) ou méthode de projection orthogonale (QR).
  * Détermination des paramètres de correction affine ou polynomiale :
    $$d_{\text{réelle}} = f(d_{\text{mesurée}}) = \beta_1 \cdot d_{\text{mesurée}} + \beta_0$$
* **Intégration** : Application directe de cette fonction de correction dans la boucle d'acquisition temps réel.

### 5.3 Analyse Statistique & Intervalles de Confiance
* Échantillonnage de $N \ge 30$ mesures pour au moins deux distances d'arrêt critiques (ex. : $150\text{ mm}$ et $300\text{ mm}$).
* Calcul de la moyenne empirique $\bar{x}$, de la variance $s^2$, et détermination de l'intervalle de confiance à 95 % (loi de Student / normale).
* Définition d'un seuil de décision d'arrêt sécuritaire intégrant la marge d'incertitude :
  $$d_{\text{seuil}} = d_{\text{cible}} + t_{\alpha/2} \frac{s}{\sqrt{N}}$$

---

## 6. Module Algorithmique & Contrôle Moteur

### 6.1 Régulateur de Vitesse & Rampes d'Accélération (Profileur Trapézoïdal)
* **Générateur de profil de vitesse** :
  * Calcul à chaque pas de temps $\Delta t$ de la consigne de vitesse :
    $$v_{k} = \min(v_{\text{cible}}, v_{k-1} + a_{\max} \cdot \Delta t) \quad \text{avec } a_{\max} \le 0{,}735\text{ m/s}^2$$
    $$v_{k} = \max(v_{\text{cible}}, v_{k-1} - a_{\max} \cdot \Delta t) \quad \text{en phase de freinage}$$
* **Freinage ciblé à l'arrêt (Précision $\pm 30\text{ mm}$)** :
  * Profil de décélération basé sur la distance restante $d$ :
    $$v_{\text{max,consigne}} = \sqrt{2 \cdot a_{\max} \cdot (d - d_{\text{cible}})}$$
  * Commutation vers l'arrêt complet lorsque $|d - d_{\text{cible}}| \le \epsilon$ sans blocage brusque des roues.

### 6.2 Asservissement Suivi de Ligne (Ligne 18 mm & Courbe $R \ge 120\text{ mm}$)
* Entrées : 3 capteurs optiques [Gauche ($L$), Centre ($C$), Droite ($R$)].
* **Algorithme d'alignement** :
  * État nominal : $C$ actif $\implies$ Angle de braquage $\theta = 0^\circ$.
  * Déviation légère : $L$ ou $R$ actif $\implies$ Correction proportionnelle $\theta = \pm K_p$.
  * Virage prononcé ($R \ge 120\text{ mm}$) : Maintien du braquage maximum jusqu'à ré-acquisition du capteur central, associé à une réduction immédiate de la consigne de vitesse ($v \le 0{,}25\text{ m/s}$).
  * Mécanisme de garde : Mémorisation de la dernière direction observée si les 3 capteurs perdent la ligne durant un court intervalle temporel ($< 150\text{ ms}$).

### 6.3 Machine à États Finis Principale (FSM)
L'automate à états finis structure l'intelligence de navigation :

```
                  ┌───────────────┐
                  │   IDLE /      │
                  │ INITIALISATION│
                  └───────┬───────┘
                          │ Démarrage
                          ▼
                  ┌───────────────┐
             ┌───►│ LINE_TRACKING │◄────────────────┐
             │    └───────┬───────┘                 │
             │            │                         │
Ligne perdue │            │ Obstacle détecté        │
  < 200 mm   │            │ à distance critique     │
             │            ▼                         │
     ┌───────┴──────┐   ┌───────────────────┐       │
     │ REVERSE_LINE │   │   DECELERATING    │       │
     │  (Recul S5)  │   │  (Freinage S1)    │       │
     └──────────────┘   └─────────┬─────────┘       │
                                  │                 │
                                  │ Arrêt stable    │
                                  │ ±30 mm atteint  │
                                  ▼                 │
                        ┌───────────────────┐       │
                        │ OBSTACLE_BYPASS   ├───────┘
                        │ (Contournement S4)│ Récupération de ligne
                        └───────────────────┘
```

1. **`LINE_TRACKING`** : Asservissement standard en direction le long de la ligne, avec modulation de vitesse selon la courbure.
2. **`DECELERATING`** : Détection d'un obstacle dans la trajectoire, réduction contrôlée de la vitesse selon la rampe $a \le 0{,}735\text{ m/s}^2$.
3. **`OBSTACLE_BYPASS`** :
   * Déviation latérale (sortie de ligne planifiée à vitesse réduite).
   * Dépassement longitudinal de l'obstacle en maintenant une distance de garde.
   * Retour vers la ligne-guide jusqu'à déclenchement des capteurs de ligne optiques.
4. **`REVERSE_LINE`** :
   * Manœuvre de marche arrière le long du ruban sur une distance bornée ($< 200\text{ mm}$) en inversant les gains de direction.
5. **`EMERGENCY_STOP`** : Arrêt contrôlé si anomalie grave ou consigne manuelle.

---

## 7. Stratégie de Tests & Validation Terrain

### 7.1 Matrice de Qualification (Tests Échelonnés)

| Étape | Domaine | Nature du test | Critère de succès (Pass/Fail) |
| :--- | :--- | :--- | :--- |
| **T1** | Physique | Accélération maximale en ligne droite | Bille maintenue sans vibration critique ($a \le 0{,}735\text{ m/s}^2$). |
| **T2** | Physique | Virage à rayon minimal ($R = 120\text{ mm}$) | Bille maintenue, aucun glissement hors coupelle. |
| **T3** | Capteurs | Banc de mesure ultrasons (50 à 800 mm) | Modèle d'étalonnage établi, bruit résiduel $< \pm 5\text{ mm}$. |
| **T4** | Logiciel | Arrêt d'obstacle à distance cible | Erreur de position $|d_{\text{mesurée}} - d_{\text{cible}}| \le 30\text{ mm}$ sur 10 essais consécutifs. |
| **T5** | Logiciel | Suivi de ligne et recul ($< 200\text{ mm}$) | Recul complet sur la ligne sans perte de contact capteur. |
| **T6** | Simulation | Épreuve complète Godot Grand Prix | Parcours virtuel bouclé en totale autonomie sans hardcoding. |
| **T7** | Réel | Séance *Conduite à l'épreuve* (4 trajets) | 4 parcours validés dans l'enveloppe de 20 minutes allouées. |
| **T8** | Compétition| Épreuve finale (*Que le meilleur serve !*) | Traversée sans faute de la piste inconnue (**piste mystère**) avec temps optimisé. |

### 7.2 Protocole de Validation Réel vs Simulation (Réduction de Divergence)
1. **Étalonnage des retards et latences** :
   * Mesurer la période d'échantillonnage de la boucle Raspberry Pi (I2C / GPIO) et injecter ce délai dans le cycle de calcul Godot.
2. **Ajustement de l'adhérence (Friction)** :
   * Comparer la distance de glissement réel lors d'un freinage d'urgence avec le modèle `VehicleBody3D` et corriger le coefficient de friction du matériau de piste.
3. **Validation croisée** :
   * Faire exécuter la même séquence de commandes en boucle ouverte dans Godot et sur le robot réel ; comparer les trajectoires résultantes.

---

## 8. Matrice de Traçabilité des Spécifications Techniques

| Code Exigence | Composant Logiciel / Module | Fichier ou Domaine Technique | Critère de Validation |
| :--- | :--- | :--- | :--- |
| **S1** (Arrêt $\pm 30\text{ mm}$) | Profil de décélération & Filtrage US | `services/Sensors`, `services/FSM` | Tolérance d'arrêt $\le \pm 30\text{ mm}$ |
| **S2** (Ligne $18\text{ mm}$) | Asservissement suiveur de ligne | `services/FSM`, `simulation` | Suivi continu sur ruban $18\text{ mm}$ |
| **S3** (Courbe $R \ge 120\text{ mm}$) | Régulation d'angle & vitesse virage | `services/FSM`, `services/Motors` | Courbe $120\text{ mm}$ franchie sans décrochage |
| **S4** (Contournement) | Sous-routine FSM `BYPASS` | `services/FSM`, `simulation` | Évitement obstacle et raccordement ligne |
| **S5** (Recul $< 200\text{ mm}$) | Routine FSM `REVERSE` | `services/FSM` | Recul contrôlé sur ruban noir |
| **S6** (Maintien bille) | Profileur d'accélération ($0{,}735\text{ m/s}^2$) | `services/Motors` | Bille conservée dans la soucoupe |
| **S7** (3 capteurs) | Module Hardware Abstraction Layer | `services/libs/shared-headers/hal` | Zéro capteur tiers utilisé |
| **S8** (Zéro hardcoding) | Architecture réactive & FSM | `services/FSM` | Navigation autonome sur piste mystère |
| **S9** (Simulation préalable)| Environnement Godot & Modèle Blender | `godot/` | Validation virtuelle préalable |
| **S10** (Symétrie code) | Cœur algorithmique partagé | `services/SimBridge` / SHM | Même algorithme en virtuel et en réel |
