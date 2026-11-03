# 🤖 Simulation Robotique Professionnelle avec Webots (FastDrink S5-GI)

Ce dossier contient l'environnement complet de simulation sous **Webots** (le simulateur robotique open-source de référence de la *Linux Foundation*).

---

## 📁 Structure du Projet Webots

```
webots_sim/
├── worlds/
│   └── fastdrink_track.wbt        # Arène 2.2m x 2.2m, ruban noir 18mm, obstacle et robot
├── controllers/
│   └── picar_webots/
│       ├── Makefile               # Chaîne de compilation C++ native Webots
│       └── picar_webots.cpp       # Contrôleur C++ natif branché sur notre firmware
└── README.md
```

---

## ⚙️ Caractéristiques Techniques

1. **Capteurs Physiques Réels :**
   * **1x `DistanceSensor` (sonar)** modélisant fidèlement le cône acoustique du capteur ultrason HC-SR04.
   * **5x `DistanceSensor` (infrarouges)** orientés vers le sol et espacés de 20 mm pour détecter le ruban noir.
2. **Couplage Direct avec le Firmware C++ :**
   * Le contrôleur Webots inclut directement les headers de notre firmware C++ :
     * `firmware/include/filters/ultrasonic_pipeline.hpp` (Filtre Médian + Kalman 1D)
     * `firmware/include/control/motion_profiler.hpp` ($a \le 0{,}735\text{ m/s}^2$)
     * `firmware/include/control/line_follower.hpp` (Asservissement PD)
     * `firmware/include/control/optimized_params.hpp` (**Paramètres calculés par Optuna !**)
3. **Synchronisation IPC :**
   * Le contrôleur met également à jour la mémoire partagée `/dev/shm/picar_shm` pour permettre une supervision externe en temps réel.

---

## 🚀 Comment lancer la simulation sur Mac ?

### Étape 1 : Télécharger et Installer Webots
Téléchargez la version macOS pour Apple Silicon ou Intel directement depuis :
👉 **[Cyberbotics Webots Releases (GitHub)](https://github.com/cyberbotics/webots/releases)** (fichier `.dmg`).  
Glissez l'application `Webots.app` dans votre dossier `/Applications`.

### Étape 2 : Ouvrir le Monde de Simulation
1. Lancez **Webots**.
2. Allez dans le menu : **File > Open World...**
3. Sélectionnez le fichier :  
   `S5-Projet/webots_sim/worlds/fastdrink_track.wbt`
4. Cliquez sur le bouton **Play (▶️)** en haut de l'écran.

Le contrôleur compilera automatiquement `picar_webots.cpp` et fera rouler le robot avec les paramètres optimisés par Optuna !
