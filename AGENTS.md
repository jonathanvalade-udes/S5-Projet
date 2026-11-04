# AGENTS.md

Instructions pour les agents IA (Claude Code, Antigravity, etc.) travaillant sur ce dépôt.

---

## Directives et Règles Strictes

1. **Un plan par prompt** : Toujours formuler et présenter un plan d'action clair et succinct avant d'exécuter des modifications importantes.
2. **Un seul service modifié à la fois** : Ne jamais modifier plusieurs services simultanément dans une même tâche (`FSM`, `Motors`, `Sensors`, `TelemetryService`, `ServiceManager`, `PeripheralsService`, `SimBridge`).
3. **Simplicité du code & threads limités** : Garder le code simple et lisible. Limiter au maximum la création et l'usage de threads (éviter la complexité concurrente non essentielle).
4. **Préservation du temps réel (RT)** :
   - Respecter scrupuleusement les contraintes temporelles et de latence (déterminisme, boucle de contrôle, périodicité).
   - Proscrire les opérations bloquantes ou non prédictives dans les chemins critiques temps réel.
5. **Modularité des fonctions et classes** :
   - Respecter le principe de responsabilité unique (SRP).
   - Fonctions courtes, découplées, facilement testables unitairement.
6. **Bibliothèques partagées intouchables** :
   - Ne **JAMAIS** modifier les headers et bibliothèques partagées (`services/libs/shared-headers/` et `services/libs/`).
   - L'interface IPC et les structures de données partagées doivent demeurer stables.
7. **Autorisation explicite pour le push Git** :
   - Ne jamais exécuter `git push` sans l'accord explicite préalable de l'utilisateur.

---

## Commandes Usuelles

Le projet utilise C++17/20 sous CMake avec support Docker pour cross-compilation ARM64 (Raspberry Pi) et simulation Godot.

Toutes les commandes d'ingénierie se lancent depuis le dossier `services/` via `./build.sh` :

- **Tester (tests unitaires Linux / Docker)** :
  ```bash
  cd services && ./build.sh --test
  ```
- **Compiler un seul service en local (ex: FSM)** :
  ```bash
  cd services && ./build.sh FSM
  ```
- **Compiler tous les services pour cible embarquée (ARM64 statique)** :
  ```bash
  cd services && ./build.sh --embedded --static
  ```
- **Lancer la pile de simulation Godot (SimBridge + FSM)** :
  ```bash
  cd services && ./build.sh --sim
  ```
- **Nettoyer les builds** :
  ```bash
  cd services && ./build.sh --clean
  ```

---

## Structure du Dépôt

- `services/` : Services C++ autonomes communiquant via IPC / sockets Unix.
  - `FSM/` : Machine à états finis / contrôle principal.
  - `Motors/` : Pilotage moteurs / direction.
  - `Sensors/` : Capteurs suiveur de ligne / périphériques.
  - `SimBridge/` : Passerelle réseau vers Godot.
  - `TelemetryService/` : Télémétrie et métriques.
  - `PeripheralsService/` : Gestion des actionneurs et périphériques matériels.
  - `ServiceManager/` : Supervision du cycle de vie des services.
  - `libs/shared-headers/` : **GELÉ / NE PAS MODIFIER**. Définitions des types IPC et structures partagées.
  - `tests/` : Tests unitaires (`ctest` / Google Test).
- `godot/` : Scènes et environnement de simulation Godot.
- `tools/` : Scripts de calibrage, test matériel et télémétrie.
