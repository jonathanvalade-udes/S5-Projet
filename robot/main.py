"""
main.py
Serveur WebSocket asynchrone pour le robot PiCar-S et le simulateur Godot 4.
Assure un pont de communication bidirectionnel robuste et typé en JSON sur le port 8765.
"""

import asyncio
import json
import logging
import math
import os
import signal
import sys
from websockets.server import serve
from websockets.exceptions import ConnectionClosed

from motion_profiler import MotionProfiler
from line_follower import LineFollowerController
from ultrasonic_filters import make_filter

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("RobotServer")

# Réglages de vitesse (modifiables sans toucher au code : variables d'environnement SPEED_SCALE / ACCEL_SCALE)
# - SPEED_SCALE : multiplie la vitesse max et les vitesses de consigne du suiveur de ligne
#                 (1 = valeurs d'origine : 0,28 m/s max, 0,22 m/s de croisière).
# - ACCEL_SCALE : multiplie l'accélération/décélération max (1 = 0,735 m/s², seuil de renversement).
#                 Au-dessus de 1, la bille tombe dès le démarrage ou au freinage.
# Valeurs par défaut (2.0 / 3.0 / 1.0) optimisées en simulation sur le parcours : tour complet en ~23 s
# (contre ~45 s à l'origine) avec la bille à ~10 mm du centre au maximum (elle tombe à 20 mm).
# Au-delà (ex. SPEED_SCALE >= 2.5 ou CURVE_SCALE >= 3.3), la bille tombe dans les premiers virages.
SPEED_SCALE = float(os.environ.get("SPEED_SCALE", "2.0"))
ACCEL_SCALE = float(os.environ.get("ACCEL_SCALE", "1.0"))
# - LAT_LIMIT : accélération latérale maximale visée en virage (m/s²). La vitesse est plafonnée selon
#               l'angle de braquage : v <= sqrt(LAT_LIMIT * L / tan(braquage)). 0 = désactivé.
LAT_LIMIT = float(os.environ.get("LAT_LIMIT", "0"))
# - CURVE_SCALE : multiplie la vitesse minimale en virage serré (0,10 m/s d'origine). Réglé par essais en simulation (voir ci-dessous).
CURVE_SCALE = float(os.environ.get("CURVE_SCALE", "3.0"))
WHEELBASE = 0.14          # m, même valeur que la cinématique du simulateur
MAX_STEER_RAD = math.radians(48.0)
SERVER_PORT = int(os.environ.get("PORT", "8765"))

# Filtre appliqué à la distance du capteur à ultrasons (voir tools/filtres/comparaison.md) :
#   "median" : filtre médian à fenêtre glissante de taille FILTER_SIZE (choix retenu : rejette les valeurs aberrantes)
#   "iir"    : filtre passe-bas RII d'ordre 1 de coefficient FILTER_ALPHA
#   "none"   : aucun filtrage
DISTANCE_FILTER = os.environ.get("DISTANCE_FILTER", "median")
FILTER_SIZE = int(os.environ.get("FILTER_SIZE", "5"))
FILTER_ALPHA = float(os.environ.get("FILTER_ALPHA", "0.3"))

# État par défaut garanti pour éviter toute clé manquante
DEFAULT_STATE = {
    "distance": 100.0,
    "line_follower": [0, 0, 1, 0, 0],
    "velocity": 0.0,
    "direction": 0.0,
    "ball_dropped": false if "false" == "true" else False
}


class WebSocketBridge:
    def __init__(self):
        self.state = dict(DEFAULT_STATE)
        self.profiler = MotionProfiler(max_accel=0.735 * ACCEL_SCALE, max_speed=0.28 * SPEED_SCALE)
        self.distance_filter = make_filter(DISTANCE_FILTER, alpha=FILTER_ALPHA, size=FILTER_SIZE)
        self.line_follower = LineFollowerController(speed_scale=SPEED_SCALE, curve_scale=CURVE_SCALE)
        self.packet_count = 0
        self.active_clients = set()

    async def handle_client(self, websocket):
        client_address = websocket.remote_address
        self.active_clients.add(websocket)
        self.line_follower.reset()
        self.profiler.reset()
        self.distance_filter.reset()
        logger.info(f"Nouveau client connecté : {client_address} (Total clients: {len(self.active_clients)})")

        packet_in_session = 0
        try:
            async for raw_message in websocket:
                packet_in_session += 1
                self.packet_count += 1

                # Décodage JSON sécurisé
                try:
                    data = json.loads(raw_message)
                except (json.JSONDecodeError, TypeError) as err:
                    logger.warning(f"Trame JSON invalide reçue de {client_address}: {err}")
                    continue

                if not isinstance(data, dict):
                    logger.warning(f"Format non dictionnaire reçu de {client_address}: {type(data)}")
                    continue

                # Mise à jour de l'état interne
                if "distance" in data:
                    try:
                        filtered = self.distance_filter.update(float(data["distance"]))
                        if filtered is not None:
                            self.state["distance"] = filtered
                    except (ValueError, TypeError):
                        pass

                if "line_follower" in data and isinstance(data["line_follower"], list):
                    # Normalisation sur 5 éléments
                    lf = data["line_follower"]
                    self.state["line_follower"] = [int(v) for v in lf[:5]] + [0] * max(0, 5 - len(lf))

                # Surveillance de la bille
                if "ball_dropped" in data:
                    is_dropped = bool(data["ball_dropped"])
                    if is_dropped and not self.state.get("ball_dropped", False):
                        logger.warning("ALERTE : Déversement détecté ! La bille est tombée de la coupelle !")
                    self.state["ball_dropped"] = is_dropped

                # Calcul autonome de direction et de vitesse de virage
                computed_steer = None
                computed_target_vel = None
                if not self.state.get("ball_dropped", False) and "line_follower" in data:
                    computed_steer, computed_target_vel = self.line_follower.update(self.state["line_follower"])

                # Profilage de vitesse trapézoïdal (a <= 0.735 m/s²)
                target_vel = 0.0
                if not self.state.get("ball_dropped", False):
                    if "target_velocity" in data:
                        try:
                            target_vel = float(data["target_velocity"])
                        except (ValueError, TypeError):
                            target_vel = 0.0
                    elif computed_target_vel is not None:
                        target_vel = computed_target_vel
                    elif "velocity" in data:
                        try:
                            target_vel = float(data["velocity"])
                        except (ValueError, TypeError):
                            target_vel = 0.0
                    else:
                        target_vel = self.state["velocity"]
                else:
                    target_vel = 0.0  # Arrêt immédiat si la bille est tombée

                # Plafond de vitesse selon le braquage (limite l'accélération latérale subie par la bille)
                if LAT_LIMIT > 0 and computed_steer is not None:
                    tan_steer = max(math.tan(abs(computed_steer) * MAX_STEER_RAD), 1e-3)
                    target_vel = min(target_vel, math.sqrt(LAT_LIMIT * WHEELBASE / tan_steer))

                # Calcul de la vitesse réelle lissée selon a_max
                self.state["velocity"] = round(self.profiler.update(target_vel), 4)

                # Direction : priorité à la commande manuelle explicite si fournie, sinon calculée par le régulateur
                if "direction" in data:
                    try:
                        self.state["direction"] = float(data["direction"])
                    except (ValueError, TypeError):
                        pass
                elif computed_steer is not None:
                    self.state["direction"] = round(computed_steer, 3)

                # Préparation de la réponse garantie avec toutes les clés requises
                response = {
                    "status": "ok",
                    "distance": self.state["distance"],
                    "line_follower": self.state["line_follower"],
                    "velocity": self.state["velocity"],
                    "direction": self.state["direction"],
                    "ack_id": data.get("packet_id", packet_in_session)
                }

                await websocket.send(json.dumps(response))

                # Log périodique (tous les 50 paquets)
                if packet_in_session % 50 == 0:
                    logger.info(
                        f"[{client_address}] 50 paquets traités. "
                        f"Dernier état -> Dist: {self.state['distance']:.1f}cm, "
                        f"Ligne: {self.state['line_follower']}, "
                        f"Vit: {self.state['velocity']:.2f}, Dir: {self.state['direction']:.2f}"
                    )

        except ConnectionClosed:
            logger.info(f"Connexion fermée normalement par le client {client_address}")
        except Exception as e:
            logger.error(f"Erreur inattendue avec {client_address}: {e}")
        finally:
            self.active_clients.discard(websocket)
            logger.info(f"Client déconnecté : {client_address} (Clients restants: {len(self.active_clients)})")


async def main():
    bridge = WebSocketBridge()
    host = "0.0.0.0"
    port = SERVER_PORT

    logger.info(f"Démarrage du serveur WebSocket sur ws://{host}:{port} ...")
    async with serve(bridge.handle_client, host, port):
        logger.info(f"Serveur opérationnel et à l'écoute des connexions.")
        await asyncio.Future()  # En attente permanente


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Arrêt demandé par l'utilisateur (Ctrl+C). Arrêt propre du serveur.")
        sys.exit(0)