"""Suivi de la bille FastDrink physique (Solid DEF BALL du monde) dans le repere de la coupelle du PiCar.

Python pur, testable sans Webots. Le controleur (superviseur) lui donne a chaque pas la pose de la voiture et la position
de la bille ; la bille est tombee quand son centre depasse le rebord de la coupelle ou descend sous le fond.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

# Coupelle (PiCarS.proto, DEF CUP_PLATE, et cad/Plaquette.obj) dans le repere du robot (origine : centre de l'essieu arriere)
CUP_X = 0.048          # m devant l'essieu arriere
CUP_Z = 0.0735         # m au-dessus de l'origine du robot (dessous de la plaque)
CUP_BOTTOM = 0.0025    # fond de la coupelle au-dessus du dessous de la plaque
CUP_RIM_RADIUS = 0.020
BALL_RADIUS = 0.010
REST_Z = CUP_BOTTOM + BALL_RADIUS   # centre de la bille au repos, au-dessus du dessous de la plaque


def ball_in_cup_frame(car_pos: Sequence[float], car_rot: Sequence[float],
                      ball_pos: Sequence[float]) -> Tuple[float, float, float]:
    """Position de la bille (x avant, y gauche, z haut) par rapport a sa position de repos au centre de la coupelle (m).

    car_rot : matrice 3 x 3 de l'orientation de la voiture, en ligne (Supervisor Node.getOrientation()).
    """
    d = [ball_pos[i] - car_pos[i] for i in range(3)]
    # repere monde -> repere voiture : transposee de la rotation
    local = [sum(car_rot[3 * r + c] * d[r] for r in range(3)) for c in range(3)]
    return local[0] - CUP_X, local[1], local[2] - (CUP_Z + REST_Z)


def is_dropped(rel: Sequence[float]) -> bool:
    """Tombee : centre au-dela du rebord (r > 20 mm + 2 mm de marge sur le bourrelet) ou 5 mm sous sa position de repos."""
    return math.hypot(rel[0], rel[1]) > CUP_RIM_RADIUS + 0.002 or rel[2] < -0.005
