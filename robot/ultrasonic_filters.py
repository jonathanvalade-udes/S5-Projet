"""
Filtres du capteur de distance à ultrasons.

- LowPassIIR   : passe-bas RII d'ordre 1,  y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
- SlidingMedian: médian sur une fenêtre glissante de N échantillons (N impair), y[k] = x_((N+1)/2)

Interface commune : update(x) -> valeur filtrée, reset(), filter(signal) -> liste filtrée.
Une mesure invalide (None, NaN, <= 0 ; le capteur SunFounder renvoie -1 en cas d'erreur)
est ignorée : le filtre renvoie sa dernière sortie.
"""

import math
from collections import deque


def _est_valide(x):
    return x is not None and not math.isnan(x) and x > 0


class _Filtre:
    """Base commune : gère la mesure invalide et le traitement d'un signal complet."""

    def __init__(self):
        self.y = None

    def reset(self):
        self.y = None

    def update(self, x):
        if _est_valide(x):
            self.y = self._calculer(float(x))
        return self.y

    def filter(self, signal):
        self.reset()
        return [self.update(x) for x in signal]


class LowPassIIR(_Filtre):
    def __init__(self, alpha=0.3):
        super().__init__()
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha doit être dans ]0, 1]")
        self.alpha = alpha

    def _calculer(self, x):
        if self.y is None:
            return x  # y[0] = x[0] : pas de saut depuis zéro au démarrage
        return self.alpha * x + (1.0 - self.alpha) * self.y


class SlidingMedian(_Filtre):
    def __init__(self, size=5):
        super().__init__()
        if size < 1 or size % 2 == 0:
            raise ValueError("la taille de la fenêtre doit être un entier impair")
        self.window = deque(maxlen=size)

    def reset(self):
        super().reset()
        self.window.clear()

    def _calculer(self, x):
        self.window.append(x)
        return sorted(self.window)[len(self.window) // 2]  # élément du milieu


class PassThrough(_Filtre):
    """Aucun filtrage."""

    def _calculer(self, x):
        return x


def make_filter(name, alpha=0.3, size=5):
    """Crée un filtre à partir de son nom : 'median', 'iir' ou 'none'."""
    name = name.strip().lower()
    if name == "median":
        return SlidingMedian(size)
    if name == "iir":
        return LowPassIIR(alpha)
    if name == "none":
        return PassThrough()
    raise ValueError(f"filtre inconnu : {name!r} (choix : median, iir, none)")
