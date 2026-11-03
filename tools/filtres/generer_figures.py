import argparse
import csv
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RACINE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(RACINE, "robot"))
from ultrasonic_filters import LowPassIIR, SlidingMedian  # noqa: E402

SORTIE = os.path.dirname(os.path.abspath(__file__))  # tableau écrit à côté du script
DOSSIER_FIGURES = os.path.join(SORTIE, "figures")    # images PNG, dans un dossier à côté du script

# ==============================================================================================
# SIMULATION : réglages comparés et bruit simulé (hypothèses, à remplacer par des mesures réelles)
# ==============================================================================================
ALPHA = 0.3            # filtre RII
TAILLE_N = 5           # filtre médian (fenêtre de 5 échantillons)
TOLERANCE_MM = 30      # précision d'arrêt exigée par le client

FS = 20.0              # Hz, fréquence de mesure
SIGMA_MM = 8.0         # bruit gaussien
PROB_ABERRANTE = 0.03  # probabilité d'une valeur aberrante (décalage de 200 à 1000 mm)
VITESSE_MM_S = 250.0   # vitesse d'approche du scénario
NB_TIRAGES = 300

# ==============================================================================================
# CONFIGURATION DES GRAPHIQUES : un bloc par figure. Modifier ici suffit, le reste du code lit ces blocs.
# ==============================================================================================
STYLE_GENERAL = {"axes.grid": True, "grid.alpha": 0.25, "axes.spines.top": False,
                 "axes.spines.right": False, "figure.dpi": 150, "savefig.bbox": "tight"}

COULEUR_IIR = "#0d00fc"
COULEUR_MEDIAN = "#0b7a5e"
GRIS, NOIR = "#ff0000", "#111111"

# ---- Figure 1 : filtre RII (haut : approche d'un obstacle, bas : réponse à un échelon) ---------
FIG1_RII = {
    "fichier": "fig1_filtre_rii.png",
    "taille": (8, 6.2),                  # largeur, hauteur en pouces
    "couleur": COULEUR_IIR,              # couleur de la courbe filtrée et de la bande cible
    "titre_haut": "Filtre passe-bas RII d'ordre 1 : approche d'un obstacle",
    "titre_bas": f"Apparition d'un obstacle (600 mm → 300 mm), bande = cible ± {TOLERANCE_MM} mm",
    "xlabel": "Temps (s)",
    "ylabel": "Distance (mm)",
    "ylim": (0, 1000),
    "legende_filtre": f"Mesure filtrée",
    "legende_mesure": "Mesure brute",
    "legende_reel": "Distance réelle",
    "legende_pos": "upper right",
    "legende_taille": 8,
    "point_taille": 4,                   # taille des points de mesure
    "epaisseur_filtre": 2,
    "bande_opacite": 0.12,               # opacité de la bande cible ± tolérance
}

# ---- Figure 2 : filtre médian ------------------------------------------------------------------
FIG2_MEDIAN = {
    "fichier": "fig2_filtre_median.png",
    "taille": (8, 6.2),
    "couleur": COULEUR_MEDIAN,
    "titre_haut": "Filtre médian à fenêtre glissante : approche d'un obstacle",
    "titre_bas": f"Apparition d'un obstacle (600 mm → 300 mm), bande = cible ± {TOLERANCE_MM} mm",
    "xlabel": "Temps (s)",
    "ylabel": "Distance (mm)",
    "ylim": (0, 1000),
    "legende_filtre": f"Mesure filtrée",
    "legende_mesure": "Mesure brute",
    "legende_reel": "Distance réelle",
    "legende_pos": "upper right",
    "legende_taille": 8,
    "point_taille": 4,
    "epaisseur_filtre": 2,
    "bande_opacite": 0.12,
}

# ---- Figure 3 : comparaison des deux filtres (haut : sorties, bas : erreurs) -------------------
FIG3_COMPARAISON = {
    "fichier": "fig3_comparaison.png",
    "taille": (8, 6.6),
    "couleur_iir": COULEUR_IIR,
    "couleur_median": COULEUR_MEDIAN,
    "titre_haut": "Comparaison des deux filtres",
    "xlabel": "Temps (s)",
    "ylabel_haut": "Distance (mm)",
    "ylabel_bas": "Erreur (mm)",
    "ylim_haut": (0, 1000),
    "ylim_bas": (-150, 300),
    "legende_iir": f"Mesure filtrée RII d'ordre 1",
    "legende_median": f"Mesure filtrée Médian",
    "legende_mesure": "Mesure brute",
    "legende_reel": "Distance réelle",
    "legende_tolerance": f"± {TOLERANCE_MM} mm exigés",
    "legende_pos": "upper right",
    "legende_taille": 8,
    "point_taille": 4,
    "epaisseur_filtre": 2,
    "epaisseur_erreur": 1.6,
    "bande_opacite": 0.18,
}

# ---- Figure 4 : compromis bruit résiduel / retard selon les réglages ---------------------------
FIG4_COMPROMIS = {
    "fichier": "fig4_compromis_parametres.png",
    "taille": (7.2, 4.6),
    "couleur_iir": COULEUR_IIR,
    "couleur_median": COULEUR_MEDIAN,
    "titre": "Choix du réglage : plus on lisse, plus le filtre est lent\n"
             "Meilleur réglage = en bas à gauche (peu de bruit, peu de retard)",
    "xlabel": "Retard du filtre (ms) : plus à droite = plus lent à réagir",
    "ylabel": "Tremblement de la mesure à l'arrêt (mm)\nplus haut = plus instable",
    "legende_iir": "RII d'ordre 1 (le chiffre est α)",
    "legende_median": "Médian (le chiffre est N, taille de la fenêtre)",
    "legende_pos": "best",
    "legende_taille": 10,
    "reglages_iir": [0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0],   # valeurs de α testées
    "reglages_median": [1, 3, 5, 7, 9, 11],                  # tailles de fenêtre N testées
    "etiquette_taille": 7,               # taille du texte à côté de chaque point
    "tirages": 60,                       # signaux bruités moyennés par point
}


# ---- Signaux --------------------------------------------------------------------------------
def signal_approche(fs):
    """600 mm pendant 3 s, approche à VITESSE_MM_S jusqu'à 200 mm, arrêt jusqu'à 8 s."""
    t = np.arange(0, 8.0, 1.0 / fs)
    reel = np.where(t < 3.0, 600.0, np.maximum(600.0 - VITESSE_MM_S * (t - 3.0), 200.0))
    return t, reel


def signal_echelon(fs):
    """Un obstacle apparaît : 600 mm puis 300 mm à partir de t = 1 s."""
    t = np.arange(0, 4.0, 1.0 / fs)
    return t, np.where(t < 1.0, 600.0, 300.0)


def ajouter_bruit(reel, rng):
    mesure = reel + rng.normal(0.0, SIGMA_MM, reel.shape)
    aberrante = rng.random(reel.shape) < PROB_ABERRANTE
    decalage = rng.uniform(200, 1000, reel.shape) * rng.choice([-1, 1], reel.shape)
    return np.clip(np.where(aberrante, reel + decalage, mesure), 20.0, 4000.0)


def filtrer(nom, x, alpha=ALPHA, n=TAILLE_N):
    """Applique le filtre 'iir', 'med' ou 'brut' (aucun) au signal x."""
    if nom == "brut":
        return np.asarray(x, dtype=float)
    return np.array((LowPassIIR(alpha) if nom == "iir" else SlidingMedian(n)).filter(x))


def mesures_pour_figures(fs, chemin_csv):
    """(temps, distance réelle, mesure) : issus du CSV si fourni, sinon simulés."""
    if chemin_csv:
        with open(chemin_csv, newline="", encoding="utf-8") as f:
            lignes = list(csv.DictReader(f))
        x = np.array([float(l["mesure_mm"]) for l in lignes])
        reel = np.array([float(l["reel_mm"]) for l in lignes])
        return np.arange(len(x)) / fs, reel, x
    t, reel = signal_approche(fs)
    return t, reel, ajouter_bruit(reel, np.random.default_rng(7))


# ---- Métriques ------------------------------------------------------------------------------
def temps_reponse_ms(y, t, cible, debut):
    """Délai après `debut` pour entrer dans cible ± tolérance et y rester 3 échantillons."""
    dedans = np.abs(y - cible) <= TOLERANCE_MM
    for k in range(int(np.searchsorted(t, debut)), len(y) - 3):
        if dedans[k:k + 3].all():
            return (t[k] - debut) * 1000.0
    return np.nan


def metriques(nom, fs, alpha=ALPHA, n=TAILLE_N, tirages=NB_TIRAGES):
    """Moyennes sur `tirages` signaux bruités + réponse à une valeur aberrante isolée."""
    rng = np.random.default_rng(2026)
    t, reel = signal_approche(fs)
    te, reel_e = signal_echelon(fs)
    arret = t >= 6.0
    rampe = (t >= 3.8) & (t < 4.6)
    mesures = {k: [] for k in ("rmse", "ecart_type", "erreur_max", "hors_tol", "retard", "reponse")}
    for _ in range(tirages):
        y = filtrer(nom, ajouter_bruit(reel, rng), alpha, n)
        e = y - reel
        mesures["rmse"].append(np.sqrt(np.mean(e ** 2)))
        mesures["ecart_type"].append(np.std(y[arret]))
        mesures["erreur_max"].append(np.max(np.abs(e[arret])))
        mesures["hors_tol"].append(100.0 * np.mean(np.abs(e[arret]) > TOLERANCE_MM))
        mesures["retard"].append(np.mean(e[rampe]))
        ye = filtrer(nom, ajouter_bruit(reel_e, rng), alpha, n)
        mesures["reponse"].append(temps_reponse_ms(ye, te, 300.0, 1.0))
    res = {k: float(np.nanmean(v)) for k, v in mesures.items()}

    # Signal sans bruit avec une seule valeur aberrante de +800 mm : effet pur du filtre
    x = np.full(60, 300.0)
    x[20] += 800.0
    ecart = np.abs(filtrer(nom, x, alpha, n) - 300.0)
    res["pic"] = float(ecart.max())
    res["duree_hors_tol"] = float(np.sum(ecart > TOLERANCE_MM)) * 1000.0 / fs
    return res


# ---- Figures --------------------------------------------------------------------------------
def enregistrer(fig, fichier):
    """Sauvegarde la figure. Réessaie si le fichier est verrouillé un instant (synchronisation OneDrive)."""
    fig.tight_layout()
    for essai in range(5):
        try:
            fig.savefig(os.path.join(DOSSIER_FIGURES, fichier))
            break
        except OSError:
            if essai == 4:
                raise
            time.sleep(1)
    plt.close(fig)


def tracer_signal(ax, t, reel, x, courbes, cfg, ylim, ylabel):
    """Mesure brute, distance réelle et courbes filtrées [(y, légende, couleur), ...]."""
    ax.plot(t, x, ".", color=GRIS, ms=cfg["point_taille"], label=cfg["legende_mesure"])
    ax.plot(t, reel, "--", color=NOIR, lw=1.2, label=cfg["legende_reel"])
    for y, legende, couleur in courbes:
        ax.plot(t, y, color=couleur, lw=cfg["epaisseur_filtre"], label=legende)
    ax.set_ylim(*ylim)
    ax.set_ylabel(ylabel)
    ax.legend(loc=cfg["legende_pos"], fontsize=cfg["legende_taille"])


def figure_un_filtre(cfg, nom, fs, chemin_csv):
    legende, couleur = cfg["legende_filtre"], cfg["couleur"]
    t, reel, x = mesures_pour_figures(fs, chemin_csv)
    te, reel_e = signal_echelon(fs)
    xe = ajouter_bruit(reel_e, np.random.default_rng(11))

    fig, (haut, bas) = plt.subplots(2, 1, figsize=cfg["taille"])
    tracer_signal(haut, t, reel, x, [(filtrer(nom, x), legende, couleur)], cfg, cfg["ylim"], cfg["ylabel"])
    haut.set_title(cfg["titre_haut"])
    tracer_signal(bas, te, reel_e, xe, [(filtrer(nom, xe), legende, couleur)], cfg, cfg["ylim"], cfg["ylabel"])
    bas.axhspan(300 - TOLERANCE_MM, 300 + TOLERANCE_MM, color=cfg["couleur"], alpha=cfg["bande_opacite"])
    bas.set_title(cfg["titre_bas"])
    bas.set_xlabel(cfg["xlabel"])
    enregistrer(fig, cfg["fichier"])


def figure_comparaison(cfg, fs, chemin_csv):
    t, reel, x = mesures_pour_figures(fs, chemin_csv)
    sorties = [("iir", cfg["legende_iir"], cfg["couleur_iir"]), ("med", cfg["legende_median"], cfg["couleur_median"])]
    y = {nom: filtrer(nom, x) for nom, _, _ in sorties}

    fig, (haut, bas) = plt.subplots(2, 1, figsize=cfg["taille"], sharex=True)
    tracer_signal(haut, t, reel, x, [(y[nom], legende, couleur) for nom, legende, couleur in sorties],
                  cfg, cfg["ylim_haut"], cfg["ylabel_haut"])
    haut.set_title(cfg["titre_haut"])
    bas.axhspan(-TOLERANCE_MM, TOLERANCE_MM, color=GRIS, alpha=cfg["bande_opacite"], label=cfg["legende_tolerance"])
    for nom, legende, couleur in sorties:
        bas.plot(t, y[nom] - reel, color=couleur, lw=cfg["epaisseur_erreur"], label="Erreur " + legende)
    bas.set_ylim(*cfg["ylim_bas"])
    bas.set_xlabel(cfg["xlabel"])
    bas.set_ylabel(cfg["ylabel_bas"])
    bas.legend(loc=cfg["legende_pos"], fontsize=cfg["legende_taille"])
    enregistrer(fig, cfg["fichier"])


def figure_compromis(cfg, fs):
    """Bruit résiduel en fonction du retard pour plusieurs α et N : justifie le réglage choisi."""
    fig, ax = plt.subplots(figsize=cfg["taille"])
    series = [("iir", cfg["legende_iir"], cfg["couleur_iir"], cfg["reglages_iir"]),
              ("med", cfg["legende_median"], cfg["couleur_median"], cfg["reglages_median"])]
    for nom, legende, couleur, valeurs in series:
        points = []
        for v in valeurs:
            reglage = {"alpha": v} if nom == "iir" else {"n": v}
            m = metriques(nom, fs, tirages=cfg["tirages"], **reglage)
            points.append((m["retard"] / VITESSE_MM_S * 1000.0, m["ecart_type"]))  # retard : mm -> ms
        ax.plot(*zip(*points), "o-", color=couleur, label=legende)
        for v, point in zip(valeurs, points):
            ax.annotate(f"{v:g}", point, textcoords="offset points", xytext=(5, 5),
                        fontsize=cfg["etiquette_taille"], color=couleur)
    ax.set_xlabel(cfg["xlabel"])
    ax.set_ylabel(cfg["ylabel"])
    ax.set_title(cfg["titre"])
    ax.legend(loc=cfg["legende_pos"], fontsize=cfg["legende_taille"])
    enregistrer(fig, cfg["fichier"])


# ---- Tableau --------------------------------------------------------------------------------
CRITERES = [
    ("rmse", "Erreur quadratique moyenne (RMSE) sur le trajet", "mm"),
    ("ecart_type", "Écart-type de la sortie à l'arrêt", "mm"),
    ("erreur_max", "Erreur maximale à l'arrêt", "mm"),
    ("hors_tol", f"Échantillons hors ± {TOLERANCE_MM} mm à l'arrêt", "%"),
    ("retard", f"Retard (erreur moyenne) pendant l'approche à {VITESSE_MM_S:g} mm/s", "mm"),
    ("reponse", f"Temps pour atteindre la cible ± {TOLERANCE_MM} mm après un échelon", "ms"),
    ("pic", "Pic d'erreur causé par une valeur aberrante de 800 mm", "mm"),
    ("duree_hors_tol", f"Durée hors ± {TOLERANCE_MM} mm après cette valeur aberrante", "ms"),
]


def ecrire_tableau(fs):
    res = {nom: metriques(nom, fs) for nom in ("brut", "iir", "med")}
    lignes = [
        "# Comparaison des filtres : capteur à ultrasons", "",
        f"Mesure à {fs:g} Hz. Moyennes sur {NB_TIRAGES} tirages. Bruit simulé : gaussien σ = {SIGMA_MM:g} mm "
        f"et {100 * PROB_ABERRANTE:g} % de valeurs aberrantes (200 à 1000 mm).", "",
        f"| Critère | Unité | Sans filtre | RII (α = {ALPHA}) | Médian (N = {TAILLE_N}) |",
        "|---|:---:|:---:|:---:|:---:|",
    ]
    for cle, nom, unite in CRITERES:
        valeurs = " | ".join(f"{res[f][cle]:.1f}" for f in ("brut", "iir", "med"))
        lignes.append(f"| {nom} | {unite} | {valeurs} |")
    with open(os.path.join(SORTIE, "comparaison.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lignes) + "\n")
    print("\n".join(lignes))


def main():
    parseur = argparse.ArgumentParser(description="Figures et tableau comparatif des filtres.")
    parseur.add_argument("--csv", help="mesures réelles (colonnes mesure_mm, reel_mm) pour les figures")
    parseur.add_argument("--fs", type=float, default=FS, help="fréquence de mesure en Hz (défaut : 20)")
    args = parseur.parse_args()

    os.makedirs(DOSSIER_FIGURES, exist_ok=True)
    plt.rcParams.update(STYLE_GENERAL)
    figure_un_filtre(FIG1_RII, "iir", args.fs, args.csv)
    figure_un_filtre(FIG2_MEDIAN, "med", args.fs, args.csv)
    figure_comparaison(FIG3_COMPARAISON, args.fs, args.csv)
    figure_compromis(FIG4_COMPROMIS, args.fs)
    ecrire_tableau(args.fs)
    print("\nFigures écrites dans", DOSSIER_FIGURES)
    print("Tableau écrit dans", os.path.join(SORTIE, "comparaison.md"))


if __name__ == "__main__":
    main()
