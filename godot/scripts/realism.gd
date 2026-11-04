class_name Realism
extends RefCounted

# Imperfections du vrai PiCar, pour que la simulation ne soit pas "trop parfaite".
# - Désactivé par défaut : la voiture est parfaite. Touche R en jeu : active / désactive toutes les imperfections.
# - Variables d'environnement (utile pour les tests) : REALISM=1 pour démarrer avec les imperfections, REALISM_SEED=<entier>,
#   et REALISM_<NOM_EN_MAJUSCULES>=<valeur> pour changer un réglage (ex. REALISM_STEER_BIAS_DEG=6).
# Le tirage aléatoire est initialisé avec une graine fixe : deux lancements donnent le même parcours.

static var enabled: bool = false
static var rng: RandomNumberGenerator = RandomNumberGenerator.new()
static var _configured: bool = false

static var p: Dictionary = {
	# --- Quantité de bruit aléatoire : 0 = aucun, 1 = valeurs ci-dessous, 2 = le double ---
	# Multiplie seulement les réglages "bruit" (marqués [bruit]). Les défauts fixes (biais, jeu, latence,
	# gain moteur) ne sont pas touchés : ce sont eux qui font dévier la voiture de façon régulière.
	"noise_scale": 1.0,
	# --- Direction ---
	"steer_bias_deg": 4.0,        # roues mal alignées : la voiture tourne toujours un peu du même côté
	"steer_backlash_deg": 2.0,    # jeu du servo : il faut un petit écart avant que la roue bouge
	"steer_noise_deg": 0.3,       # [bruit] dérive lente de l'angle de braquage (constante de temps 0,4 s)
	"servo_speed_dps": 400.0,     # vitesse max du servo (°/s) : il ne tourne pas instantanément
	# --- Moteurs ---
	"motor_gain": 0.93,           # la voiture roule à 93 % de la vitesse demandée
	"motor_noise": 0.03,          # [bruit] variation lente de la vitesse (± 3 %)
	# --- Communication ---
	"latency_s": 0.06,            # délai entre l'ordre de Python et son effet sur la voiture
	# --- Imprévus ---
	"kick_interval_s": 6.0,       # un petit choc (bosse, gravier) en moyenne toutes les 6 s...
	"kick_yaw_deg": 3.0,          # [bruit] ...qui fait pivoter la voiture d'au plus 3°
	# --- Capteurs ---
	"line_flip_prob": 0.002,      # [bruit] probabilité qu'un capteur de ligne se trompe (par capteur et par image)
	"ultrasonic_sigma_mm": 8.0,   # [bruit] bruit gaussien du capteur ultrason
	"ultrasonic_outlier_prob": 0.03,  # [bruit] probabilité d'une valeur aberrante (200 à 1000 mm d'écart)
}

# Lit les variables d'environnement une seule fois
static func setup() -> void:
	if _configured:
		return
	_configured = true
	var env_enabled: String = OS.get_environment("REALISM")
	if env_enabled != "":
		enabled = env_enabled != "0"
	var env_seed: String = OS.get_environment("REALISM_SEED")
	rng.seed = int(env_seed) if env_seed != "" else 12345
	for key in p.keys():
		var value: String = OS.get_environment("REALISM_" + String(key).to_upper())
		if value != "":
			p[key] = float(value)
	print("[Realism] ", "ACTIF" if enabled else "désactivé", " (touche R pour ", "désactiver" if enabled else "activer", ")")

static func toggle() -> void:
	enabled = not enabled
	print("[Realism] ", "ACTIF" if enabled else "désactivé")

# Nombre aléatoire gaussien de moyenne 0 et d'écart-type sigma
static func gauss(sigma: float) -> float:
	return rng.randfn(0.0, sigma)

# Valeur d'un réglage [bruit], multipliée par noise_scale
static func noise(key: String) -> float:
	return float(p[key]) * float(p["noise_scale"])

# Distance ultrason (m) avec bruit gaussien et valeurs aberrantes occasionnelles
static func noisy_distance(distance_m: float) -> float:
	if not enabled:
		return distance_m
	var d: float = distance_m + gauss(noise("ultrasonic_sigma_mm") / 1000.0)
	if rng.randf() < noise("ultrasonic_outlier_prob"):
		d += rng.randf_range(0.2, 1.0) * (1.0 if rng.randf() < 0.5 else -1.0)
	return clampf(d, 0.02, 4.0)

# Capteur de ligne (0 ou 1) qui se trompe parfois
static func noisy_line_bit(bit: int) -> int:
	if enabled and rng.randf() < noise("line_flip_prob"):
		return 1 - bit
	return bit
